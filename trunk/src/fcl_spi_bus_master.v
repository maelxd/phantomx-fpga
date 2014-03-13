
module fcl_spi_bus_master
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//							Ports							\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
(
	// System Signals
	input	wire			_reset_in,
	input	wire			clk_in,
	
	// SPI Interface Signals
	input	wire			spi_clk_in,
	input	wire			spi_ncs_in,
	input	wire			spi_mosi_in,
	output	wire			spi_miso_out,
	
	// Bus Master Signals
	input	wire	[15:0]	bus_data_in,
	output	reg		[15:0]	bus_data_out,
	output	reg		[15:0]	bus_addr_out,
	output	wire			bus_read_out,
	output	wire			bus_write_out,
	input	wire			bus_ack_in
);

	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//					Signals and Variables					\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	localparam	[4:0]	ss_idle			= 5'h00;
	localparam	[4:0]	ss_addr0		= 5'h01;
	localparam	[4:0]	ss_addr1		= 5'h02;
	localparam	[4:0]	ss_pre_data0	= 5'h03;
	localparam	[4:0]	ss_pre_data1	= 5'h04;
	localparam	[4:0]	ss_chksum_check	= 5'h05;
	localparam	[4:0]	ss_cmd_rd0		= 5'h06;
	localparam	[4:0]	ss_cmd_rd1		= 5'h07;
	localparam	[4:0]	ss_cmd_rd2		= 5'h08;
	localparam	[4:0]	ss_cmd_rd3		= 5'h09;
	localparam	[4:0]	ss_cmd_rd4		= 5'h0A;
	localparam	[4:0]	ss_cmd_rd5		= 5'h0B;
	localparam	[4:0]	ss_cmd_rd6		= 5'h0C;
	localparam	[4:0]	ss_cmd_rd7		= 5'h0D;
	localparam	[4:0]	ss_cmd_wr0		= 5'h0E;
	localparam	[4:0]	ss_cmd_wr1		= 5'h0F;
	localparam	[4:0]	ss_finish		= 5'h10;
	
	
	
	
	wire			rx_sync;
	wire	[7:0]	rx_data;
	wire			rx_data_valid;
	reg		[7:0]	tx_data;
	wire			tx_data_valid;
	
	reg		[4:0]	control_state;
	reg		[7:0]	check_sum;
	reg		[4:0]	data_length;
	reg		[4:0]	word_counter;
	reg				write_n_read;
	reg		[15:0]	start_addr;
	
	reg		[7:0]	data_write_buf;
	reg		[15:0]	data_buffer	[31:0];
	reg		[7:0]	data_read_buf;
	
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			control_state <= ss_idle;
		end
		else begin
			case (control_state)
				ss_idle	: begin
					tx_data[7:0] <= 8'h00;
					if (rx_data_valid) begin
						check_sum[7:0] <= rx_data[7:0];
						data_length[4:0] <= rx_data[4:0];
						write_n_read <= rx_data[7];
						word_counter <= 5'h00;
						control_state <= ss_addr0;
					end
				end
				ss_addr0	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						check_sum[7:0] <= check_sum[7:0] + rx_data[7:0];
						start_addr[15:8] <= rx_data[7:0];
						control_state <= ss_addr1;
					end
				end
				ss_addr1	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						check_sum[7:0] <= check_sum[7:0] + rx_data[7:0];
						start_addr[7:0] <= rx_data[7:0];
						if (!write_n_read) begin
							control_state <= ss_chksum_check;
						end
						else begin
							control_state <= ss_pre_data0;
						end
					end
				end
				ss_pre_data0	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						check_sum[7:0] <= check_sum[7:0] + rx_data[7:0];
						data_write_buf[7:0] <= rx_data[7:0];	// Required because of Xilinx issues
						control_state <= ss_pre_data1;
					end
				end
				ss_pre_data1	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						check_sum[7:0] <= check_sum[7:0] + rx_data[7:0];
						word_counter <= word_counter + 1'b1;
						data_buffer[word_counter][15:0] <= {data_write_buf[7:0], rx_data[7:0]};
						if (word_counter == data_length) begin
							control_state <= ss_chksum_check;
						end
						else begin
							control_state <= ss_pre_data0;
						end
					end
				end
				ss_chksum_check	: begin
					if (rx_sync) begin	// Wait until the end of this corrupt packet
						control_state <= ss_idle;
					end
					else if (rx_data_valid && (&(check_sum[7:0] + rx_data[7:0]))) begin
						bus_addr_out[15:0] <= start_addr[15:0];
						word_counter <= 5'h01;
						if (!write_n_read) begin
							check_sum[7:0] <= 8'h00;
							control_state <= ss_cmd_rd0;
						end
						else begin
							bus_data_out[15:0] <= data_buffer[5'h00][15:0];
							control_state <= ss_cmd_wr0;
						end
					end
				end 
				ss_cmd_rd0	: begin
					if (bus_ack_in) begin
						check_sum[7:0] <= check_sum[7:0] + bus_data_in[15:8];
						tx_data[7:0] <= bus_data_in[15:8];
						data_read_buf[7:0] <= bus_data_in[7:0];
						control_state <= ss_cmd_rd2;
					end
					else begin
						control_state <= ss_cmd_rd1;
					end
				end 
				ss_cmd_rd1	: begin
					if (bus_ack_in) begin
						check_sum[7:0] <= check_sum[7:0] + bus_data_in[15:8];
						tx_data[7:0] <= bus_data_in[15:8];
						data_read_buf[7:0] <= bus_data_in[7:0];
						control_state <= ss_cmd_rd2;
					end
				end 
				ss_cmd_rd2	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						control_state <= ss_cmd_rd3;
					end
				end 
				ss_cmd_rd3	: begin
					control_state <= ss_cmd_rd4;
				end 
				ss_cmd_rd4	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						tx_data[7:0] <= data_read_buf[7:0];
						check_sum[7:0] <= check_sum[7:0] + data_read_buf[7:0];
						control_state <= ss_cmd_rd5;
					end
				end 
				ss_cmd_rd5	: begin
					if (word_counter == (data_length + 1'b1)) begin
						control_state <= ss_cmd_rd6;
					end
					else begin
						word_counter <= word_counter + 1'b1;
						bus_addr_out[15:0] <= bus_addr_out[15:0] + 1'b1;
						control_state <= ss_cmd_rd0;
					end
				end 
				ss_cmd_rd6	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
					else if (rx_data_valid) begin
						tx_data[7:0] <= 8'hFF - check_sum[7:0];
						control_state <= ss_cmd_rd7;
					end
				end 
				ss_cmd_rd7	: begin
					control_state <= ss_finish;
				end 
				
				ss_cmd_wr0	: begin
					if (bus_ack_in) begin
						if (word_counter == (data_length + 1'b1)) begin
							control_state <= ss_finish;
						end
						else begin
							bus_addr_out[15:0] <= bus_addr_out[15:0] + 1'b1;
							bus_data_out[15:0] <= data_buffer[word_counter][15:0];
							word_counter <= word_counter + 1'b1;
							control_state <= ss_cmd_wr0;
						end
					end
					else begin
						control_state <= ss_cmd_wr1;
					end
				end 
				ss_cmd_wr1	: begin
					if (bus_ack_in) begin
						if (word_counter == (data_length + 1'b1)) begin
							control_state <= ss_finish;
						end
						else begin
							bus_addr_out[15:0] <= bus_addr_out[15:0] + 1'b1;
							bus_data_out[15:0] <= data_buffer[word_counter][15:0];
							word_counter <= word_counter + 1'b1;
							control_state <= ss_cmd_wr0;
						end
					end
				end 
				ss_finish	: begin
					if (rx_sync) begin
						control_state <= ss_idle;
					end
				end 
				
				default	: begin
					control_state <= ss_idle;
				end
			endcase
		end
	end
	
	
	assign bus_write_out = (control_state == ss_cmd_wr0);
	assign bus_read_out = (control_state == ss_cmd_rd0);
	assign tx_data_valid = ((control_state == ss_idle) || (control_state == ss_cmd_rd3) || (control_state == ss_cmd_rd5) || (control_state == ss_cmd_rd7));
	

	fcl_spi_phy
		phy (
			._reset_in			(_reset_in),
			.clk_in				(clk_in),
			
			.spi_clk_in			(spi_clk_in),
			.spi_ncs_in			(spi_ncs_in),
			.spi_mosi_in		(spi_mosi_in),
			.spi_miso_out		(spi_miso_out),
			
			.rx_sync_out		(rx_sync),
			.rx_data_out		(rx_data),
			.rx_data_valid_out	(rx_data_valid),
			.tx_data_in			(tx_data),
			.tx_data_valid_in	(tx_data_valid));


	
endmodule
