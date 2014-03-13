//////////////////////////////////////////////////////////////////////////////////
// A very simple SPI implementation (synchronous)
// This expects SPI Mode 1 (CPOL = 0, CPHA = 1)
//
//////////////////////////////////////////////////////////////////////////////////

module fcl_spi_phy
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
	output	reg				spi_miso_out,
	
	// Bus Signals
	output	wire			rx_sync_out,
	output	reg		[7:0]	rx_data_out,
	output	reg				rx_data_valid_out,
	input	wire	[7:0]	tx_data_in,
	input	wire			tx_data_valid_in
);

	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//					Signals and Variables					\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	reg				spi_clk_buf;		// SPI Input Buffer
	reg				spi_mosi_buf;		// SPI Input Buffer
	reg				spi_ncs_buf;		// SPI Input Buffer
	reg				spi_clk_re_buf;		// SPI Input Buffer
	reg				spi_clk_fe_buf;		// SPI Input Buffer
	wire			spi_clk_re;			// Clock Rising Edge Signal
	wire			spi_clk_fe;			// Clock Rising Edge Signal
	reg		[7:0]	rx_data_shift;		// Received Data Shift Register
	reg		[2:0]	rx_bit_count;
	reg		[7:0]	tx_data_buf;
	
	
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//						Input Buffering						\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			spi_clk_buf <= 1'b0;
			spi_mosi_buf <= 1'b0;
			spi_ncs_buf <= 1'b1;
		end
		else begin
			spi_clk_buf <= spi_clk_in;
			spi_mosi_buf <= spi_mosi_in;
			spi_ncs_buf <= spi_ncs_in;
		end
	end
	
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//					Clock Edge Detect Logic					\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			spi_clk_re_buf <= 1'b1;
			spi_clk_fe_buf <= 1'b0;
		end
		else begin
			spi_clk_re_buf <= spi_clk_buf;
			spi_clk_fe_buf <= spi_clk_buf;
		end
	end
	
	assign spi_clk_re = (!spi_ncs_buf) && ((!spi_clk_re_buf) && spi_clk_buf);
	assign spi_clk_fe = (!spi_ncs_buf) && (spi_clk_fe_buf && (!spi_clk_buf));
	
	
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//					Rx Data Path Logic						\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			rx_data_shift <= 8'h00;
			rx_bit_count <= 3'b000;
			rx_data_out <= 8'h00;
			rx_data_valid_out <= 1'b0;
		end
		else begin
			if (spi_ncs_buf) begin
				rx_data_shift <= 8'h00;
				rx_bit_count <= 3'b000;
				rx_data_valid_out <= 1'b0;
			end
			else if (spi_clk_fe) begin
				rx_data_shift[7:0] <= {rx_data_shift[6:0], spi_mosi_buf};
				rx_bit_count <= rx_bit_count + 1'b1;
				if (&rx_bit_count) begin
					rx_data_out[7:0] <= {rx_data_shift[6:0], spi_mosi_buf};
					rx_data_valid_out <= 1'b1;
				end
				else begin
					rx_data_valid_out <= 1'b0;
				end
			end
			else begin
				rx_data_valid_out <= 1'b0;
			end
		end
	end
	
	assign rx_sync_out = spi_ncs_buf;
	
	
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	//					Tx Data Path Logic						\\
	//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><>\\
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			tx_data_buf <= 8'h00;
			spi_miso_out <= 1'b0;
		end
		else begin
			if (tx_data_valid_in) begin
				tx_data_buf <= tx_data_in;
			end
			
			if (spi_clk_re) begin
				spi_miso_out <= tx_data_buf[3'b111 - rx_bit_count];
			end
		end
	end
	
endmodule
