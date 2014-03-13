
// A module written to test Dynamixel Servos

module fcl_servo_test
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 	INPUT_CLOCK_SPEED = 125000000,
	parameter integer 	DYNA_CLOCK_SPEED = 1000000,
	parameter integer 	DNET_ADDR_WIDTH = 16,
	parameter integer 	DNET_DATA_WIDTH = 32,
	parameter integer 	DNET_OFFSET = 0,
	parameter integer 	NUM_DYNAMIXELS = 1
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset,
    input 	wire			sys_clk,
	
	// Dynamixel Signals
    inout 	wire	[(NUM_DYNAMIXELS-1):0]	dyna_sda_inout,
	
	// DNET Signals
	output	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_out,
	input	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_in,
	input	wire  	[(DNET_ADDR_WIDTH-1):0] dnet_addr_in,
	input 	wire 							dnet_read,
	input 	wire 							dnet_write,
	output	reg 							dnet_ack
);



	localparam [4:0] dyc_idle		= 5'h00;
	localparam [4:0] dyc_start_0	= 5'h01;
	localparam [4:0] dyc_start_1	= 5'h02;
	localparam [4:0] dyc_start_2	= 5'h03;
	localparam [4:0] dyc_start_3	= 5'h04;
	localparam [4:0] dyc_id_0		= 5'h05;
	localparam [4:0] dyc_id_1		= 5'h06;
	localparam [4:0] dyc_length_0	= 5'h07;
	localparam [4:0] dyc_length_1	= 5'h08;
	localparam [4:0] dyc_inst_0		= 5'h09;
	localparam [4:0] dyc_inst_1		= 5'h0A;
	localparam [4:0] dyc_reg_addr_0	= 5'h0B;
	localparam [4:0] dyc_reg_addr_1	= 5'h0C;
	localparam [4:0] dyc_reg_data_0	= 5'h0D;
	localparam [4:0] dyc_reg_data_1	= 5'h0E;
	localparam [4:0] dyc_reg_data_2 = 5'h0F;
	localparam [4:0] dyc_reg_data_3	= 5'h10;
	localparam [4:0] dyc_chksum_0	= 5'h11;
	localparam [4:0] dyc_chksum_1	= 5'h12;
	localparam [4:0] dyc_read_0		= 5'h13;
	localparam [4:0] dyc_read_1		= 5'h14;
	localparam [4:0] dyc_read_2		= 5'h15;
	localparam [4:0] dyc_read_3		= 5'h16;
	localparam [4:0] dyc_read_4		= 5'h17;
	localparam [4:0] dyc_read_5		= 5'h18;
	localparam [4:0] dyc_read_6		= 5'h19;

	reg [(DNET_DATA_WIDTH-1):0] dnet_data_in_buf; 
	reg [(DNET_ADDR_WIDTH-1):0] dnet_addr_in_buf; 
	reg  dnet_read_buf; 
	reg  dnet_write_buf; 
	
	reg	[4:0]	dyna_control;	
	reg			dyna_mode_rnw;
	reg	[7:0]	dyna_checksum;
	reg [7:0] dyna_reg_id;
	reg [7:0] dyna_reg_addr;
	reg [15:0] dyna_reg_data_in;
	reg 	 dyna_reg_length_in;

	reg [7:0] uart_tx_data;
	wire [7:0] uart_rx_data;
	reg [31:0] uart_rx_data_buf;
	wire uart_rx_data_valid;
	reg tx_data_send;
	wire uart_tx_done;
	
	wire uart_sda_rx;
	wire uart_sda_tx;
	
	wire start_node;

	
	//------------------------------------------------------
	//--				 	 Logic 				 		  --
	//------------------------------------------------------
    fcl_uart_bidir  #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.BAUD_RATE(DYNA_CLOCK_SPEED),
		.USE_TRI_BUFFER(0))
	serial_port (
		._reset_in(_reset), 
		.clk_in(sys_clk),
        .uart_sda_in(uart_sda_rx),
        .uart_sda_out(uart_sda_tx),
		.rx_data_out(uart_rx_data), 
		.rx_data_valid_out(uart_rx_data_valid),
		.tx_data_in(uart_tx_data), 
		.tx_data_send_in(tx_data_send),
		.tx_done_out(uart_tx_done), 
		.tx_busy_out());
	
	assign dyna_sda_inout[(NUM_DYNAMIXELS-1):0] = uart_sda_tx ? {NUM_DYNAMIXELS{1'bz}} : {NUM_DYNAMIXELS{1'b0}};
	assign uart_sda_rx = &dyna_sda_inout[(NUM_DYNAMIXELS-1):0];
	

	//------------------------------------------------------
	//--				DNET Interface Logic		 	  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			dnet_data_in_buf <= {DNET_DATA_WIDTH{1'b0}};
			dnet_addr_in_buf <= {DNET_ADDR_WIDTH{1'b0}};
			dnet_read_buf <= 1'b0;
			dnet_write_buf <= 1'b0;
		end
		else begin
			dnet_data_in_buf <= dnet_data_in;
			dnet_addr_in_buf <= dnet_addr_in;
			dnet_read_buf <= dnet_read;
			dnet_write_buf <= dnet_write;
		end
	end

	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			dyna_reg_id <= {8{1'b0}};
			dyna_reg_addr <= {8{1'b0}};
			dyna_reg_data_in <= {16{1'b0}};
			dyna_reg_length_in <= 1'b0;
		end
		else begin
			if (dyna_control == dyc_idle) begin
				if (dnet_write_buf||dnet_read_buf) begin
					dyna_reg_id[7:0] <= dnet_addr_in_buf[15:8];
					dyna_reg_addr[7:0] <= dnet_addr_in_buf[7:0];
					dyna_reg_data_in[15:0] <= dnet_data_in_buf[15:0];
					dyna_reg_length_in <= dnet_data_in_buf[31];
				end
			end
		end
	end

	
	assign dnet_data_out = (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):24] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):24]) ? uart_rx_data_buf[31:0] : {DNET_DATA_WIDTH{1'b0}};
	
	
	
	assign start_node = ((dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):24] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):24]) && (dnet_write_buf || dnet_read_buf));

	
	
	//------------------------------------------------------
	//--			Servo State Machine Logic		 	  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
            dyna_control <= dyc_idle;
			dyna_mode_rnw <= 1'b0;
			uart_rx_data_buf <= {32{1'b0}};
			dnet_ack <= 1'b0;
        end 
		else begin
			if (start_node) begin	// A new start aborts the current operation (better hope this doesn't result in a write / read collide)
				dnet_ack <= 1'b0;
				if (dnet_write_buf) begin
					dyna_mode_rnw <= 1'b0;
					dyna_control <= dyc_start_0;
				end 
				else if (dnet_read_buf) begin
					dyna_mode_rnw <= 1'b1;
					dyna_control <= dyc_start_0;
				end
			end
			else begin
				case(dyna_control)
					dyc_idle: begin
						dnet_ack <= 1'b0;
					end
					
					// Send Header
					dyc_start_0: begin	
						dyna_control <= dyc_start_1;
					end
					dyc_start_1: begin	
						if (uart_tx_done)
							dyna_control <= dyc_start_2;
					end
					dyc_start_2: begin	
						dyna_control <= dyc_start_3;
					end
					dyc_start_3: begin	
						if (uart_tx_done)
							dyna_control <= dyc_id_0;
					end
					dyc_id_0: begin	
						dyna_control <= dyc_id_1;
					end
					dyc_id_1: begin	
						if (uart_tx_done) begin
							dyna_control <= dyc_length_0;
						end
					end
					dyc_length_0: begin	
						dyna_control <= dyc_length_1;
					end
					dyc_length_1: begin	
						if (uart_tx_done) begin
							dyna_control <= dyc_inst_0;
						end
					end
					dyc_inst_0: begin	
						dyna_control <= dyc_inst_1;
					end
					dyc_inst_1: begin	
						if (uart_tx_done) begin
							dyna_control <= dyc_reg_addr_0;
						end
					end
					dyc_reg_addr_0: begin	
						dyna_control <= dyc_reg_addr_1;
					end
					dyc_reg_addr_1: begin	
						if (uart_tx_done) begin
							dyna_control <= dyc_reg_data_0;
						end
					end
					dyc_reg_data_0: begin	
						dyna_control <= dyc_reg_data_1;
					end
					dyc_reg_data_1: begin	
						if (uart_tx_done) begin
							dyna_control <= (dyna_reg_length_in && (!dyna_mode_rnw)) ? dyc_reg_data_2 : dyc_chksum_0;
						end
					end
					dyc_reg_data_2: begin	
						dyna_control <= dyc_reg_data_3;
					end
					dyc_reg_data_3: begin	
						if (uart_tx_done) begin
							dyna_control <= dyc_chksum_0;
						end
					end
					dyc_chksum_0: begin	
						dyna_control <= dyc_chksum_1;
					end
					dyc_chksum_1: begin	
						if (uart_tx_done) begin
							if (dyna_mode_rnw) begin
								dnet_ack <= 1'b0;
								dyna_control <= dyc_read_0;
							end
							else begin
								dnet_ack <= 1'b1;
								dyna_control <= dyc_idle;
							end
						end
					end			
					dyc_read_0: begin
						if (uart_rx_data_valid)	// First byte of FF
							dyna_control <= dyc_read_1; 
					end
					dyc_read_1: begin
						if (uart_rx_data_valid)	// Second byte of FF
							dyna_control <= dyc_read_2; 
					end
					dyc_read_2: begin
						if (uart_rx_data_valid)	// ID
							dyna_control <= dyc_read_3; 
					end
					dyc_read_3: begin
						if (uart_rx_data_valid)	begin // Length
							//uart_rx_data_buf[31:24] <= uart_rx_data[7:0];
							dyna_control <= dyc_read_4; 
						end
					end
					dyc_read_4: begin
						if (uart_rx_data_valid)	begin// Error
							//uart_rx_data_buf[23:16] <= uart_rx_data[7:0];
							dyna_control <= dyc_read_5; 
						end
					end
					dyc_read_5: begin
						if (uart_rx_data_valid)	begin // Data
							uart_rx_data_buf[7:0] <= uart_rx_data[7:0];
							dnet_ack <= 1'b0;
							dyna_control <= dyc_read_6; 
						end
					end
					dyc_read_6: begin
						if (uart_rx_data_valid)	begin // Checksum (or more data, we don't really care too much)
							uart_rx_data_buf[15:8] <= uart_rx_data[7:0];
							dnet_ack <= 1'b1;
							dyna_control <= dyc_idle; 
						end
					end
					
					default: dyna_control <= dyc_idle;
				endcase
			end
        end
	end

	
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
			uart_tx_data <= 8'h00;
			dyna_checksum <= 8'h00;
			tx_data_send <= 1'b0;
        end 
		else begin
			case(dyna_control)
				dyc_start_0:	tx_data_send <= 1'b1;
				dyc_start_2:	tx_data_send <= 1'b1;
				dyc_id_0:		tx_data_send <= 1'b1;
				dyc_length_0:	tx_data_send <= 1'b1;
				dyc_inst_0:		tx_data_send <= 1'b1;
				dyc_reg_addr_0:	tx_data_send <= 1'b1;
				dyc_reg_data_0:	tx_data_send <= 1'b1;
				dyc_reg_data_2:	tx_data_send <= 1'b1;
				dyc_chksum_0:	tx_data_send <= 1'b1;
				default: tx_data_send <= 1'b0;
			endcase
			
			
			if (dyna_control == dyc_idle) begin
				dyna_checksum <= 8'h00;
			end
			else if (uart_tx_done) begin
				case(dyna_control)
					dyc_id_1:		dyna_checksum <= dyna_checksum + uart_tx_data;
					dyc_length_1:	dyna_checksum <= dyna_checksum + uart_tx_data;
					dyc_inst_1:		dyna_checksum <= dyna_checksum + uart_tx_data;
					dyc_reg_addr_1:	dyna_checksum <= dyna_checksum + uart_tx_data;
					dyc_reg_data_1:	dyna_checksum <= dyna_checksum + uart_tx_data;
					dyc_reg_data_3:	dyna_checksum <= dyna_checksum + uart_tx_data;
					default: dyna_checksum <= dyna_checksum;
				endcase
			end
			
			
			
			case(dyna_control)
				dyc_start_0: uart_tx_data <= 8'hFF;
				dyc_start_2: uart_tx_data <= 8'hFF;
				dyc_id_0: uart_tx_data <= dyna_reg_id[7:0];
				dyc_length_0: uart_tx_data <= (dyna_reg_length_in && (!dyna_mode_rnw)) ? 8'h05 : 8'h04;
				dyc_inst_0: uart_tx_data <= dyna_mode_rnw ? 8'h02 : 8'h03;	
				dyc_reg_addr_0: uart_tx_data <= dyna_reg_addr[7:0];
				dyc_reg_data_0: uart_tx_data <= dyna_mode_rnw ? 8'h02 : dyna_reg_data_in[7:0];
				dyc_reg_data_2: uart_tx_data <= dyna_reg_data_in[15:8];
				dyc_chksum_0: uart_tx_data <= ~dyna_checksum;
				default: uart_tx_data <= uart_tx_data;
			endcase
        end
	end

	
endmodule
