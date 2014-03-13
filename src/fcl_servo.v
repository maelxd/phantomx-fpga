
// A module written to auto-control Dynamixel Servos

// The simple way this will work:
// The servo bus will be divided into three commands
// A position read command
// A goal set command
// And a custom command (or TBA)

// The idea is that by providing the correct timings, these commands will be able to run in a loop at 1kHz, continually updating and sensing position

// The hardest part will be ensuring that the timings are correct. It has been measured that the position read command takes 327us to complete reliably.
// The issue being that as this is interfacing to a micro-controller, the time time it takes to send back the command is variable. This means, that without modifying the
// servo code, we will have to ensure that this system is very robust against servo response errors.

// The custom command will be added at a later time, so for now, simply do position read and write


module fcl_servo
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 	INPUT_CLOCK_SPEED = 125000000,
	parameter integer 	DYNA_CLOCK_SPEED = 1000000
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset_in,
    input 	wire			clk_in,
	
	// Dynamixel Signals
    inout 	wire			dyna_sda_inout,
	
	// Dynamixel Control
	input	wire	[9:0]		dyna_pos_in,
	input	wire				dyna_pos_we_in,
	output	reg		[9:0]		dyna_pos_out,
	output	reg		[7:0]		dyna_errors_out,
	output	reg		[7:0]		read_error_count_out,
	output	reg					dyna_pos_valid_out,	
	output	wire				dyna_sync_out
);


	
	
	localparam	integer	ONE_MS_COUNT = (INPUT_CLOCK_SPEED / 800) - 1;	// 800Hz Update Rate (1kHz was proving to be too fast!), this rate has been verified to operate without error counts increasing.
	localparam	integer	CMD_COUNT_W = clogb2(ONE_MS_COUNT);
	localparam	integer	CMD_TIME_0 = 0;
	localparam	integer	CMD_TIME_1 = 1*(ONE_MS_COUNT/3);
	localparam	integer	CMD_TIME_2 = 2*(ONE_MS_COUNT/3);
	
	
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
	
	
	
	localparam [1:0] cs_idle		= 2'b00;
	localparam [1:0] cs_cmd_0		= 2'b01;
	localparam [1:0] cs_cmd_1		= 2'b10;
	localparam [1:0] cs_cmd_2		= 2'b11;
	
		
	wire	command_0_init;
	wire	command_1_init;
	wire	command_2_init;
	
	wire	command_0;
	wire	command_1;
	wire	command_2;
	
	
	reg		[4:0]						dyna_control;	
	reg		[7:0]						dyna_checksum;
	reg		[7:0]						uart_tx_data;
	reg									tx_data_send;
	
	wire 								uart_tx_done;
	wire	[7:0]						uart_rx_data;
	wire								uart_rx_data_valid;
	
	wire								uart_sda_rx;
	wire								uart_sda_tx;
	
	reg		[(CMD_COUNT_W-1):0]			con_timer;
	reg		[1:0]						con_state;
	wire								cmd_init;
	
	reg									init_latch;

	reg		[9:0]						dyna_pos_buf;
	
	
	reg									read_state_buf;

	
	wire								dyna_led_status;
	wire	[9:0]						min_error_limit;
	wire	[9:0]						max_error_limit;
	
	assign min_error_limit = (dyna_pos_buf >= 10'h005) ? (dyna_pos_buf - 10'h005) : 10'h000;
	assign max_error_limit = (dyna_pos_buf <= 10'h3FA) ? (dyna_pos_buf + 10'h005) : 10'h3FF;
	assign dyna_led_status = !((dyna_pos_out > min_error_limit) && (dyna_pos_out < max_error_limit));
	

	
	//------------------------------------------------------
	//--				Position In Buffer			 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			dyna_pos_buf <= {1'b1, {9{1'b0}}};
			init_latch <= 1'b0;
		end
		else if (dyna_pos_we_in) begin
			dyna_pos_buf[9:0] <= dyna_pos_in[9:0]; 
			init_latch <= 1'b1;
		end
	end

	
	//------------------------------------------------------
	//--				 	 Logic 				 		  --
	//------------------------------------------------------
    fcl_uart_bidir  #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.BAUD_RATE(DYNA_CLOCK_SPEED),
		.USE_TRI_BUFFER(0))
	serial_port (
		._reset_in(_reset_in), 
		.clk_in(clk_in),
        .uart_sda_in(uart_sda_rx),
        .uart_sda_out(uart_sda_tx),
		.rx_data_out(uart_rx_data), 
		.rx_data_valid_out(uart_rx_data_valid),
		.tx_data_in(uart_tx_data), 
		.tx_data_send_in(tx_data_send),
		.tx_done_out(uart_tx_done), 
		.tx_busy_out());
	
	assign dyna_sda_inout = uart_sda_tx ? 1'bz : 1'b0;
	assign uart_sda_rx = dyna_sda_inout;
	


	
	//------------------------------------------------------
	//--				Controlling Timer			 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
            con_timer <= {CMD_COUNT_W{1'b0}};
        end 
		else begin
			if (con_timer[(CMD_COUNT_W-1):0] == ONE_MS_COUNT[(CMD_COUNT_W-1):0]) begin
				con_timer <= {CMD_COUNT_W{1'b0}};
			end
			else 
				con_timer <= con_timer + 1'b1;
		end
	end

	assign command_0_init = (con_timer[(CMD_COUNT_W-1):0] == CMD_TIME_0[(CMD_COUNT_W-1):0]);
	assign command_1_init = (con_timer[(CMD_COUNT_W-1):0] == CMD_TIME_1[(CMD_COUNT_W-1):0]);
	assign command_2_init = (con_timer[(CMD_COUNT_W-1):0] == CMD_TIME_2[(CMD_COUNT_W-1):0]);

	
	//------------------------------------------------------
	//--		Controller State Machine Logic		 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
            con_state <= cs_idle;
        end 
		else begin
			case(con_state)
				cs_idle:	if (command_0_init)	con_state <= cs_cmd_0;
				cs_cmd_0:	if (command_1_init)	con_state <= cs_cmd_1;
				cs_cmd_1:	if (command_2_init)	con_state <= cs_cmd_2;
				cs_cmd_2: 	if (command_0_init)	con_state <= cs_cmd_0;
				default: 						con_state <= cs_idle;
			endcase
        end
	end

	assign command_0 = (con_state == cs_cmd_0);	// Read Position
	assign command_1 = (con_state == cs_cmd_1); // Write LED
	assign command_2 = (con_state == cs_cmd_2);	// Write Position
	
	assign cmd_init = command_0_init || command_1_init || (command_2_init && init_latch); 
	assign dyna_sync_out = command_1_init;	// After read has completed
	
			
	//------------------------------------------------------
	//--			Servo State Machine Logic		 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
            dyna_control <= dyc_idle;
			read_state_buf <= 1'b0;
        end 
		else begin
			read_state_buf <= ((dyna_control == dyc_read_1) || (dyna_control == dyc_read_2) || (dyna_control == dyc_read_3) || (dyna_control == dyc_read_4) || (dyna_control == dyc_read_5) || (dyna_control == dyc_read_6));
	
			if (cmd_init) begin	// A new start aborts the current operation (better hope this doesn't result in a write / read collide)
				dyna_control <= dyc_start_0;
			end
			else begin
				case(dyna_control)
					dyc_idle: 								dyna_control <= dyc_idle;
					dyc_start_0:							dyna_control <= dyc_start_1;
					dyc_start_1:	if (uart_tx_done)		dyna_control <= dyc_start_2;
					dyc_start_2:							dyna_control <= dyc_start_3;
					dyc_start_3:	if (uart_tx_done)		dyna_control <= dyc_id_0;
					dyc_id_0:								dyna_control <= dyc_id_1;
					dyc_id_1:		if (uart_tx_done)		dyna_control <= dyc_length_0;
					dyc_length_0: 							dyna_control <= dyc_length_1;
					dyc_length_1: 	if (uart_tx_done)		dyna_control <= dyc_inst_0;
					dyc_inst_0: 							dyna_control <= dyc_inst_1;
					dyc_inst_1:		if (uart_tx_done)		dyna_control <= dyc_reg_addr_0;
					dyc_reg_addr_0:							dyna_control <= dyc_reg_addr_1;
					dyc_reg_addr_1:	if (uart_tx_done)		dyna_control <= dyc_reg_data_0;
					dyc_reg_data_0:							dyna_control <= dyc_reg_data_1;
					dyc_reg_data_1:	if (uart_tx_done)		dyna_control <= (command_0 || command_1) ? dyc_chksum_0 : dyc_reg_data_2;
					dyc_reg_data_2:							dyna_control <= dyc_reg_data_3;
					dyc_reg_data_3:	if (uart_tx_done)		dyna_control <= dyc_chksum_0;
					dyc_chksum_0:							dyna_control <= dyc_chksum_1;
					dyc_chksum_1:	if (uart_tx_done)	begin
						dyna_control <= command_0 ? dyc_read_0 : dyc_idle;
					end
					dyc_read_0:		if (uart_rx_data_valid)	dyna_control <= dyc_read_1; 
					dyc_read_1:		if (uart_rx_data_valid)	dyna_control <= dyc_read_2; 
					dyc_read_2:		if (uart_rx_data_valid)	dyna_control <= dyc_read_3; 
					dyc_read_3:		if (uart_rx_data_valid)	dyna_control <= dyc_read_4; 
					dyc_read_4:		if (uart_rx_data_valid)	dyna_control <= dyc_read_5; 
					dyc_read_5:		if (uart_rx_data_valid)	dyna_control <= dyc_read_6; 
					dyc_read_6:		if (uart_rx_data_valid)	dyna_control <= dyc_idle; 
					default: dyna_control <= dyc_idle;
				endcase
			end
        end
	end

	
	//------------------------------------------------------
	//--				Serial Tx Interface 	 	 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
			uart_tx_data <= 8'h00;
			dyna_checksum <= 8'h00;
			tx_data_send <= 1'b0;
        end 
		else begin
			case(dyna_control)
				dyc_start_0:	uart_tx_data <= 8'hFF;								// Start Packet 1
				dyc_start_2:	uart_tx_data <= 8'hFF;								// Start Packet 2
				dyc_id_0:		uart_tx_data <= command_0 ? 8'h01 : 8'h01;		// Dynamixel ID (0xFE = broadcast)
				dyc_length_0:	uart_tx_data <= (command_0 || command_1) ? 8'h04 : 8'h05;		// Length (bytes including this, but not including checksum)
				dyc_inst_0:		uart_tx_data <= command_0 ? 8'h02 : 8'h03;		// Instruction (0x03 = write, 0x02 = read)
				dyc_reg_addr_0:	uart_tx_data <= command_0 ? 8'h24 : (command_1 ? 8'h19 : 8'h1E);		// Register Address
				dyc_reg_data_0:	uart_tx_data <= command_0 ? 8'h02 : (command_1 ? {{7{1'b0}}, dyna_led_status} : dyna_pos_buf[7:0]); 
				dyc_reg_data_2:	uart_tx_data <= {{6{1'b0}}, dyna_pos_buf[9:8]};
				dyc_chksum_0:	uart_tx_data <= ~dyna_checksum;		// Checksum
				default: uart_tx_data <= uart_tx_data;
			endcase
			
			case(dyna_control)
				dyc_idle:		tx_data_send <= 1'b0;
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
        end
	end
	

	//------------------------------------------------------
	//--				Serial Rx Interface 	 	 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
			dyna_errors_out <= {8{1'b0}};
			dyna_pos_out <= {10{1'b0}};
			dyna_pos_valid_out <= 1'b0;
        end 
		else begin
			dyna_errors_out <= read_error_count_out; // debug
			if (uart_rx_data_valid) begin
				case(dyna_control)
					//dyc_read_4:		dyna_errors_out[7:0] <= uart_rx_data[7:0]; 
					dyc_read_5:		dyna_pos_out[7:0] <= uart_rx_data[7:0];
					dyc_read_6:		begin
						dyna_pos_out[9:8] <= uart_rx_data[1:0];
						dyna_pos_valid_out <= 1'b1;
					end
					default: begin

					end
				endcase
			end
			else begin
				if (dyna_control == dyc_chksum_0)	begin	// So we know if reads have failed
						dyna_pos_valid_out <= 1'b0; // may be removed later
				end
			end
        end
	end

	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
			read_error_count_out <= {8{1'b0}};
        end 
		else begin
			if (read_state_buf && (dyna_control == dyc_start_0)) begin
				read_error_count_out <= read_error_count_out + 1'b1;
			end			
        end
	end
	


	function integer clogb2;input [31:0] value;
		for (clogb2=0; value>0; clogb2=clogb2+1)
		value = value>>1;
	endfunction

endmodule
