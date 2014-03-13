

module fcl_rc_remote
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter	integer						INPUT_CLOCK_SPEED = 125000000,
	parameter	integer						RBUS_ADDR_WIDTH = 16,
	parameter	integer						RBUS_DATA_WIDTH = 16,
	parameter	integer						RBUS_OFFSET = 0
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset_in,
    input 	wire			clk_in,
	
	
	input 	wire			[5:0]			pwm_in,
	output 	reg		signed	[15:0]			pwm_1_out,
	output 	reg		signed	[15:0]			pwm_2_out,
	output 	reg		signed	[15:0]			pwm_3_out,
	output 	reg		signed	[15:0]			pwm_4_out,
	output 	reg		signed	[15:0]			pwm_5_out,
	output 	reg		signed	[15:0]			pwm_6_out,

	// RBUS Signals (debug only)
	output	reg  	[(RBUS_DATA_WIDTH-1):0] rbus_data_out,
	input	wire  	[(RBUS_DATA_WIDTH-1):0] rbus_data_in,
	input	wire  	[(RBUS_ADDR_WIDTH-1):0] rbus_addr_in,
	input 	wire 							rbus_read_in,
	input 	wire 							rbus_write_in,
	output	reg 							rbus_ack_out
);

	// These values have been determined experimentally using the AR6200 and DX6i 
	// They should give a 16bit signed int, with a max of around 25600 and a min of around -25600
	// Always assume lots of noise on this value AND that its min and max values could change
	
	localparam	integer	SERVO_CENTRE_COUNT = (INPUT_CLOCK_SPEED / 663) - 1;	// 188536
	localparam	integer	CMD_COUNT_W = 1 + clogb2(SERVO_CENTRE_COUNT);
	localparam	integer	SCALE_SHIFT = 1;
	
	reg 	[1:0]					pwm_1_buf;
	reg 	[1:0]					pwm_2_buf;
	reg 	[1:0]					pwm_3_buf;
	reg 	[1:0]					pwm_4_buf;
	reg 	[1:0]					pwm_5_buf;
	reg 	[1:0]					pwm_6_buf;
	reg		[(CMD_COUNT_W-1):0]		con_timer_1;
	reg		[(CMD_COUNT_W-1):0]		con_timer_2;
	reg		[(CMD_COUNT_W-1):0]		con_timer_3;
	reg		[(CMD_COUNT_W-1):0]		con_timer_4;
	reg		[(CMD_COUNT_W-1):0]		con_timer_5;
	reg		[(CMD_COUNT_W-1):0]		con_timer_6;
	
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
			pwm_1_buf <= 2'b10;
			pwm_2_buf <= 2'b10;
			pwm_3_buf <= 2'b10;
			pwm_4_buf <= 2'b10;
			pwm_5_buf <= 2'b10;
			pwm_6_buf <= 2'b10;
            con_timer_1 <= {CMD_COUNT_W{1'b0}};
            con_timer_2 <= {CMD_COUNT_W{1'b0}};
            con_timer_3 <= {CMD_COUNT_W{1'b0}};
            con_timer_4 <= {CMD_COUNT_W{1'b0}};
            con_timer_5 <= {CMD_COUNT_W{1'b0}};
            con_timer_6 <= {CMD_COUNT_W{1'b0}};
            pwm_1_out <= {16{1'b0}};
            pwm_2_out <= {16{1'b0}};
            pwm_3_out <= {16{1'b0}};
            pwm_4_out <= {16{1'b0}};
            pwm_5_out <= {16{1'b0}};
            pwm_6_out <= {16{1'b0}};
        end 
		else begin
			pwm_1_buf[1:0] <= {pwm_1_buf[0], pwm_in[0]};
			pwm_2_buf[1:0] <= {pwm_2_buf[0], pwm_in[1]};
			pwm_3_buf[1:0] <= {pwm_3_buf[0], pwm_in[2]};
			pwm_4_buf[1:0] <= {pwm_4_buf[0], pwm_in[3]};
			pwm_5_buf[1:0] <= {pwm_5_buf[0], pwm_in[4]};
			pwm_6_buf[1:0] <= {pwm_6_buf[0], pwm_in[5]};
					
			if (pwm_1_buf == 2'b01) begin
				con_timer_1 <= {CMD_COUNT_W{1'b0}};
			end
			else if (pwm_1_buf[0]) begin
				con_timer_1 <= con_timer_1 + 1'b1;
			end
			if (pwm_1_buf == 2'b10) begin
				pwm_1_out <= ((con_timer_1 - SERVO_CENTRE_COUNT) >>> SCALE_SHIFT);
			end
			
			if (pwm_2_buf == 2'b01) begin
				con_timer_2 <= {CMD_COUNT_W{1'b0}};
			end
			else if (pwm_2_buf[0]) begin
				con_timer_2 <= con_timer_2 + 1'b1;
			end
			if (pwm_2_buf == 2'b10) begin
				pwm_2_out <= ((con_timer_2 - SERVO_CENTRE_COUNT) >>> SCALE_SHIFT);
			end

			if (pwm_3_buf == 2'b01) begin
				con_timer_3 <= {CMD_COUNT_W{1'b0}};
			end
			else if (pwm_3_buf[0]) begin
				con_timer_3 <= con_timer_3 + 1'b1;
			end
			if (pwm_3_buf == 2'b10) begin
				pwm_3_out <= ((con_timer_3 - SERVO_CENTRE_COUNT) >>> SCALE_SHIFT);
			end
			
			if (pwm_4_buf == 2'b01) begin
				con_timer_4 <= {CMD_COUNT_W{1'b0}};
			end
			else if (pwm_4_buf[0]) begin
				con_timer_4 <= con_timer_4 + 1'b1;
			end
			if (pwm_4_buf == 2'b10) begin
				pwm_4_out <= ((con_timer_4 - SERVO_CENTRE_COUNT) >>> SCALE_SHIFT);
			end
			
			if (pwm_5_buf == 2'b01) begin
				con_timer_5 <= {CMD_COUNT_W{1'b0}};
			end
			else if (pwm_5_buf[0]) begin
				con_timer_5 <= con_timer_5 + 1'b1;
			end
			if (pwm_5_buf == 2'b10) begin
				pwm_5_out <= ((con_timer_5 - SERVO_CENTRE_COUNT) >>> SCALE_SHIFT);
			end
			
			if (pwm_6_buf == 2'b01) begin
				con_timer_6 <= {CMD_COUNT_W{1'b0}};
			end
			else if (pwm_6_buf[0]) begin
				con_timer_6 <= con_timer_6 + 1'b1;
			end
			if (pwm_6_buf == 2'b10) begin
				pwm_6_out <= ((con_timer_6 - SERVO_CENTRE_COUNT) >>> SCALE_SHIFT);
			end
		end
	end

	
	always @(*) begin //Start a combinational address mux
		casez (rbus_addr_in)
			RBUS_OFFSET + 'h00 : begin
				rbus_data_out	= pwm_1_out;
				rbus_ack_out	= rbus_read_in; 
			end
			RBUS_OFFSET + 'h01 : begin
				rbus_data_out	= pwm_2_out;
				rbus_ack_out	= rbus_read_in; 
			end
			RBUS_OFFSET + 'h02 : begin
				rbus_data_out	= pwm_3_out;
				rbus_ack_out	= rbus_read_in; 
			end
			RBUS_OFFSET + 'h03 : begin
				rbus_data_out	= pwm_4_out;
				rbus_ack_out	= rbus_read_in; 
			end
			RBUS_OFFSET + 'h04 : begin
				rbus_data_out	= pwm_5_out;
				rbus_ack_out	= rbus_read_in; 
			end
			RBUS_OFFSET + 'h05 : begin
				rbus_data_out	= pwm_6_out;
				rbus_ack_out	= rbus_read_in; 
			end
			
			default : begin
				rbus_data_out	= {RBUS_DATA_WIDTH{1'b0}};
				rbus_ack_out	= 1'b0;
			end
		endcase
	end

	
	function integer clogb2;input [31:0] value;
		for (clogb2=0; value>0; clogb2=clogb2+1)
		value = value>>1;
	endfunction

endmodule
