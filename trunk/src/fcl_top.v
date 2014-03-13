
module fcl_top
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 	SYS_CLK_FREQ = 125000000,
	parameter integer 	UART_BAUD_RATE = 115200
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Ports
	input	wire					sys_clk,
	input	wire					rst,
	input	wire					reset_n,
	
	// RS232 Interface
	input	wire					uart_rx,
	output	wire					uart_tx,
	
	// Dynamixel Servos Interface
	inout	wire	[23:0]			dynamixel_io,
	
	// RC / Servo Interface
	input	wire	[7:0]			servo_io,
	
	// XBee Interface
	input	wire					xbee_dout,
	input	wire					xbee_din,
	input	wire					xbee_cts,
	input	wire					xbee_dtr,
	input	wire					xbee_rts,
	input	wire	[1:0]			xbee_io,
	
	// Extra PMOD (Servo-Controller Board)
	input	wire	[3:0]			pmod_exp,
	
	// Raspberry Pi SPI Interface
	input	wire					rasp_spi_mosi_in,
	output	wire					rasp_spi_miso_out,
	input	wire					rasp_spi_clk_in,
	input	wire					rasp_spi_ncs0_in,
	
	// Debugging LEDs
	output	wire	[3:0]			aux_led,
	output	wire					led
);


	assign uart_tx = 1'b0;

	//------------------------------------------------------
	//--		Local Parameters and Variables		 	  --
	//------------------------------------------------------
	localparam integer RBUS_DATA_WIDTH = 16;
	localparam integer RBUS_ADDR_WIDTH = 16;
	
	// Need to verify the new address widths (and improve timing paths)
	localparam [(RBUS_ADDR_WIDTH-1):0] 	
		RBUS_ADDR_RC				= 16'h0000,
		RBUS_ADDR_SERVO				= 16'h1000;
	
	// System Signals
	(* KEEP = "TRUE" *) wire sys_clk_buf;
	wire				sys_reset;
	reg		[1:0]		_reset_buf;
	reg		[3:0]		reset_hold_count;
	wire				_reset;
	
	// Debug LED Signals
	wire	[3:0]		debug_led;

	// RBus Signals
	wire	[(RBUS_DATA_WIDTH-1):0]	rbus_data_in;
	wire	[(RBUS_ADDR_WIDTH-1):0]	rbus_addr;
	wire							rbus_read;
	wire							rbus_write;
	wire	[(RBUS_DATA_WIDTH-1):0]	rbus_data_out;
	wor		[(RBUS_DATA_WIDTH-1):0]	rbus_data_out_wor;
	wire							rbus_ack;
	wor								rbus_ack_wor;

	wire							rbus_servo_ack;
	wire	[(RBUS_DATA_WIDTH-1):0]	rbus_servo_data_out;
	wire							rbus_remote_ack;
	wire	[(RBUS_DATA_WIDTH-1):0]	rbus_remote_data_out;
	
	reg	[(RBUS_DATA_WIDTH-1):0]	rbus_test_data_out;
	reg							rbus_test_ack;
	
	wire			ike_start;
	wire			ike_done;
	
	wire	[9:0]	l1_q1_value_out;
	wire	[9:0]	l1_q2_value_out;
	wire	[9:0]	l1_q3_value_out;
	wire	[9:0]	l2_q1_value_out;
	wire	[9:0]	l2_q2_value_out;
	wire	[9:0]	l2_q3_value_out;
	wire	[9:0]	l3_q1_value_out;
	wire	[9:0]	l3_q2_value_out;
	wire	[9:0]	l3_q3_value_out;
	wire	[9:0]	l4_q1_value_out;
	wire	[9:0]	l4_q2_value_out;
	wire	[9:0]	l4_q3_value_out;
	wire	[9:0]	l5_q1_value_out;
	wire	[9:0]	l5_q2_value_out;
	wire	[9:0]	l5_q3_value_out;
	wire	[9:0]	l6_q1_value_out;
	wire	[9:0]	l6_q2_value_out;
	wire	[9:0]	l6_q3_value_out;


	//------------------------------------------------------
	//--					Clocking Logic			 	  --
	//------------------------------------------------------
	// Firstly, the clock must be buffered (before it can be used by PLLs)
	`ifdef DEF_SIM
		assign sys_clk_buf = sys_clk;
	`else
		IBUFG  u_ibufg_sys_clk (
			.I(sys_clk),
			.O(sys_clk_buf));
	`endif
	
		
	//------------------------------------------------------
	//--					Reset Logic				 	  --
	//------------------------------------------------------
	// The system has three methods of reset:
	//		1. Power-on-Reset - This reset is held until the reset controller IC has finished
	//		2. Reset Button - The button the expansion board triggers this reset
	//		3. RBUS Reset Command - The bus master can issue a reset command
	// The first two methods are asynchronous, and are held by a 4bit counter once they are released
	// The logic ensures asynchronous set and syncronous de-assert of assynchronous resets
	// The third method is by nature a synchronous reset
	assign sys_reset = (!reset_n) || rst;
	always @(posedge sys_clk_buf or posedge sys_reset) begin
		if (sys_reset) begin
			reset_hold_count <= {4{1'b0}};
			_reset_buf[1:0] <= 2'b00; 
		end 
		else begin
			if (!(&reset_hold_count)) reset_hold_count <= reset_hold_count + 1'b1;
			else _reset_buf[1:0] <= {_reset_buf[0], 1'b1};
		end
	end
	assign _reset = _reset_buf[1];
	

	//------------------------------------------------------
	//--			SPI Interface Logic 				  --
	//------------------------------------------------------
	fcl_spi_bus_master
		test_spi (
			._reset_in		(_reset),
			.clk_in			(sys_clk_buf),
			
			.spi_clk_in		(rasp_spi_clk_in),
			.spi_ncs_in		(rasp_spi_ncs0_in),
			.spi_mosi_in	(rasp_spi_mosi_in),
			.spi_miso_out	(rasp_spi_miso_out),
			
			.bus_data_in	(rbus_data_out),
			.bus_data_out	(rbus_data_in),
			.bus_addr_out	(rbus_addr),
			.bus_read_out	(rbus_read),
			.bus_write_out	(rbus_write),
			.bus_ack_in		(rbus_ack));
	
	
	reg		[15:0]	test_a;
	reg		[15:0]	test_b;
	
	
	always @(posedge sys_clk_buf or negedge _reset) begin
		if (!_reset) begin
			rbus_test_data_out <= {16{1'b0}};
			test_a <= {16{1'b0}};
			test_b <= {16{1'b1}};
			rbus_test_ack <= 1'b0;
		end 
		else begin
			if (rbus_addr == 16'h2000) begin
				if (rbus_write)
					test_a <= rbus_data_in;
				if (rbus_read)
					rbus_test_data_out <= test_a;
				else
					rbus_test_data_out <= {16{1'b0}};
			end
			else if (rbus_addr == 16'h2001) begin
				if (rbus_write)
					test_b <= rbus_data_in;
				if (rbus_read)
					rbus_test_data_out <= test_b;
				else
					rbus_test_data_out <= {16{1'b0}};
			end
			else begin
				rbus_test_data_out <= {16{1'b0}};
			end
			
			rbus_test_ack <= rbus_write || rbus_read;
		end
	end
	
	
	
	wire signed [15:0] rc_signal [5:0];

	fcl_rc_remote #(
			.INPUT_CLOCK_SPEED	(SYS_CLK_FREQ),
			.RBUS_ADDR_WIDTH	(RBUS_ADDR_WIDTH),
			.RBUS_DATA_WIDTH	(RBUS_DATA_WIDTH),
			.RBUS_OFFSET		(RBUS_ADDR_RC))
		remote_interface (
			.clk_in			(sys_clk_buf),
			._reset_in		(_reset),

			.pwm_in			(servo_io[5:0]),
			.pwm_1_out		(rc_signal[0]),
			.pwm_2_out		(rc_signal[1]),
			.pwm_3_out		(rc_signal[2]),
			.pwm_4_out		(rc_signal[3]),
			.pwm_5_out		(rc_signal[4]),
			.pwm_6_out		(rc_signal[5]),

			.rbus_data_out	(rbus_remote_data_out),
			.rbus_data_in	(rbus_data_in),
			.rbus_addr_in	(rbus_addr),
			.rbus_read_in	(rbus_read),
			.rbus_write_in	(rbus_write),
			.rbus_ack_out	(rbus_remote_ack));
	
	fcl_sequencer #(
			.DNET_ADDR_WIDTH	(RBUS_ADDR_WIDTH),
			.DNET_DATA_WIDTH	(RBUS_DATA_WIDTH),
			.DNET_OFFSET		(0),
			.POS_WIDTH			(16))
		ike (
			.clk_in		(sys_clk_buf),
			._reset_in	(_reset),
			
			.sync_pulse_in(ike_start && (rc_signal[4]>0)),
			
			.delta_angle_x_in(0), //(rc_signal[1] >>> 3)),
			.delta_angle_y_in(0), //(rc_signal[1] >>> 3)),
			
			.speed_in({{4{rc_signal[0][15]}}, rc_signal[0][15:4]}),
			.rotate_mode_in(rc_signal[5]>0),
			.z_offset_in({{4{rc_signal[3][15]}}, rc_signal[3][15:4]}),

			.done_out(ike_done),
			.l1_q1_dyna_out(l1_q1_value_out),
			.l1_q2_dyna_out(l1_q2_value_out),
			.l1_q3_dyna_out(l1_q3_value_out),
			.l2_q1_dyna_out(l2_q1_value_out),
			.l2_q2_dyna_out(l2_q2_value_out),
			.l2_q3_dyna_out(l2_q3_value_out),
			.l3_q1_dyna_out(l3_q1_value_out),
			.l3_q2_dyna_out(l3_q2_value_out),
			.l3_q3_dyna_out(l3_q3_value_out),
			.l4_q1_dyna_out(l4_q1_value_out),
			.l4_q2_dyna_out(l4_q2_value_out),
			.l4_q3_dyna_out(l4_q3_value_out),
			.l5_q1_dyna_out(l5_q1_value_out),
			.l5_q2_dyna_out(l5_q2_value_out),
			.l5_q3_dyna_out(l5_q3_value_out),
			.l6_q1_dyna_out(l6_q1_value_out),
			.l6_q2_dyna_out(l6_q2_value_out),
			.l6_q3_dyna_out(l6_q3_value_out));
	
	
	fcl_servo_controller #(
			.RBUS_ADDR_WIDTH	(RBUS_ADDR_WIDTH),
			.RBUS_DATA_WIDTH	(RBUS_DATA_WIDTH),
			.RBUS_OFFSET		(RBUS_ADDR_SERVO))
		servo_interface (
			.clk_in		(sys_clk_buf),
			._reset_in	(_reset),

			// Joint Mapping Here!
			.dyna_sda_inout({	//Q3				Q2					Q1
								dynamixel_io[23],	dynamixel_io[22],	dynamixel_io[19],	// Leg 6
								dynamixel_io[20],	dynamixel_io[21],	dynamixel_io[16],	// Leg 5
								dynamixel_io[15],	dynamixel_io[14],	dynamixel_io[11],	// Leg 4
								dynamixel_io[12],	dynamixel_io[13],	dynamixel_io[8],	// Leg 3
								dynamixel_io[7],	dynamixel_io[6],	dynamixel_io[3],	// Leg 2
								dynamixel_io[4],	dynamixel_io[5],	dynamixel_io[0]}),	// Leg 1

			.dyna_pos_we_in		(ike_done),
			.dyna_l1_q1_pos_in	(l1_q1_value_out),
			.dyna_l1_q2_pos_in	(l1_q2_value_out),
			.dyna_l1_q3_pos_in	(l1_q3_value_out),
			.dyna_l2_q1_pos_in	(l2_q1_value_out),
			.dyna_l2_q2_pos_in	(l2_q2_value_out),
			.dyna_l2_q3_pos_in	(l2_q3_value_out),
			.dyna_l3_q1_pos_in	(l3_q1_value_out),
			.dyna_l3_q2_pos_in	(l3_q2_value_out),
			.dyna_l3_q3_pos_in	(l3_q3_value_out),
			.dyna_l4_q1_pos_in	(l4_q1_value_out),
			.dyna_l4_q2_pos_in	(l4_q2_value_out),
			.dyna_l4_q3_pos_in	(l4_q3_value_out),
			.dyna_l5_q1_pos_in	(l5_q1_value_out),
			.dyna_l5_q2_pos_in	(l5_q2_value_out),
			.dyna_l5_q3_pos_in	(l5_q3_value_out),
			.dyna_l6_q1_pos_in	(l6_q1_value_out),
			.dyna_l6_q2_pos_in	(l6_q2_value_out),
			.dyna_l6_q3_pos_in	(l6_q3_value_out),
			.dyna_sync_out		(ike_start),
			
			.rbus_data_out		(rbus_servo_data_out),
			.rbus_data_in		(rbus_data_in),
			.rbus_addr_in		(rbus_addr),
			.rbus_read_in		(rbus_read),
			.rbus_write_in		(rbus_write),
			.rbus_ack_out		(rbus_servo_ack));

	// Bus Combining
	assign rbus_data_out_wor	= {RBUS_DATA_WIDTH{1'b0}};
	assign rbus_data_out_wor	= rbus_servo_data_out;
	assign rbus_data_out_wor	= rbus_remote_data_out;
	assign rbus_data_out_wor	= rbus_test_data_out;
	assign rbus_data_out		= rbus_data_out_wor;

	assign rbus_ack_wor	= 1'b0;
	assign rbus_ack_wor	= rbus_servo_ack;
	assign rbus_ack_wor	= rbus_remote_ack;
	assign rbus_ack_wor	= rbus_test_ack;
	assign rbus_ack		= rbus_ack_wor;
	
	assign debug_led[3:0] = 4'hF; 
	assign aux_led = ~debug_led;
	assign led = 1'b1;

	// This is a dummy interface to the spare servo ports (also used for RAW dynamixel interface tests)
	fcl_servo_test #(
		.DNET_ADDR_WIDTH(32),
		.DNET_DATA_WIDTH(32),
		.NUM_DYNAMIXELS(6))
	servo_interface_generic (
		.sys_clk(sys_clk_buf),
		._reset(_reset),

		.dyna_sda_inout({dynamixel_io[2:1], dynamixel_io[10:9], dynamixel_io[18:17]}),

		.dnet_data_out(),
		.dnet_data_in({32{1'b0}}),
		.dnet_addr_in({16{1'b0}}),
		.dnet_read(1'b0),
		.dnet_write(1'b0),
		.dnet_ack());

endmodule
