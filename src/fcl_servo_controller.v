
// This module is used to group the individual servo interfaces together (greatly simplifies the top level!)


module fcl_servo_controller
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 	INPUT_CLOCK_SPEED = 125000000,
	parameter integer 	DYNA_CLOCK_SPEED = 1000000,
	parameter integer 	RBUS_ADDR_WIDTH = 16,
	parameter integer 	RBUS_DATA_WIDTH = 16,
	parameter integer 	RBUS_OFFSET = 0
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire				_reset_in,
    input 	wire				clk_in,
	
	// Dynamixel Signals
    inout 	wire	[17:0]		dyna_sda_inout,
	
	// Dynamixel Control
	input	wire				dyna_pos_we_in,
	input	wire	[9:0]		dyna_l1_q1_pos_in,
	input	wire	[9:0]		dyna_l1_q2_pos_in,
	input	wire	[9:0]		dyna_l1_q3_pos_in,
	input	wire	[9:0]		dyna_l2_q1_pos_in,
	input	wire	[9:0]		dyna_l2_q2_pos_in,
	input	wire	[9:0]		dyna_l2_q3_pos_in,
	input	wire	[9:0]		dyna_l3_q1_pos_in,
	input	wire	[9:0]		dyna_l3_q2_pos_in,
	input	wire	[9:0]		dyna_l3_q3_pos_in,
	input	wire	[9:0]		dyna_l4_q1_pos_in,
	input	wire	[9:0]		dyna_l4_q2_pos_in,
	input	wire	[9:0]		dyna_l4_q3_pos_in,
	input	wire	[9:0]		dyna_l5_q1_pos_in,
	input	wire	[9:0]		dyna_l5_q2_pos_in,
	input	wire	[9:0]		dyna_l5_q3_pos_in,
	input	wire	[9:0]		dyna_l6_q1_pos_in,
	input	wire	[9:0]		dyna_l6_q2_pos_in,
	input	wire	[9:0]		dyna_l6_q3_pos_in,
	output	wire				dyna_sync_out,
		
		
	// RBUS Signals
	output	reg  	[(RBUS_DATA_WIDTH-1):0] rbus_data_out,
	input	wire  	[(RBUS_DATA_WIDTH-1):0] rbus_data_in,
	input	wire  	[(RBUS_ADDR_WIDTH-1):0] rbus_addr_in,
	input 	wire 							rbus_read_in,
	input 	wire 							rbus_write_in,
	output	reg 							rbus_ack_out
);
	
	localparam	integer	SERVO_DRIVE_MODE_DEFAULT = 1; 
	
	reg 	[(RBUS_ADDR_WIDTH-1):0]	rbus_addr_in_buf;
	reg 	[(RBUS_DATA_WIDTH-1):0]	rbus_data_in_buf;
	reg 							rbus_read_buf;
	reg 							rbus_write_buf;
	
	reg				servo_write_pos_enable;

	wire	[9:0]	dyna_l1_q1_pos_out;
	wire	[7:0]	dyna_l1_q1_errors_out;
	wire			dyna_l1_q1_sync_out;
	wire			dyna_l1_q1_pos_valid_out;
	wire	[9:0]	dyna_l1_q2_pos_out;
	wire	[7:0]	dyna_l1_q2_errors_out;
	wire			dyna_l1_q2_sync_out;
	wire			dyna_l1_q2_pos_valid_out;
	wire	[9:0]	dyna_l1_q3_pos_out;
	wire	[7:0]	dyna_l1_q3_errors_out;
	wire			dyna_l1_q3_sync_out;
	wire			dyna_l1_q3_pos_valid_out;
	
	wire	[9:0]	dyna_l2_q1_pos_out;
	wire	[7:0]	dyna_l2_q1_errors_out;
	wire			dyna_l2_q1_sync_out;
	wire			dyna_l2_q1_pos_valid_out;
	wire	[9:0]	dyna_l2_q2_pos_out;
	wire	[7:0]	dyna_l2_q2_errors_out;
	wire			dyna_l2_q2_sync_out;
	wire			dyna_l2_q2_pos_valid_out;
	wire	[9:0]	dyna_l2_q3_pos_out;
	wire	[7:0]	dyna_l2_q3_errors_out;
	wire			dyna_l2_q3_sync_out;
	wire			dyna_l2_q3_pos_valid_out;
	
	wire	[9:0]	dyna_l3_q1_pos_out;
	wire	[7:0]	dyna_l3_q1_errors_out;
	wire			dyna_l3_q1_sync_out;
	wire			dyna_l3_q1_pos_valid_out;
	wire	[9:0]	dyna_l3_q2_pos_out;
	wire	[7:0]	dyna_l3_q2_errors_out;
	wire			dyna_l3_q2_sync_out;
	wire			dyna_l3_q2_pos_valid_out;
	wire	[9:0]	dyna_l3_q3_pos_out;
	wire	[7:0]	dyna_l3_q3_errors_out;
	wire			dyna_l3_q3_sync_out;
	wire			dyna_l3_q3_pos_valid_out;
	
	wire	[9:0]	dyna_l4_q1_pos_out;
	wire	[7:0]	dyna_l4_q1_errors_out;
	wire			dyna_l4_q1_sync_out;
	wire			dyna_l4_q1_pos_valid_out;
	wire	[9:0]	dyna_l4_q2_pos_out;
	wire	[7:0]	dyna_l4_q2_errors_out;
	wire			dyna_l4_q2_sync_out;
	wire			dyna_l4_q2_pos_valid_out;
	wire	[9:0]	dyna_l4_q3_pos_out;
	wire	[7:0]	dyna_l4_q3_errors_out;
	wire			dyna_l4_q3_sync_out;
	wire			dyna_l4_q3_pos_valid_out;
	
	wire	[9:0]	dyna_l5_q1_pos_out;
	wire	[7:0]	dyna_l5_q1_errors_out;
	wire			dyna_l5_q1_sync_out;
	wire			dyna_l5_q1_pos_valid_out;
	wire	[9:0]	dyna_l5_q2_pos_out;
	wire	[7:0]	dyna_l5_q2_errors_out;
	wire			dyna_l5_q2_sync_out;
	wire			dyna_l5_q2_pos_valid_out;
	wire	[9:0]	dyna_l5_q3_pos_out;
	wire	[7:0]	dyna_l5_q3_errors_out;
	wire			dyna_l5_q3_sync_out;
	wire			dyna_l5_q3_pos_valid_out;
	
	wire	[9:0]	dyna_l6_q1_pos_out;
	wire	[7:0]	dyna_l6_q1_errors_out;
	wire			dyna_l6_q1_sync_out;
	wire			dyna_l6_q1_pos_valid_out;
	wire	[9:0]	dyna_l6_q2_pos_out;
	wire	[7:0]	dyna_l6_q2_errors_out;
	wire			dyna_l6_q2_sync_out;
	wire			dyna_l6_q2_pos_valid_out;
	wire	[9:0]	dyna_l6_q3_pos_out;
	wire	[7:0]	dyna_l6_q3_errors_out;
	wire			dyna_l6_q3_sync_out;
	wire			dyna_l6_q3_pos_valid_out;
	
		
	assign dyna_sync_out = dyna_l1_q1_sync_out;
	
	
	//------------------------------------------------------
	//--					Leg 1					 	  --
	//------------------------------------------------------
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l1_q1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[0]),

		.dyna_pos_in(dyna_l1_q1_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l1_q1_pos_out),
		.dyna_errors_out(dyna_l1_q1_errors_out),
		.dyna_pos_valid_out(dyna_l1_q1_pos_valid_out),
		.dyna_sync_out(dyna_l1_q1_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l1_q2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[1]),

		.dyna_pos_in(dyna_l1_q2_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l1_q2_pos_out),
		.dyna_errors_out(dyna_l1_q2_errors_out),
		.dyna_pos_valid_out(dyna_l1_q2_pos_valid_out),
		.dyna_sync_out(dyna_l1_q2_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l1_q3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[2]),

		.dyna_pos_in(dyna_l1_q3_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l1_q3_pos_out),
		.dyna_errors_out(dyna_l1_q3_errors_out),
		.dyna_pos_valid_out(dyna_l1_q3_pos_valid_out),
		.dyna_sync_out(dyna_l1_q3_sync_out));
		
	//------------------------------------------------------
	//--					Leg 2					 	  --
	//------------------------------------------------------
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l2_q1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[3]),

		.dyna_pos_in(dyna_l2_q1_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l2_q1_pos_out),
		.dyna_errors_out(dyna_l2_q1_errors_out),
		.dyna_pos_valid_out(dyna_l2_q1_pos_valid_out),
		.dyna_sync_out(dyna_l2_q1_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l2_q2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[4]),

		.dyna_pos_in(dyna_l2_q2_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l2_q2_pos_out),
		.dyna_errors_out(dyna_l2_q2_errors_out),
		.dyna_pos_valid_out(dyna_l2_q2_pos_valid_out),
		.dyna_sync_out(dyna_l2_q2_sync_out));

	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l2_q3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[5]),

		.dyna_pos_in(dyna_l2_q3_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l2_q3_pos_out),
		.dyna_errors_out(dyna_l2_q3_errors_out),
		.dyna_pos_valid_out(dyna_l2_q3_pos_valid_out),
		.dyna_sync_out(dyna_l2_q3_sync_out));
		
	//------------------------------------------------------
	//--					Leg 3					 	  --
	//------------------------------------------------------
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l3_q1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[6]),

		.dyna_pos_in(dyna_l3_q1_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l3_q1_pos_out),
		.dyna_errors_out(dyna_l3_q1_errors_out),
		.dyna_pos_valid_out(dyna_l3_q1_pos_valid_out),
		.dyna_sync_out(dyna_l3_q1_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l3_q2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[7]),

		.dyna_pos_in(dyna_l3_q2_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l3_q2_pos_out),
		.dyna_errors_out(dyna_l3_q2_errors_out),
		.dyna_pos_valid_out(dyna_l3_q2_pos_valid_out),
		.dyna_sync_out(dyna_l3_q2_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l3_q3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[8]),

		.dyna_pos_in(dyna_l3_q3_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l3_q3_pos_out),
		.dyna_errors_out(dyna_l3_q3_errors_out),
		.dyna_pos_valid_out(dyna_l3_q3_pos_valid_out),
		.dyna_sync_out(dyna_l3_q3_sync_out));
		
	//------------------------------------------------------
	//--					Leg 4					 	  --
	//------------------------------------------------------
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l4_q1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[9]),

		.dyna_pos_in(dyna_l4_q1_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l4_q1_pos_out),
		.dyna_errors_out(dyna_l4_q1_errors_out),
		.dyna_pos_valid_out(dyna_l4_q1_pos_valid_out),
		.dyna_sync_out(dyna_l4_q1_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l4_q2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[10]),

		.dyna_pos_in(dyna_l4_q2_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l4_q2_pos_out),
		.dyna_errors_out(dyna_l4_q2_errors_out),
		.dyna_pos_valid_out(dyna_l4_q2_pos_valid_out),
		.dyna_sync_out(dyna_l4_q2_sync_out));

	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l4_q3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[11]),

		.dyna_pos_in(dyna_l4_q3_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l4_q3_pos_out),
		.dyna_errors_out(dyna_l4_q3_errors_out),
		.dyna_pos_valid_out(dyna_l4_q3_pos_valid_out),
		.dyna_sync_out(dyna_l4_q3_sync_out));
		
	//------------------------------------------------------
	//--					Leg 5					 	  --
	//------------------------------------------------------
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l5_q1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[12]),

		.dyna_pos_in(dyna_l5_q1_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l5_q1_pos_out),
		.dyna_errors_out(dyna_l5_q1_errors_out),
		.dyna_pos_valid_out(dyna_l5_q1_pos_valid_out),
		.dyna_sync_out(dyna_l5_q1_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l5_q2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[13]),

		.dyna_pos_in(dyna_l5_q2_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l5_q2_pos_out),
		.dyna_errors_out(dyna_l5_q2_errors_out),
		.dyna_pos_valid_out(dyna_l5_q2_pos_valid_out),
		.dyna_sync_out(dyna_l5_q2_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l5_q3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[14]),

		.dyna_pos_in(dyna_l5_q3_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l5_q3_pos_out),
		.dyna_errors_out(dyna_l5_q3_errors_out),
		.dyna_pos_valid_out(dyna_l5_q3_pos_valid_out),
		.dyna_sync_out(dyna_l5_q3_sync_out));
		
	//------------------------------------------------------
	//--					Leg 6					 	  --
	//------------------------------------------------------
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l6_q1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[15]),

		.dyna_pos_in(dyna_l6_q1_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l6_q1_pos_out),
		.dyna_errors_out(dyna_l6_q1_errors_out),
		.dyna_pos_valid_out(dyna_l6_q1_pos_valid_out),
		.dyna_sync_out(dyna_l6_q1_sync_out));
		
	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l6_q2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[16]),

		.dyna_pos_in(dyna_l6_q2_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l6_q2_pos_out),
		.dyna_errors_out(dyna_l6_q2_errors_out),
		.dyna_pos_valid_out(dyna_l6_q2_pos_valid_out),
		.dyna_sync_out(dyna_l6_q2_sync_out));

	fcl_servo #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.DYNA_CLOCK_SPEED(DYNA_CLOCK_SPEED))
	servo_interface_l6_q3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),

		.dyna_sda_inout(dyna_sda_inout[17]),

		.dyna_pos_in(dyna_l6_q3_pos_in),
		.dyna_pos_we_in(dyna_pos_we_in && servo_write_pos_enable),
		.dyna_pos_out(dyna_l6_q3_pos_out),
		.dyna_errors_out(dyna_l6_q3_errors_out),
		.dyna_pos_valid_out(dyna_l6_q3_pos_valid_out),
		.dyna_sync_out(dyna_l6_q3_sync_out));
		


	//------------------------------------------------------
	//--			Position and Error Buffering 		  --
	//------------------------------------------------------
	reg		[9:0]	dyna_l1_q1_pos_buf;
	reg		[7:0]	dyna_l1_q1_errors_buf;
	reg		[9:0]	dyna_l1_q2_pos_buf;
	reg		[7:0]	dyna_l1_q2_errors_buf;
	reg		[9:0]	dyna_l1_q3_pos_buf;
	reg		[7:0]	dyna_l1_q3_errors_buf;
	reg		[9:0]	dyna_l2_q1_pos_buf;
	reg		[7:0]	dyna_l2_q1_errors_buf;
	reg		[9:0]	dyna_l2_q2_pos_buf;
	reg		[7:0]	dyna_l2_q2_errors_buf;
	reg		[9:0]	dyna_l2_q3_pos_buf;
	reg		[7:0]	dyna_l2_q3_errors_buf;
	reg		[9:0]	dyna_l3_q1_pos_buf;
	reg		[7:0]	dyna_l3_q1_errors_buf;
	reg		[9:0]	dyna_l3_q2_pos_buf;
	reg		[7:0]	dyna_l3_q2_errors_buf;
	reg		[9:0]	dyna_l3_q3_pos_buf;
	reg		[7:0]	dyna_l3_q3_errors_buf;
	reg		[9:0]	dyna_l4_q1_pos_buf;
	reg		[7:0]	dyna_l4_q1_errors_buf;
	reg		[9:0]	dyna_l4_q2_pos_buf;
	reg		[7:0]	dyna_l4_q2_errors_buf;
	reg		[9:0]	dyna_l4_q3_pos_buf;
	reg		[7:0]	dyna_l4_q3_errors_buf;
	reg		[9:0]	dyna_l5_q1_pos_buf;
	reg		[7:0]	dyna_l5_q1_errors_buf;
	reg		[9:0]	dyna_l5_q2_pos_buf;
	reg		[7:0]	dyna_l5_q2_errors_buf;
	reg		[9:0]	dyna_l5_q3_pos_buf;
	reg		[7:0]	dyna_l5_q3_errors_buf;
	reg		[9:0]	dyna_l6_q1_pos_buf;
	reg		[7:0]	dyna_l6_q1_errors_buf;
	reg		[9:0]	dyna_l6_q2_pos_buf;
	reg		[7:0]	dyna_l6_q2_errors_buf;
	reg		[9:0]	dyna_l6_q3_pos_buf;
	reg		[7:0]	dyna_l6_q3_errors_buf;
	
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			dyna_l1_q1_pos_buf <= {10{1'b0}};
			dyna_l1_q1_errors_buf <= {8{1'b0}};
			dyna_l1_q2_pos_buf <= {10{1'b0}};
			dyna_l1_q2_errors_buf <= {8{1'b0}};
			dyna_l1_q3_pos_buf <= {10{1'b0}};
			dyna_l1_q3_errors_buf <= {8{1'b0}};
			dyna_l2_q1_pos_buf <= {10{1'b0}};
			dyna_l2_q1_errors_buf <= {8{1'b0}};
			dyna_l2_q2_pos_buf <= {10{1'b0}};
			dyna_l2_q2_errors_buf <= {8{1'b0}};
			dyna_l2_q3_pos_buf <= {10{1'b0}};
			dyna_l2_q3_errors_buf <= {8{1'b0}};
			dyna_l3_q1_pos_buf <= {10{1'b0}};
			dyna_l3_q1_errors_buf <= {8{1'b0}};
			dyna_l3_q2_pos_buf <= {10{1'b0}};
			dyna_l3_q2_errors_buf <= {8{1'b0}};
			dyna_l3_q3_pos_buf <= {10{1'b0}};
			dyna_l3_q3_errors_buf <= {8{1'b0}};
			dyna_l4_q1_pos_buf <= {10{1'b0}};
			dyna_l4_q1_errors_buf <= {8{1'b0}};
			dyna_l4_q2_pos_buf <= {10{1'b0}};
			dyna_l4_q2_errors_buf <= {8{1'b0}};
			dyna_l4_q3_pos_buf <= {10{1'b0}};
			dyna_l4_q3_errors_buf <= {8{1'b0}};
			dyna_l5_q1_pos_buf <= {10{1'b0}};
			dyna_l5_q1_errors_buf <= {8{1'b0}};
			dyna_l5_q2_pos_buf <= {10{1'b0}};
			dyna_l5_q2_errors_buf <= {8{1'b0}};
			dyna_l5_q3_pos_buf <= {10{1'b0}};
			dyna_l5_q3_errors_buf <= {8{1'b0}};
			dyna_l6_q1_pos_buf <= {10{1'b0}};
			dyna_l6_q1_errors_buf <= {8{1'b0}};
			dyna_l6_q2_pos_buf <= {10{1'b0}};
			dyna_l6_q2_errors_buf <= {8{1'b0}};
			dyna_l6_q3_pos_buf <= {10{1'b0}};
			dyna_l6_q3_errors_buf <= {8{1'b0}};
		end
		else begin
			if (dyna_l1_q1_sync_out) begin
				dyna_l1_q1_pos_buf <= (!dyna_l1_q1_pos_valid_out) ? {10{1'b0}} : dyna_l1_q1_pos_out;
				dyna_l1_q1_errors_buf <= (!dyna_l1_q1_pos_valid_out) ? 8'h80 : dyna_l1_q1_errors_out;
			end
			
			if (dyna_l1_q2_sync_out) begin
				dyna_l1_q2_pos_buf <= (!dyna_l1_q2_pos_valid_out) ? {10{1'b0}} : dyna_l1_q2_pos_out;
				dyna_l1_q2_errors_buf <= (!dyna_l1_q2_pos_valid_out) ? 8'h80 : dyna_l1_q2_errors_out;
			end
			
			if (dyna_l1_q3_sync_out) begin
				dyna_l1_q3_pos_buf <= (!dyna_l1_q3_pos_valid_out) ? {10{1'b0}} : dyna_l1_q3_pos_out;
				dyna_l1_q3_errors_buf <= (!dyna_l1_q3_pos_valid_out) ? 8'h80 : dyna_l1_q3_errors_out;
			end
			
			if (dyna_l2_q1_sync_out) begin
				dyna_l2_q1_pos_buf <= (!dyna_l2_q1_pos_valid_out) ? {10{1'b0}} : dyna_l2_q1_pos_out;
				dyna_l2_q1_errors_buf <= (!dyna_l2_q1_pos_valid_out) ? 8'h80 : dyna_l2_q1_errors_out;
			end
			
			if (dyna_l2_q2_sync_out) begin
				dyna_l2_q2_pos_buf <= (!dyna_l2_q2_pos_valid_out) ? {10{1'b0}} : dyna_l2_q2_pos_out;
				dyna_l2_q2_errors_buf <= (!dyna_l2_q2_pos_valid_out) ? 8'h80 : dyna_l2_q2_errors_out;
			end
			
			if (dyna_l2_q3_sync_out) begin
				dyna_l2_q3_pos_buf <= (!dyna_l2_q3_pos_valid_out) ? {10{1'b0}} : dyna_l2_q3_pos_out;
				dyna_l2_q3_errors_buf <= (!dyna_l2_q3_pos_valid_out) ? 8'h80 : dyna_l2_q3_errors_out;
			end
			
			if (dyna_l3_q1_sync_out) begin
				dyna_l3_q1_pos_buf <= (!dyna_l3_q1_pos_valid_out) ? {10{1'b0}} : dyna_l3_q1_pos_out;
				dyna_l3_q1_errors_buf <= (!dyna_l3_q1_pos_valid_out) ? 8'h80 : dyna_l3_q1_errors_out;
			end
			
			if (dyna_l3_q2_sync_out) begin
				dyna_l3_q2_pos_buf <= (!dyna_l3_q2_pos_valid_out) ? {10{1'b0}} : dyna_l3_q2_pos_out;
				dyna_l3_q2_errors_buf <= (!dyna_l3_q2_pos_valid_out) ? 8'h80 : dyna_l3_q2_errors_out;
			end
			
			if (dyna_l3_q3_sync_out) begin
				dyna_l3_q3_pos_buf <= (!dyna_l3_q3_pos_valid_out) ? {10{1'b0}} : dyna_l3_q3_pos_out;
				dyna_l3_q3_errors_buf <= (!dyna_l3_q3_pos_valid_out) ? 8'h80 : dyna_l3_q3_errors_out;
			end
			
			if (dyna_l4_q1_sync_out) begin
				dyna_l4_q1_pos_buf <= (!dyna_l4_q1_pos_valid_out) ? {10{1'b0}} : dyna_l4_q1_pos_out;
				dyna_l4_q1_errors_buf <= (!dyna_l4_q1_pos_valid_out) ? 8'h80 : dyna_l4_q1_errors_out;
			end
			
			if (dyna_l4_q2_sync_out) begin
				dyna_l4_q2_pos_buf <= (!dyna_l4_q2_pos_valid_out) ? {10{1'b0}} : dyna_l4_q2_pos_out;
				dyna_l4_q2_errors_buf <= (!dyna_l4_q2_pos_valid_out) ? 8'h80 : dyna_l4_q2_errors_out;
			end
			
			if (dyna_l4_q3_sync_out) begin
				dyna_l4_q3_pos_buf <= (!dyna_l4_q3_pos_valid_out) ? {10{1'b0}} : dyna_l4_q3_pos_out;
				dyna_l4_q3_errors_buf <= (!dyna_l4_q3_pos_valid_out) ? 8'h80 : dyna_l4_q3_errors_out;
			end
			
			if (dyna_l5_q1_sync_out) begin
				dyna_l5_q1_pos_buf <= (!dyna_l5_q1_pos_valid_out) ? {10{1'b0}} : dyna_l5_q1_pos_out;
				dyna_l5_q1_errors_buf <= (!dyna_l5_q1_pos_valid_out) ? 8'h80 : dyna_l5_q1_errors_out;
			end
			
			if (dyna_l5_q2_sync_out) begin
				dyna_l5_q2_pos_buf <= (!dyna_l5_q2_pos_valid_out) ? {10{1'b0}} : dyna_l5_q2_pos_out;
				dyna_l5_q2_errors_buf <= (!dyna_l5_q2_pos_valid_out) ? 8'h80 : dyna_l5_q2_errors_out;
			end
			
			if (dyna_l5_q3_sync_out) begin
				dyna_l5_q3_pos_buf <= (!dyna_l5_q3_pos_valid_out) ? {10{1'b0}} : dyna_l5_q3_pos_out;
				dyna_l5_q3_errors_buf <= (!dyna_l5_q3_pos_valid_out) ? 8'h80 : dyna_l5_q3_errors_out;
			end
			
			if (dyna_l6_q1_sync_out) begin
				dyna_l6_q1_pos_buf <= (!dyna_l6_q1_pos_valid_out) ? {10{1'b0}} : dyna_l6_q1_pos_out;
				dyna_l6_q1_errors_buf <= (!dyna_l6_q1_pos_valid_out) ? 8'h80 : dyna_l6_q1_errors_out;
			end
			
			if (dyna_l6_q2_sync_out) begin
				dyna_l6_q2_pos_buf <= (!dyna_l6_q2_pos_valid_out) ? {10{1'b0}} : dyna_l6_q2_pos_out;
				dyna_l6_q2_errors_buf <= (!dyna_l6_q2_pos_valid_out) ? 8'h80 : dyna_l6_q2_errors_out;
			end
			
			if (dyna_l6_q3_sync_out) begin
				dyna_l6_q3_pos_buf <= (!dyna_l6_q3_pos_valid_out) ? {10{1'b0}} : dyna_l6_q3_pos_out;
				dyna_l6_q3_errors_buf <= (!dyna_l6_q3_pos_valid_out) ? 8'h80 : dyna_l6_q3_errors_out;
			end
		end
	end


	
	//------------------------------------------------------
	//--				RBUS Interface Logic		 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			rbus_addr_in_buf <= {RBUS_ADDR_WIDTH{1'b0}};
			rbus_data_in_buf <= {RBUS_DATA_WIDTH{1'b0}};
			rbus_read_buf <= 1'b0;
			rbus_write_buf <= 1'b0;
		end
		else begin
			rbus_addr_in_buf <= rbus_addr_in;
			rbus_data_in_buf <= rbus_data_in;
			rbus_read_buf <= rbus_read_in;
			rbus_write_buf <= rbus_write_in;
		end
	end
	
	
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			servo_write_pos_enable <= SERVO_DRIVE_MODE_DEFAULT;
		end
		else begin
			if ((rbus_addr_in_buf == RBUS_OFFSET + 'h12) && rbus_write_buf) begin
				servo_write_pos_enable <= rbus_data_in_buf[0];
			end
		end
	end
			
	always @(*) begin //Start a combinational address mux
		casez (rbus_addr_in_buf)
			RBUS_OFFSET + 'h00 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l1_q1_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h01 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l1_q2_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h02 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l1_q3_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h03 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l2_q1_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h04 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l2_q2_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h05 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l2_q3_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h06 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l3_q1_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h07 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l3_q2_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h08 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l3_q3_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h09 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l4_q1_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h0A : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l4_q2_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h0B : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l4_q3_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h0C : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l5_q1_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h0D : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l5_q2_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h0E : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l5_q3_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h0F : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l6_q1_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h10 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l6_q2_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h11 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l6_q3_pos_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			
		
			RBUS_OFFSET + 'h20 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l1_q1_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h21 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l1_q2_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h22 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l1_q3_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h23 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l2_q1_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h24 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l2_q2_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h25 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l2_q3_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h26 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l3_q1_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h27 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l3_q2_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h28 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l3_q3_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h29 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l4_q1_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h2A : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l4_q2_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h2B : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l4_q3_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h2C : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l5_q1_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h2D : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l5_q2_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h2E : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l5_q3_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h2F : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l6_q1_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h30 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l6_q2_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h31 : begin
				rbus_data_out	= {{6{1'b0}}, dyna_l6_q3_pos_in};
				rbus_ack_out	= rbus_read_buf; 
			end
		
			RBUS_OFFSET + 'h40 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l1_q1_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h41 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l1_q2_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h42 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l1_q3_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h43 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l2_q1_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h44 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l2_q2_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h45 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l2_q3_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h46 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l3_q1_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h47 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l3_q2_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h48 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l3_q3_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h49 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l4_q1_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h4A : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l4_q2_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h4B : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l4_q3_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h4C : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l5_q1_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h4D : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l5_q2_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h4E : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l5_q3_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h4F : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l6_q1_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h50 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l6_q2_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
			end
			RBUS_OFFSET + 'h51 : begin
				rbus_data_out	= {{8{1'b0}}, dyna_l6_q3_errors_buf};
				rbus_ack_out	= rbus_read_buf; 
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
