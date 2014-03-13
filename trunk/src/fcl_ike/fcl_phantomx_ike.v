
// This module calculates the inverse kinematics for ALL joints on the PhantomX MkII Hexapod

						
module fcl_phantomx_ike
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 						DNET_ADDR_WIDTH = 16,
	parameter integer 						DNET_DATA_WIDTH = 32,
	parameter integer 						DNET_OFFSET = 0,
    parameter integer 						POS_WIDTH = 16,		// Bitwidth for spatial coordinates (16 sint) - Do not change
    parameter integer 						DYNA_WIDTH = 10,	
    parameter 		 						COSINE_LUT_FILE = "lut_cosine.mif",	
    parameter 		 						SINE_LUT_FILE = "lut_sine.mif"
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset_in,
    input 	wire			clk_in,
	
	// This will be replaced with DNET as an alternative input method
	input 	wire										start_in,
    input 	wire				[(POS_WIDTH-1):0]		delta_angle_x_in,
    input 	wire				[(POS_WIDTH-1):0]		delta_angle_y_in,
	
    input 	wire	signed		[(POS_WIDTH-1):0]		l1_x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l1_y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l1_z_pos_in,
	input 	wire	signed		[(POS_WIDTH-1):0]		l2_x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l2_y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l2_z_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l3_x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l3_y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l3_z_pos_in,
	input 	wire	signed		[(POS_WIDTH-1):0]		l4_x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l4_y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l4_z_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l5_x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l5_y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l5_z_pos_in,
	input 	wire	signed		[(POS_WIDTH-1):0]		l6_x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l6_y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		l6_z_pos_in,
	
    output 	wire										done_out,
    output	reg					[(DYNA_WIDTH-1):0]		l1_q1_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l1_q2_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l1_q3_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l2_q1_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l2_q2_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l2_q3_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l3_q1_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l3_q2_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l3_q3_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l4_q1_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l4_q2_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l4_q3_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l5_q1_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l5_q2_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l5_q3_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l6_q1_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l6_q2_dyna_out,
    output	reg					[(DYNA_WIDTH-1):0]		l6_q3_dyna_out
);


/*
// It seems we need some sort of actual table of which servo is where

   L1               L2
    \               /
   Q3O             O
      \           /
     Q2O  FRONT  O
        \       /
       Q1O-----O
         |     |
         |  T  |
 --O--O--O  O  O--O--O--
 L3      |  P  |      L4
         |     |
         O-----O
        /       \
       O         O
      /           \
     O             O
    /               \
   L5               L6
   
   
			DYNAMIXEL PORTS
			Q1		 Q2		 Q3
			AX18    AX12    AX12 
   LEG1		D_01	D_06	D_05
   LEG2		D_04	D_07	D_08
   LEG3		D_09	D_14	D_13
   LEG4		D_12	D_15	D_16
   LEG5		D_17	D_22	D_21
   LEG6		D_20	D_23	D_24
   
	SIGN MODIFIER (to be tested!)
			Q1		 Q2		 Q3
			AX18    AX12    AX12 
   LEG1		0		0		0
   LEG2		0		1		1
   LEG3		0		0		0
   LEG4		0		1		1
   LEG5		0		0		0
   LEG6		0		1		1

	ANGLE OFFSETS
			Q1		 Q2		 Q3		Q1		 Q2		 Q3
			AX18    AX12    AX12	AX18    AX12    AX12 
   LEG1		135		13.09   45.26	6000	094F	202F
   LEG2		45		13.09   45.26	2000	094F	202F
   LEG3		180		13.09   45.26	8000	094F	202F
   LEG4		0		13.09   45.26	0000	094F	202F
   LEG5		225		13.09   45.26	A000	094F	202F
   LEG6		315		13.09   45.26	E000	094F	202F

   
	POSITION OFFSETS (could be more precice, check with solidworks)
			X(mm)	Y(mm)	Z(mm)	X(dec)	Y(dec)	Z(dec) 
   LEG1		-60		+120.13	  0		-960	+1922	  0	
   LEG2		+60		+120.13	  0		+960	+1922	  0	
   LEG3		-100	  0		  0		-1600	   0	  0	
   LEG4		+100	  0		  0		+1600	   0	  0	
   LEG5		-60		-120.13	  0		-960	-1922	  0	
   LEG6		+60		-120.13	  0		+960	-1922	  0	
   
    
*/


	localparam	signed	[(POS_WIDTH-1):0]	L1_X_POS_OFFSET	= -960;
	localparam	signed	[(POS_WIDTH-1):0]	L1_Y_POS_OFFSET	= 1922;
	localparam	signed	[(POS_WIDTH-1):0]	L1_Z_POS_OFFSET	= 0;
	localparam			[0:0]				L1_Q1_REV		= 1'b0;
	localparam			[0:0]				L1_Q2_REV		= 1'b0;
	localparam			[0:0]				L1_Q3_REV		= 1'b0;
	localparam			[(POS_WIDTH-1):0]	L1_Q1_OFFSET	= 'h6000;
	localparam			[(POS_WIDTH-1):0]	L1_Q2_OFFSET	= 'h094F;
	localparam			[(POS_WIDTH-1):0]	L1_Q3_OFFSET	= 'h202F;
	
	localparam	signed	[(POS_WIDTH-1):0]	L2_X_POS_OFFSET	= 960;
	localparam	signed	[(POS_WIDTH-1):0]	L2_Y_POS_OFFSET	= 1922;
	localparam	signed	[(POS_WIDTH-1):0]	L2_Z_POS_OFFSET	= 0;
	localparam			[0:0]				L2_Q1_REV		= 1'b0;
	localparam			[0:0]				L2_Q2_REV		= 1'b1;
	localparam			[0:0]				L2_Q3_REV		= 1'b1;
	localparam			[(POS_WIDTH-1):0]	L2_Q1_OFFSET	= 'h2000;
	localparam			[(POS_WIDTH-1):0]	L2_Q2_OFFSET	= 'h094F;
	localparam			[(POS_WIDTH-1):0]	L2_Q3_OFFSET	= 'h202F;

	localparam	signed	[(POS_WIDTH-1):0]	L3_X_POS_OFFSET	= -1600;
	localparam	signed	[(POS_WIDTH-1):0]	L3_Y_POS_OFFSET	= 0;
	localparam	signed	[(POS_WIDTH-1):0]	L3_Z_POS_OFFSET	= 0;
	localparam			[0:0]				L3_Q1_REV		= 1'b0;
	localparam			[0:0]				L3_Q2_REV		= 1'b0;
	localparam			[0:0]				L3_Q3_REV		= 1'b0;
	localparam			[(POS_WIDTH-1):0]	L3_Q1_OFFSET	= 'h8000;
	localparam			[(POS_WIDTH-1):0]	L3_Q2_OFFSET	= 'h094F;
	localparam			[(POS_WIDTH-1):0]	L3_Q3_OFFSET	= 'h202F;
	
	localparam	signed	[(POS_WIDTH-1):0]	L4_X_POS_OFFSET	= 1600;
	localparam	signed	[(POS_WIDTH-1):0]	L4_Y_POS_OFFSET	= 0;
	localparam	signed	[(POS_WIDTH-1):0]	L4_Z_POS_OFFSET	= 0;
	localparam			[0:0]				L4_Q1_REV		= 1'b0;
	localparam			[0:0]				L4_Q2_REV		= 1'b1;
	localparam			[0:0]				L4_Q3_REV		= 1'b1;
	localparam			[(POS_WIDTH-1):0]	L4_Q1_OFFSET	= 'h0000;
	localparam			[(POS_WIDTH-1):0]	L4_Q2_OFFSET	= 'h094F;
	localparam			[(POS_WIDTH-1):0]	L4_Q3_OFFSET	= 'h202F;

	localparam	signed	[(POS_WIDTH-1):0]	L5_X_POS_OFFSET	= -960;
	localparam	signed	[(POS_WIDTH-1):0]	L5_Y_POS_OFFSET	= -1922;
	localparam	signed	[(POS_WIDTH-1):0]	L5_Z_POS_OFFSET	= 0;
	localparam			[0:0]				L5_Q1_REV		= 1'b0;
	localparam			[0:0]				L5_Q2_REV		= 1'b0;
	localparam			[0:0]				L5_Q3_REV		= 1'b0;
	localparam			[(POS_WIDTH-1):0]	L5_Q1_OFFSET	= 'hA000;
	localparam			[(POS_WIDTH-1):0]	L5_Q2_OFFSET	= 'h094F;
	localparam			[(POS_WIDTH-1):0]	L5_Q3_OFFSET	= 'h202F;
	
	localparam	signed	[(POS_WIDTH-1):0]	L6_X_POS_OFFSET	= 960;
	localparam	signed	[(POS_WIDTH-1):0]	L6_Y_POS_OFFSET	= -1922;
	localparam	signed	[(POS_WIDTH-1):0]	L6_Z_POS_OFFSET	= 0;
	localparam			[0:0]				L6_Q1_REV		= 1'b0;
	localparam			[0:0]				L6_Q2_REV		= 1'b1;
	localparam			[0:0]				L6_Q3_REV		= 1'b1;
	localparam			[(POS_WIDTH-1):0]	L6_Q1_OFFSET	= 'hE000;
	localparam			[(POS_WIDTH-1):0]	L6_Q2_OFFSET	= 'h094F;
	localparam			[(POS_WIDTH-1):0]	L6_Q3_OFFSET	= 'h202F;
	

	localparam [3:0]	iks_idle			= 'h00,
						iks_init			= 'h01,
						iks_leg_1_a			= 'h02,
						iks_leg_1_b			= 'h03,
						iks_leg_2_a			= 'h04,
						iks_leg_2_b			= 'h05,
						iks_leg_3_a			= 'h06,
						iks_leg_3_b			= 'h07,
						iks_leg_4_a			= 'h08,
						iks_leg_4_b			= 'h09,
						iks_leg_5_a			= 'h0A,
						iks_leg_5_b			= 'h0B,
						iks_leg_6_a			= 'h0C,
						iks_leg_6_b			= 'h0D,
						iks_done			= 'h0E;
						
						
	reg		[3:0]		ike_state;	// Controlling State Machine
	
	
    reg				[(POS_WIDTH-1):0]		delta_angle_x_buf;
    reg				[(POS_WIDTH-1):0]		delta_angle_y_buf;
    reg	signed		[(POS_WIDTH-1):0]		l1_x_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l1_y_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l1_z_pos_buf;
	reg	signed		[(POS_WIDTH-1):0]		l2_x_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l2_y_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l2_z_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l3_x_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l3_y_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l3_z_pos_buf;
	reg	signed		[(POS_WIDTH-1):0]		l4_x_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l4_y_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l4_z_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l5_x_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l5_y_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l5_z_pos_buf;
	reg	signed		[(POS_WIDTH-1):0]		l6_x_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l6_y_pos_buf;
    reg	signed		[(POS_WIDTH-1):0]		l6_z_pos_buf;
		
	
	
	wire									ike_start_in;
	reg		signed		[(POS_WIDTH-1):0]	ike_x_pos_in;
	reg		signed		[(POS_WIDTH-1):0]	ike_y_pos_in;
	reg		signed		[(POS_WIDTH-1):0]	ike_z_pos_in;	
	reg		signed		[(POS_WIDTH-1):0]	ike_x_pos_offset_in;
	reg		signed		[(POS_WIDTH-1):0]	ike_y_pos_offset_in;
	reg		signed		[(POS_WIDTH-1):0]	ike_z_pos_offset_in;	
	reg										ike_q1_dyna_rev_in;
	reg										ike_q2_dyna_rev_in;
	reg										ike_q3_dyna_rev_in;
	reg					[(POS_WIDTH-1):0]	ike_q1_dyna_offset_in;
	reg					[(POS_WIDTH-1):0]	ike_q2_dyna_offset_in;
	reg					[(POS_WIDTH-1):0]	ike_q3_dyna_offset_in;
	
	wire									ike_done_out;
	wire									ike_no_solution_out;
	wire				[(DYNA_WIDTH-1):0]	ike_q1_dyna_out;
	wire				[(DYNA_WIDTH-1):0]	ike_q2_dyna_out;
	wire				[(DYNA_WIDTH-1):0]	ike_q3_dyna_out;
	
	
	// Buffer Inputs
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
            delta_angle_x_buf <= {POS_WIDTH{1'b0}};
            delta_angle_y_buf <= {POS_WIDTH{1'b0}};
            l1_x_pos_buf <= {POS_WIDTH{1'b0}};
            l1_y_pos_buf <= {POS_WIDTH{1'b0}};
            l1_z_pos_buf <= {POS_WIDTH{1'b0}};
			l2_x_pos_buf <= {POS_WIDTH{1'b0}};
            l2_y_pos_buf <= {POS_WIDTH{1'b0}};
            l2_z_pos_buf <= {POS_WIDTH{1'b0}};
            l3_x_pos_buf <= {POS_WIDTH{1'b0}};
            l3_y_pos_buf <= {POS_WIDTH{1'b0}};
            l3_z_pos_buf <= {POS_WIDTH{1'b0}};
			l4_x_pos_buf <= {POS_WIDTH{1'b0}};
            l4_y_pos_buf <= {POS_WIDTH{1'b0}};
            l4_z_pos_buf <= {POS_WIDTH{1'b0}};
            l5_x_pos_buf <= {POS_WIDTH{1'b0}};
            l5_y_pos_buf <= {POS_WIDTH{1'b0}};
            l5_z_pos_buf <= {POS_WIDTH{1'b0}};
			l6_x_pos_buf <= {POS_WIDTH{1'b0}};
            l6_y_pos_buf <= {POS_WIDTH{1'b0}};
            l6_z_pos_buf <= {POS_WIDTH{1'b0}};
        end 
		else if (start_in) begin
            delta_angle_x_buf <= delta_angle_x_in;
            delta_angle_y_buf <= delta_angle_y_in;
            l1_x_pos_buf <= l1_x_pos_in;
            l1_y_pos_buf <= l1_y_pos_in;
            l1_z_pos_buf <= l1_z_pos_in;
			l2_x_pos_buf <= l2_x_pos_in;
            l2_y_pos_buf <= l2_y_pos_in;
            l2_z_pos_buf <= l2_z_pos_in;
            l3_x_pos_buf <= l3_x_pos_in;
            l3_y_pos_buf <= l3_y_pos_in;
            l3_z_pos_buf <= l3_z_pos_in;
			l4_x_pos_buf <= l4_x_pos_in;
            l4_y_pos_buf <= l4_y_pos_in;
            l4_z_pos_buf <= l4_z_pos_in;
            l5_x_pos_buf <= l5_x_pos_in;
            l5_y_pos_buf <= l5_y_pos_in;
            l5_z_pos_buf <= l5_z_pos_in;
			l6_x_pos_buf <= l6_x_pos_in;
            l6_y_pos_buf <= l6_y_pos_in;
            l6_z_pos_buf <= l6_z_pos_in;
		end
	end


	
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
            ike_state <= iks_idle;
			ike_x_pos_in <= {POS_WIDTH{1'b0}};
			ike_y_pos_in <= {POS_WIDTH{1'b0}};
			ike_z_pos_in <= {POS_WIDTH{1'b0}};			
			ike_x_pos_offset_in <= {POS_WIDTH{1'b0}};
			ike_y_pos_offset_in <= {POS_WIDTH{1'b0}};
			ike_z_pos_offset_in <= {POS_WIDTH{1'b0}};			
			ike_q1_dyna_rev_in <= 1'b0;
			ike_q2_dyna_rev_in <= 1'b0;
			ike_q3_dyna_rev_in <= 1'b0;
			ike_q1_dyna_offset_in <= {POS_WIDTH{1'b0}};
			ike_q2_dyna_offset_in <= {POS_WIDTH{1'b0}};
			ike_q3_dyna_offset_in <= {POS_WIDTH{1'b0}};
			l1_q1_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l1_q2_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l1_q3_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l2_q1_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l2_q2_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l2_q3_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l3_q1_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l3_q2_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l3_q3_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l4_q1_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l4_q2_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l4_q3_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l5_q1_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l5_q2_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l5_q3_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l6_q1_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l6_q2_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
			l6_q3_dyna_out <= {1'b1, {(DYNA_WIDTH-1){1'b0}}};
        end 
		else begin
			if (ike_no_solution_out) begin
				ike_state <= iks_init;
			end
			else begin
				case(ike_state)
					iks_idle: if (start_in) ike_state <= iks_init;
					iks_init: begin
						ike_x_pos_in <= l1_x_pos_buf;
						ike_y_pos_in <= l1_y_pos_buf;
						ike_z_pos_in <= l1_z_pos_buf;
						ike_x_pos_offset_in <= L1_X_POS_OFFSET;
						ike_y_pos_offset_in <= L1_Y_POS_OFFSET;
						ike_z_pos_offset_in <= L1_Z_POS_OFFSET;
						ike_q1_dyna_rev_in <= L1_Q1_REV;
						ike_q2_dyna_rev_in <= L1_Q2_REV;
						ike_q3_dyna_rev_in <= L1_Q3_REV;
						ike_q1_dyna_offset_in <= L1_Q1_OFFSET;
						ike_q2_dyna_offset_in <= L1_Q2_OFFSET;
						ike_q3_dyna_offset_in <= L1_Q3_OFFSET;
						ike_state <= iks_leg_1_a;
					end
					iks_leg_1_a: ike_state <= iks_leg_1_b;
					iks_leg_1_b: if (ike_done_out) begin
						if (!ike_no_solution_out) begin
							l1_q1_dyna_out <= ike_q1_dyna_out;
							l1_q2_dyna_out <= ike_q2_dyna_out;
							l1_q3_dyna_out <= ike_q3_dyna_out;
						end
						ike_x_pos_in <= l2_x_pos_buf;
						ike_y_pos_in <= l2_y_pos_buf;
						ike_z_pos_in <= l2_z_pos_buf;
						ike_x_pos_offset_in <= L2_X_POS_OFFSET;
						ike_y_pos_offset_in <= L2_Y_POS_OFFSET;
						ike_z_pos_offset_in <= L2_Z_POS_OFFSET;
						ike_q1_dyna_rev_in <= L2_Q1_REV;
						ike_q2_dyna_rev_in <= L2_Q2_REV;
						ike_q3_dyna_rev_in <= L2_Q3_REV;
						ike_q1_dyna_offset_in <= L2_Q1_OFFSET;
						ike_q2_dyna_offset_in <= L2_Q2_OFFSET;
						ike_q3_dyna_offset_in <= L2_Q3_OFFSET;
						ike_state <= iks_leg_2_a;
					end
					iks_leg_2_a: ike_state <= iks_leg_2_b;
					iks_leg_2_b: if (ike_done_out) begin
						if (!ike_no_solution_out) begin
							l2_q1_dyna_out <= ike_q1_dyna_out;
							l2_q2_dyna_out <= ike_q2_dyna_out;
							l2_q3_dyna_out <= ike_q3_dyna_out;
						end
						ike_x_pos_in <= l3_x_pos_buf;
						ike_y_pos_in <= l3_y_pos_buf;
						ike_z_pos_in <= l3_z_pos_buf;
						ike_x_pos_offset_in <= L3_X_POS_OFFSET;
						ike_y_pos_offset_in <= L3_Y_POS_OFFSET;
						ike_z_pos_offset_in <= L3_Z_POS_OFFSET;
						ike_q1_dyna_rev_in <= L3_Q1_REV;
						ike_q2_dyna_rev_in <= L3_Q2_REV;
						ike_q3_dyna_rev_in <= L3_Q3_REV;
						ike_q1_dyna_offset_in <= L3_Q1_OFFSET;
						ike_q2_dyna_offset_in <= L3_Q2_OFFSET;
						ike_q3_dyna_offset_in <= L3_Q3_OFFSET;
						ike_state <= iks_leg_3_a;
					end
					iks_leg_3_a: ike_state <= iks_leg_3_b;
					iks_leg_3_b: if (ike_done_out) begin
						if (!ike_no_solution_out) begin
							l3_q1_dyna_out <= ike_q1_dyna_out;
							l3_q2_dyna_out <= ike_q2_dyna_out;
							l3_q3_dyna_out <= ike_q3_dyna_out;
						end
						ike_x_pos_in <= l4_x_pos_buf;
						ike_y_pos_in <= l4_y_pos_buf;
						ike_z_pos_in <= l4_z_pos_buf;
						ike_x_pos_offset_in <= L4_X_POS_OFFSET;
						ike_y_pos_offset_in <= L4_Y_POS_OFFSET;
						ike_z_pos_offset_in <= L4_Z_POS_OFFSET;
						ike_q1_dyna_rev_in <= L4_Q1_REV;
						ike_q2_dyna_rev_in <= L4_Q2_REV;
						ike_q3_dyna_rev_in <= L4_Q3_REV;
						ike_q1_dyna_offset_in <= L4_Q1_OFFSET;
						ike_q2_dyna_offset_in <= L4_Q2_OFFSET;
						ike_q3_dyna_offset_in <= L4_Q3_OFFSET;
						ike_state <= iks_leg_4_a;
					end
					iks_leg_4_a: ike_state <= iks_leg_4_b;
					iks_leg_4_b: if (ike_done_out) begin
						if (!ike_no_solution_out) begin
							l4_q1_dyna_out <= ike_q1_dyna_out;
							l4_q2_dyna_out <= ike_q2_dyna_out;
							l4_q3_dyna_out <= ike_q3_dyna_out;
						end
						ike_x_pos_in <= l5_x_pos_buf;
						ike_y_pos_in <= l5_y_pos_buf;
						ike_z_pos_in <= l5_z_pos_buf;
						ike_x_pos_offset_in <= L5_X_POS_OFFSET;
						ike_y_pos_offset_in <= L5_Y_POS_OFFSET;
						ike_z_pos_offset_in <= L5_Z_POS_OFFSET;
						ike_q1_dyna_rev_in <= L5_Q1_REV;
						ike_q2_dyna_rev_in <= L5_Q2_REV;
						ike_q3_dyna_rev_in <= L5_Q3_REV;
						ike_q1_dyna_offset_in <= L5_Q1_OFFSET;
						ike_q2_dyna_offset_in <= L5_Q2_OFFSET;
						ike_q3_dyna_offset_in <= L5_Q3_OFFSET;
						ike_state <= iks_leg_5_a;
					end
					iks_leg_5_a: ike_state <= iks_leg_5_b;
					iks_leg_5_b: if (ike_done_out) begin
						if (!ike_no_solution_out) begin
							l5_q1_dyna_out <= ike_q1_dyna_out;
							l5_q2_dyna_out <= ike_q2_dyna_out;
							l5_q3_dyna_out <= ike_q3_dyna_out;
						end
						ike_x_pos_in <= l6_x_pos_buf;
						ike_y_pos_in <= l6_y_pos_buf;
						ike_z_pos_in <= l6_z_pos_buf;
						ike_x_pos_offset_in <= L6_X_POS_OFFSET;
						ike_y_pos_offset_in <= L6_Y_POS_OFFSET;
						ike_z_pos_offset_in <= L6_Z_POS_OFFSET;
						ike_q1_dyna_rev_in <= L6_Q1_REV;
						ike_q2_dyna_rev_in <= L6_Q2_REV;
						ike_q3_dyna_rev_in <= L6_Q3_REV;
						ike_q1_dyna_offset_in <= L6_Q1_OFFSET;
						ike_q2_dyna_offset_in <= L6_Q2_OFFSET;
						ike_q3_dyna_offset_in <= L6_Q3_OFFSET;
						ike_state <= iks_leg_6_a;
					end
					iks_leg_6_a: ike_state <= iks_leg_6_b;
					iks_leg_6_b: if (ike_done_out) begin
						if (!ike_no_solution_out) begin
							l6_q1_dyna_out <= ike_q1_dyna_out;
							l6_q2_dyna_out <= ike_q2_dyna_out;
							l6_q3_dyna_out <= ike_q3_dyna_out;
						end
						ike_state <= iks_done;
					end
					iks_done: ike_state <= iks_idle;
					default:	ike_state <= iks_idle;
				endcase
			end
		end
	end
	
	
	assign ike_start_in = (ike_state == iks_leg_1_a) || (ike_state == iks_leg_2_a) || (ike_state == iks_leg_3_a) || (ike_state == iks_leg_4_a) || (ike_state == iks_leg_5_a) || (ike_state == iks_leg_6_a);
	assign done_out = (ike_state == iks_done);
	
	//------------------------------------------------------
	//--			Inverse Kinematics Engine		 	  --
	//------------------------------------------------------
	fcl_inv_kinematics #(
			.DNET_ADDR_WIDTH(DNET_ADDR_WIDTH),
			.DNET_DATA_WIDTH(DNET_DATA_WIDTH),
			.DNET_OFFSET(DNET_OFFSET),
			.POS_WIDTH(POS_WIDTH),
			.DYNA_WIDTH(DYNA_WIDTH),
			.COSINE_LUT_FILE("lut_cosine.mif"),
			.SINE_LUT_FILE("lut_sine.mif"))
		ike (
			._reset_in(_reset_in),
			.clk_in(clk_in),
			
			.start_in(ike_start_in),
			.x_pos_in(ike_x_pos_in),
			.y_pos_in(ike_y_pos_in),
			.z_pos_in(ike_z_pos_in),			
			.delta_angle_x_in(delta_angle_x_buf),			
			.delta_angle_y_in(delta_angle_y_buf),			
			.x_pos_offset_in(ike_x_pos_offset_in),
			.y_pos_offset_in(ike_y_pos_offset_in),
			.z_pos_offset_in(ike_z_pos_offset_in),
			
			.q1_dyna_rev_in(ike_q1_dyna_rev_in),
			.q2_dyna_rev_in(ike_q2_dyna_rev_in),
			.q3_dyna_rev_in(ike_q3_dyna_rev_in),
			.q1_dyna_offset_in(ike_q1_dyna_offset_in),
			.q2_dyna_offset_in(ike_q2_dyna_offset_in),
			.q3_dyna_offset_in(ike_q3_dyna_offset_in),
			
			.done_out(ike_done_out),
			.no_solution_out(),
			.q1_dyna_out(ike_q1_dyna_out),
			.q2_dyna_out(ike_q2_dyna_out),
			.q3_dyna_out(ike_q3_dyna_out),
			.q1_phase_out(),
			.q2_phase_out(),
			.q3_phase_out());

	assign ike_no_solution_out = 1'b0;
	
			
endmodule
