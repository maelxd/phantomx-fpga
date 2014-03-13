

module fcl_sequencer
	//------------------------------------------------------
	//--				External Parameters				  --
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
	//--					Ports						  --
	//------------------------------------------------------
(
	// System Signals
	input	wire			_reset_in,
	input	wire			clk_in,
	
	input	wire										sync_pulse_in,		// Initiate a Processing Step
	input	wire				[(POS_WIDTH-1):0]		delta_angle_x_in,
	input	wire				[(POS_WIDTH-1):0]		delta_angle_y_in,
	input	wire				[(POS_WIDTH-1):0]		speed_in,			// 8.8
	input	wire										rotate_mode_in,
	input	wire				[(POS_WIDTH-1):0]		z_offset_in,

	// Dynamixel Interfaces
	output	wire										done_out,
	output	wire				[(DYNA_WIDTH-1):0]		l1_q1_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l1_q2_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l1_q3_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l2_q1_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l2_q2_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l2_q3_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l3_q1_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l3_q2_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l3_q3_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l4_q1_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l4_q2_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l4_q3_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l5_q1_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l5_q2_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l5_q3_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l6_q1_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l6_q2_dyna_out,
	output	wire				[(DYNA_WIDTH-1):0]		l6_q3_dyna_out
);


	localparam integer MIN_SPEED = 32; // Till stationary
	localparam integer STAT_ZONE = MIN_SPEED + 32; // When to go again!
	localparam integer STATIONARY_HOLD_OFF = 80; 
	

	// Parameterization, to simplify the basic walking pattern
	localparam integer Z_GROUND = -2200; 
	localparam integer Z_RAISED = -1900; 

	
	// Walk Parameters
	localparam integer WALK_X_SPREAD_VALUE = 3100; 
	localparam integer WALK_X_OUTER_LEGS = WALK_X_SPREAD_VALUE; 
	localparam integer WALK_X_MIDDLE_LEGS = WALK_X_SPREAD_VALUE + 640; // This fixes the wobble (due to inv kin errors)
	localparam integer WALK_Y_FRONT_LEGS = 2700; 
	localparam integer WALK_Y_MIDDLE_LEGS = 0; 
	localparam integer WALK_Y_REAR_LEGS = -2700; 
	localparam integer WALK_Y_POS_0 = 650; 
	localparam integer WALK_Y_POS_1 = 585; 
	localparam integer WALK_Y_POS_2 = 520; 
	localparam integer WALK_Y_POS_3 = -520; 
	localparam integer WALK_Y_POS_4 = -585; 
	localparam integer WALK_Y_POS_5 = -650; 
		
	// Rotate Parameters
	localparam integer ROT_X_SPREAD_VALUE = 3100; 
	localparam integer ROT_X_OUTER_LEGS = ROT_X_SPREAD_VALUE; 
	localparam integer ROT_X_MIDDLE_LEGS = ROT_X_SPREAD_VALUE + 640; // This fixes the wobble (due to inv kin errors)
	localparam integer ROT_Y_FRONT_LEGS = 2700; 
	localparam integer ROT_Y_MIDDLE_LEGS = 0; 
	localparam integer ROT_Y_REAR_LEGS = -2700;
	localparam integer ROT_Y_MIDDLE_POS_0 = 650; 
	localparam integer ROT_Y_MIDDLE_POS_1 = 585; 
	localparam integer ROT_Y_MIDDLE_POS_2 = 520; 
	localparam integer ROT_Y_MIDDLE_POS_3 = -520; 
	localparam integer ROT_Y_MIDDLE_POS_4 = -585; 
	localparam integer ROT_Y_MIDDLE_POS_5 = -650; 
	
	localparam integer ROT_X_CORNER_POS_0 = 436; 
	localparam integer ROT_X_CORNER_POS_1 = 392; 
	localparam integer ROT_X_CORNER_POS_2 = 349; 
	localparam integer ROT_X_CORNER_POS_3 = -349; 
	localparam integer ROT_X_CORNER_POS_4 = -392; 
	localparam integer ROT_X_CORNER_POS_5 = -436; 
	localparam integer ROT_Y_CORNER_POS_0 = 484; 
	localparam integer ROT_Y_CORNER_POS_1 = 436; 
	localparam integer ROT_Y_CORNER_POS_2 = 387; 
	localparam integer ROT_Y_CORNER_POS_3 = -387; 
	localparam integer ROT_Y_CORNER_POS_4 = -436; 
	localparam integer ROT_Y_CORNER_POS_5 = -484; 
	
	
	
	
	// Default Positions
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L1_X_POS = -WALK_X_OUTER_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L1_Y_POS = WALK_Y_FRONT_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L1_Z_POS = Z_GROUND;

	localparam	signed	[(POS_WIDTH-1):0]	INIT_L2_X_POS = WALK_X_OUTER_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L2_Y_POS = WALK_Y_FRONT_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L2_Z_POS = Z_GROUND;

	localparam	signed	[(POS_WIDTH-1):0]	INIT_L3_X_POS = -WALK_X_MIDDLE_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L3_Y_POS = WALK_Y_MIDDLE_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L3_Z_POS = Z_GROUND;

	localparam	signed	[(POS_WIDTH-1):0]	INIT_L4_X_POS = WALK_X_MIDDLE_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L4_Y_POS = WALK_Y_MIDDLE_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L4_Z_POS = Z_GROUND;

	localparam	signed	[(POS_WIDTH-1):0]	INIT_L5_X_POS = -WALK_X_OUTER_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L5_Y_POS = WALK_Y_REAR_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L5_Z_POS = Z_GROUND;

	localparam	signed	[(POS_WIDTH-1):0]	INIT_L6_X_POS = WALK_X_OUTER_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L6_Y_POS = WALK_Y_REAR_LEGS;
	localparam	signed	[(POS_WIDTH-1):0]	INIT_L6_Z_POS = Z_GROUND;

	
	// Input targets to the interpolation
	reg		signed		[(POS_WIDTH-1):0]		l1_x_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l1_y_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l1_z_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l2_x_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l2_y_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l2_z_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l3_x_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l3_y_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l3_z_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l4_x_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l4_y_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l4_z_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l5_x_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l5_y_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l5_z_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l6_x_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l6_y_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		l6_z_pos_target;
	reg		signed		[(POS_WIDTH-1):0]		z_offset_buf;

	reg								interp_load;
	reg		[(POS_WIDTH-1):0]		target_time_in;		// Keep all the interpolation synchronized
	reg		[(POS_WIDTH-1):0]		speed_buf;			// Keep all the interpolation synchronized
	reg								dir_rev_buf;		
	reg		[(POS_WIDTH-1):0]		target_speed_in;	// Keep all the interpolation synchronized
	wire	[5:0]					interp_sync_out;
	wire	[5:0]					interp_done;
	
	reg		[1:0]					stationary_buf;
	reg								stationary_re_buf;	
	reg		[1:0]					go_buf;
	reg								rotate_mode_buf;
	reg		[1:0]					go_re_buf;
	reg		[7:0]					stationary_count;

	// Signals linking the interpolation to the IKE
	wire	signed		[(POS_WIDTH-1):0]		l1_x_pos;
	wire	signed		[(POS_WIDTH-1):0]		l1_y_pos;
	wire	signed		[(POS_WIDTH-1):0]		l1_z_pos;
	wire	signed		[(POS_WIDTH-1):0]		l2_x_pos;
	wire	signed		[(POS_WIDTH-1):0]		l2_y_pos;
	wire	signed		[(POS_WIDTH-1):0]		l2_z_pos;
	wire	signed		[(POS_WIDTH-1):0]		l3_x_pos;
	wire	signed		[(POS_WIDTH-1):0]		l3_y_pos;
	wire	signed		[(POS_WIDTH-1):0]		l3_z_pos;
	wire	signed		[(POS_WIDTH-1):0]		l4_x_pos;
	wire	signed		[(POS_WIDTH-1):0]		l4_y_pos;
	wire	signed		[(POS_WIDTH-1):0]		l4_z_pos;
	wire	signed		[(POS_WIDTH-1):0]		l5_x_pos;
	wire	signed		[(POS_WIDTH-1):0]		l5_y_pos;
	wire	signed		[(POS_WIDTH-1):0]		l5_z_pos;
	wire	signed		[(POS_WIDTH-1):0]		l6_x_pos;
	wire	signed		[(POS_WIDTH-1):0]		l6_y_pos;
	wire	signed		[(POS_WIDTH-1):0]		l6_z_pos;

	
	localparam [4:0]	iks_stationary1				= 'h00,
						iks_stationary2				= 'h01,
						iks_walk_fwd_stage1			= 'h02,
						iks_walk_fwd_stage2			= 'h03,
						iks_walk_fwd_stage3			= 'h04,
						iks_walk_fwd_stage4			= 'h05,
						iks_walk_fwd_stage5			= 'h06,
						iks_walk_fwd_stage6			= 'h07,
						iks_walk_rev_stage1			= 'h08,
						iks_walk_rev_stage2			= 'h09,
						iks_walk_rev_stage3			= 'h0A,
						iks_walk_rev_stage4			= 'h0B,
						iks_walk_rev_stage5			= 'h0C,
						iks_walk_rev_stage6			= 'h0D,
						iks_rotate_right_stage1		= 'h0E,
						iks_rotate_right_stage2		= 'h0F,
						iks_rotate_right_stage3		= 'h10,
						iks_rotate_right_stage4		= 'h11,
						iks_rotate_right_stage5		= 'h12,
						iks_rotate_right_stage6		= 'h13,
						iks_rotate_left_stage1		= 'h14,
						iks_rotate_left_stage2		= 'h15,
						iks_rotate_left_stage3		= 'h16,
						iks_rotate_left_stage4		= 'h17,
						iks_rotate_left_stage5		= 'h18,
						iks_rotate_left_stage6		= 'h19;
						
	reg		[4:0]		ike_state;	// Controlling State Machine
	reg					sync_pulse_buf;
	
	
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			stationary_buf <= 2'b00;
			stationary_re_buf <= 1'b0;			
			go_buf <= 2'b00;
			go_re_buf <= 2'b00;
			sync_pulse_buf <= 1'b0;
			rotate_mode_buf <= 1'b0;
			interp_load <= 1'b0;
			ike_state <= iks_stationary1;
			target_time_in <= {POS_WIDTH{1'b0}};
			z_offset_buf <= {POS_WIDTH{1'b0}};
			speed_buf <= 16'h0100;			// 8.8, so this is 1.0
			dir_rev_buf <= 1'b0;	
			target_speed_in <= 16'h0100;	// 8.8, so this is 1.0
			stationary_count <= {8{1'b0}};
			l1_x_pos_target <= INIT_L1_X_POS;
			l1_y_pos_target <= INIT_L1_Y_POS;
			l1_z_pos_target <= INIT_L1_Z_POS;
			l2_x_pos_target <= INIT_L2_X_POS;
			l2_y_pos_target <= INIT_L2_Y_POS;
			l2_z_pos_target <= INIT_L2_Z_POS;
			l3_x_pos_target <= INIT_L3_X_POS;
			l3_y_pos_target <= INIT_L3_Y_POS;
			l3_z_pos_target <= INIT_L3_Z_POS;
			l4_x_pos_target <= INIT_L4_X_POS;
			l4_y_pos_target <= INIT_L4_Y_POS;
			l4_z_pos_target <= INIT_L4_Z_POS;
			l5_x_pos_target <= INIT_L5_X_POS;
			l5_y_pos_target <= INIT_L5_Y_POS;
			l5_z_pos_target <= INIT_L5_Z_POS;
			l6_x_pos_target <= INIT_L6_X_POS;
			l6_y_pos_target <= INIT_L6_Y_POS;
			l6_z_pos_target <= INIT_L6_Z_POS;
        end 
		else begin
			z_offset_buf <= z_offset_in;
			sync_pulse_buf <= sync_pulse_in;
			rotate_mode_buf <= rotate_mode_in;
			interp_load <= interp_done[0] || stationary_re_buf || go_re_buf[1];
			
			speed_buf <= speed_in[15] ? (~speed_in) : speed_in;	 // Just how necessary is this reactivity?
			dir_rev_buf <= speed_in[15];
			
			stationary_buf[1:0] <= {stationary_buf[0], (speed_buf < MIN_SPEED)};
			stationary_re_buf <= (stationary_buf[0] && (!stationary_buf[1]));
			
			go_buf[1:0] <= {go_buf[0], (speed_buf > STAT_ZONE)};
			go_re_buf[1:0] <= {go_re_buf[0], (go_buf[0] && (!go_buf[1]))};

			if (stationary_buf[0] && (!stationary_buf[1])) begin
				ike_state <= iks_stationary1;
				stationary_count <= {8{1'b0}};
			end
			else begin
				if ((stationary_count < STATIONARY_HOLD_OFF) && sync_pulse_buf) begin
					stationary_count <= stationary_count + 1'b1;
				end

				case(ike_state)
					iks_stationary1: begin
						target_time_in <= 300;
						target_speed_in <= 16'h0100;
						
						l1_x_pos_target <= INIT_L1_X_POS;
						l1_y_pos_target <= INIT_L1_Y_POS;
						l1_z_pos_target <= INIT_L1_Z_POS;
						l2_x_pos_target <= INIT_L2_X_POS;
						l2_y_pos_target <= INIT_L2_Y_POS;
						l2_z_pos_target <= INIT_L2_Z_POS;
						l3_x_pos_target <= INIT_L3_X_POS;
						l3_y_pos_target <= INIT_L3_Y_POS;
						l3_z_pos_target <= INIT_L3_Z_POS;
						l4_x_pos_target <= INIT_L4_X_POS;
						l4_y_pos_target <= INIT_L4_Y_POS;
						l4_z_pos_target <= INIT_L4_Z_POS;
						l5_x_pos_target <= INIT_L5_X_POS;
						l5_y_pos_target <= INIT_L5_Y_POS;
						l5_z_pos_target <= INIT_L5_Z_POS;
						l6_x_pos_target <= INIT_L6_X_POS;
						l6_y_pos_target <= INIT_L6_Y_POS;
						l6_z_pos_target <= INIT_L6_Z_POS;

						if (go_buf[0] && (!go_buf[1]))
							ike_state <= iks_stationary2;
					end
					iks_stationary2: begin
						if ((stationary_count == STATIONARY_HOLD_OFF) && (go_buf[0])) begin
							if (rotate_mode_buf) begin
								if (dir_rev_buf)
									ike_state <= iks_rotate_left_stage1;
								else
									ike_state <= iks_rotate_right_stage1;
							end
							else begin
								if (dir_rev_buf)
									ike_state <= iks_walk_rev_stage1;
								else
									ike_state <= iks_walk_fwd_stage1;
							end
						end
					end
					
					// Rotate Right
					iks_rotate_right_stage1: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_4;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_4;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_4;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_4;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_1;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_1;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_4;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_4;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_4;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_4;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_right_stage2;
						end
					end
					
					iks_rotate_right_stage2: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_5;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_5;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_3;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_3;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_2;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_0;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_5;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_5;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_3;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_3;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_right_stage3;
						end
					end
					
					iks_rotate_right_stage3: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_0;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_0;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_2;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_2;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_3;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_5;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_0;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_0;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_2;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_2;
						l6_z_pos_target <= Z_GROUND;
					
						if (interp_done[0]) begin
							ike_state <= iks_rotate_right_stage4;
						end
					end
					
					iks_rotate_right_stage4: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_1;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_1;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_1;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_1;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_4;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_4;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_1;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_1;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_1;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_1;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_right_stage5;
						end
					end
					
					iks_rotate_right_stage5: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_2;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_2;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_0;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_0;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_5;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_3;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_2;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_2;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_0;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_0;
						l6_z_pos_target <= Z_RAISED;
					
						if (interp_done[0]) begin
							ike_state <= iks_rotate_right_stage6;
						end
					end
					
					iks_rotate_right_stage6: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_3;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_3;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_5;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_5;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_0;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_2;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_3;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_3;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_5;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_5;
						l6_z_pos_target <= Z_RAISED;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_right_stage1;
						end
					end
				
				
					// Rotate Left
					iks_rotate_left_stage1: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_1;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_1;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_1;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_1;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_4;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_4;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_1;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_1;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_1;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_1;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_left_stage2;
						end
					end
					
					iks_rotate_left_stage2: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_0;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_0;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_2;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_2;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_3;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_5;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_0;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_0;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_2;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_2;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_left_stage3;
						end
					end
					
					iks_rotate_left_stage3: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_5;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_5;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_3;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_3;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_2;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_0;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_5;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_5;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_3;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_3;
						l6_z_pos_target <= Z_GROUND;
					
						if (interp_done[0]) begin
							ike_state <= iks_rotate_left_stage4;
						end
					end
					
					iks_rotate_left_stage4: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_4;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_4;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_4;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_4;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_1;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_1;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_4;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_4;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_4;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_4;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_left_stage5;
						end
					end
					
					iks_rotate_left_stage5: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_3;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_3;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_5;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_5;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_0;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_2;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_3;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_3;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_5;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_5;
						l6_z_pos_target <= Z_RAISED;
					
						if (interp_done[0]) begin
							ike_state <= iks_rotate_left_stage6;
						end
					end
					
					iks_rotate_left_stage6: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_2;
						l1_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_2;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_0;
						l2_y_pos_target <= ROT_Y_FRONT_LEGS + ROT_Y_CORNER_POS_0;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -ROT_X_MIDDLE_LEGS;
						l3_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_5;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= ROT_X_MIDDLE_LEGS;
						l4_y_pos_target <= ROT_Y_MIDDLE_LEGS + ROT_Y_MIDDLE_POS_3;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -ROT_X_OUTER_LEGS - ROT_X_CORNER_POS_2;
						l5_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_2;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= ROT_X_OUTER_LEGS + ROT_X_CORNER_POS_0;
						l6_y_pos_target <= ROT_Y_REAR_LEGS + ROT_Y_CORNER_POS_0;
						l6_z_pos_target <= Z_RAISED;
						
						if (interp_done[0]) begin
							ike_state <= iks_rotate_left_stage1;
						end
					end
				
				
					// Walk Forward
					iks_walk_fwd_stage1: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_4;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_1;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_1;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_4;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_4;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_1;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_fwd_stage2;
					end
					
					iks_walk_fwd_stage2: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_5;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_2;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_2;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_5;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_5;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_2;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_fwd_stage3;
					end
					
					iks_walk_fwd_stage3: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_0;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_3;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_3;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_0;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_0;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_3;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_fwd_stage4;
					end
					
					iks_walk_fwd_stage4: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_1;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_4;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_4;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_1;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_1;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_4;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_fwd_stage5;
					end
					
					iks_walk_fwd_stage5: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_2;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_5;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_5;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_2;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_2;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_5;
						l6_z_pos_target <= Z_RAISED;
						
						if (interp_done[0])
							ike_state <= iks_walk_fwd_stage6;
					end
					
					iks_walk_fwd_stage6: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_3;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_0;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_0;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_3;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_3;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_0;
						l6_z_pos_target <= Z_RAISED;
						
						if (interp_done[0])
							ike_state <= iks_walk_fwd_stage1;
					end
			
					// Walk Reverse
					iks_walk_rev_stage1: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_1;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_4;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_4;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_1;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_1;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_4;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_rev_stage2;
					end
					
					iks_walk_rev_stage2: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_0;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_3;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_3;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_0;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_0;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_3;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_rev_stage3;
					end
					
					iks_walk_rev_stage3: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_5;
						l1_z_pos_target <= Z_RAISED;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_2;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_2;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_5;
						l4_z_pos_target <= Z_RAISED;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_5;
						l5_z_pos_target <= Z_RAISED;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_2;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_rev_stage4;
					end
					
					iks_walk_rev_stage4: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_4;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_1;
						l2_z_pos_target <= Z_GROUND;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_1;
						l3_z_pos_target <= Z_GROUND;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_4;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_4;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_1;
						l6_z_pos_target <= Z_GROUND;
						
						if (interp_done[0])
							ike_state <= iks_walk_rev_stage5;
					end
					
					iks_walk_rev_stage5: begin
						target_time_in <= 25;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_3;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_0;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_0;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_3;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_3;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_0;
						l6_z_pos_target <= Z_RAISED;
						
						if (interp_done[0])
							ike_state <= iks_walk_rev_stage6;
					end
					
					iks_walk_rev_stage6: begin
						target_time_in <= 350;
						target_speed_in <= speed_buf;
						
						l1_x_pos_target <= -WALK_X_OUTER_LEGS;
						l1_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_2;
						l1_z_pos_target <= Z_GROUND;
						l2_x_pos_target <= WALK_X_OUTER_LEGS;
						l2_y_pos_target <= WALK_Y_FRONT_LEGS + WALK_Y_POS_5;
						l2_z_pos_target <= Z_RAISED;

						l3_x_pos_target <= -WALK_X_MIDDLE_LEGS;
						l3_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_5;
						l3_z_pos_target <= Z_RAISED;
						l4_x_pos_target <= WALK_X_MIDDLE_LEGS;
						l4_y_pos_target <= WALK_Y_MIDDLE_LEGS + WALK_Y_POS_2;
						l4_z_pos_target <= Z_GROUND;

						l5_x_pos_target <= -WALK_X_OUTER_LEGS;
						l5_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_2;
						l5_z_pos_target <= Z_GROUND;
						l6_x_pos_target <= WALK_X_OUTER_LEGS;
						l6_y_pos_target <= WALK_Y_REAR_LEGS + WALK_Y_POS_5;
						l6_z_pos_target <= Z_RAISED;
						
						if (interp_done[0])
							ike_state <= iks_walk_rev_stage1;
					end
					
					default:	ike_state <= iks_stationary1;
				endcase
			end
		end
	end
	
	
	
	//------------------------------------------------------
	//--			Interpolation Modules				  --
	//------------------------------------------------------
	fcl_interp #(
		.INIT_X_POS(INIT_L1_X_POS),
		.INIT_Y_POS(INIT_L1_Y_POS),
		.INIT_Z_POS(INIT_L1_Z_POS))
	interpolation_l1 (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		.sync_pulse_in(sync_pulse_buf),
		.set_current_pos_in(1'b0),	// Only used for debug or as an alternative to the INIT params
		.set_target_pos_in(interp_load),
		.set_target_time_in(interp_load),
		.set_target_speed_in(sync_pulse_buf), // Always be updating the speed!
		.target_x_pos_in(l1_x_pos_target),
		.target_y_pos_in(l1_y_pos_target),
		.target_z_pos_in(l1_z_pos_target+z_offset_buf),
		.target_time_in(target_time_in), 
		.target_speed_in(target_speed_in), 
		.current_x_pos_in({POS_WIDTH{1'b0}}),
		.current_y_pos_in({POS_WIDTH{1'b0}}),
		.current_z_pos_in({POS_WIDTH{1'b0}}),
		.sync_pulse_out(interp_sync_out[0]),
		.interp_done_out(interp_done[0]),
		.new_x_pos_out(l1_x_pos),
		.new_y_pos_out(l1_y_pos),
		.new_z_pos_out(l1_z_pos),
		.current_time_out());

	fcl_interp #(
		.INIT_X_POS(INIT_L2_X_POS),
		.INIT_Y_POS(INIT_L2_Y_POS),
		.INIT_Z_POS(INIT_L2_Z_POS))
	interpolation_l2 (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		.sync_pulse_in(sync_pulse_buf),
		.set_current_pos_in(1'b0),	// Only used for debug or as an alternative to the INIT params
		.set_target_pos_in(interp_load),
		.set_target_time_in(interp_load),
		.set_target_speed_in(sync_pulse_buf), // Always be updating the speed!
		.target_x_pos_in(l2_x_pos_target),
		.target_y_pos_in(l2_y_pos_target),
		.target_z_pos_in(l2_z_pos_target+z_offset_buf),
		.target_time_in(target_time_in), 
		.target_speed_in(target_speed_in), 
		.current_x_pos_in({POS_WIDTH{1'b0}}),
		.current_y_pos_in({POS_WIDTH{1'b0}}),
		.current_z_pos_in({POS_WIDTH{1'b0}}),
		.sync_pulse_out(interp_sync_out[1]),
		.interp_done_out(interp_done[1]),
		.new_x_pos_out(l2_x_pos),
		.new_y_pos_out(l2_y_pos),
		.new_z_pos_out(l2_z_pos),
		.current_time_out());

	fcl_interp #(
		.INIT_X_POS(INIT_L3_X_POS),
		.INIT_Y_POS(INIT_L3_Y_POS),
		.INIT_Z_POS(INIT_L3_Z_POS))
	interpolation_l3 (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		.sync_pulse_in(sync_pulse_buf),
		.set_current_pos_in(1'b0),	// Only used for debug or as an alternative to the INIT params
		.set_target_pos_in(interp_load),
		.set_target_time_in(interp_load),
		.set_target_speed_in(sync_pulse_buf), // Always be updating the speed!
		.target_x_pos_in(l3_x_pos_target),
		.target_y_pos_in(l3_y_pos_target),
		.target_z_pos_in(l3_z_pos_target+z_offset_buf),
		.target_time_in(target_time_in), 
		.target_speed_in(target_speed_in), 
		.current_x_pos_in({POS_WIDTH{1'b0}}),
		.current_y_pos_in({POS_WIDTH{1'b0}}),
		.current_z_pos_in({POS_WIDTH{1'b0}}),
		.sync_pulse_out(interp_sync_out[2]),
		.interp_done_out(interp_done[2]),
		.new_x_pos_out(l3_x_pos),
		.new_y_pos_out(l3_y_pos),
		.new_z_pos_out(l3_z_pos),
		.current_time_out());
		
	fcl_interp #(
		.INIT_X_POS(INIT_L4_X_POS),
		.INIT_Y_POS(INIT_L4_Y_POS),
		.INIT_Z_POS(INIT_L4_Z_POS))
	interpolation_l4 (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		.sync_pulse_in(sync_pulse_buf),
		.set_current_pos_in(1'b0),	// Only used for debug or as an alternative to the INIT params
		.set_target_pos_in(interp_load),
		.set_target_time_in(interp_load),
		.set_target_speed_in(sync_pulse_buf), // Always be updating the speed!
		.target_x_pos_in(l4_x_pos_target),
		.target_y_pos_in(l4_y_pos_target),
		.target_z_pos_in(l4_z_pos_target+z_offset_buf),
		.target_time_in(target_time_in), 
		.target_speed_in(target_speed_in), 
		.current_x_pos_in({POS_WIDTH{1'b0}}),
		.current_y_pos_in({POS_WIDTH{1'b0}}),
		.current_z_pos_in({POS_WIDTH{1'b0}}),
		.sync_pulse_out(interp_sync_out[3]),
		.interp_done_out(interp_done[3]),
		.new_x_pos_out(l4_x_pos),
		.new_y_pos_out(l4_y_pos),
		.new_z_pos_out(l4_z_pos),
		.current_time_out());
		
	fcl_interp #(
		.INIT_X_POS(INIT_L5_X_POS),
		.INIT_Y_POS(INIT_L5_Y_POS),
		.INIT_Z_POS(INIT_L5_Z_POS))
	interpolation_l5 (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		.sync_pulse_in(sync_pulse_buf),
		.set_current_pos_in(1'b0),	// Only used for debug or as an alternative to the INIT params
		.set_target_pos_in(interp_load),
		.set_target_time_in(interp_load),
		.set_target_speed_in(sync_pulse_buf), // Always be updating the speed!
		.target_x_pos_in(l5_x_pos_target),
		.target_y_pos_in(l5_y_pos_target),
		.target_z_pos_in(l5_z_pos_target+z_offset_buf),
		.target_time_in(target_time_in), 
		.target_speed_in(target_speed_in), 
		.current_x_pos_in({POS_WIDTH{1'b0}}),
		.current_y_pos_in({POS_WIDTH{1'b0}}),
		.current_z_pos_in({POS_WIDTH{1'b0}}),
		.sync_pulse_out(interp_sync_out[4]),
		.interp_done_out(interp_done[4]),
		.new_x_pos_out(l5_x_pos),
		.new_y_pos_out(l5_y_pos),
		.new_z_pos_out(l5_z_pos),
		.current_time_out());
		
	fcl_interp #(
		.INIT_X_POS(INIT_L6_X_POS),
		.INIT_Y_POS(INIT_L6_Y_POS),
		.INIT_Z_POS(INIT_L6_Z_POS))
	interpolation_l6 (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		.sync_pulse_in(sync_pulse_buf),
		.set_current_pos_in(1'b0),	// Only used for debug or as an alternative to the INIT params
		.set_target_pos_in(interp_load),
		.set_target_time_in(interp_load),
		.set_target_speed_in(sync_pulse_buf), // Always be updating the speed!
		.target_x_pos_in(l6_x_pos_target),
		.target_y_pos_in(l6_y_pos_target),
		.target_z_pos_in(l6_z_pos_target+z_offset_buf),
		.target_time_in(target_time_in), 
		.target_speed_in(target_speed_in), 
		.current_x_pos_in({POS_WIDTH{1'b0}}),
		.current_y_pos_in({POS_WIDTH{1'b0}}),
		.current_z_pos_in({POS_WIDTH{1'b0}}),
		.sync_pulse_out(interp_sync_out[5]),
		.interp_done_out(interp_done[5]),
		.new_x_pos_out(l6_x_pos),
		.new_y_pos_out(l6_y_pos),
		.new_z_pos_out(l6_z_pos),
		.current_time_out());


	//------------------------------------------------------
	//--			Inverse Kinematics Wrapper			  --
	//------------------------------------------------------
	fcl_phantomx_ike #(
		.DNET_ADDR_WIDTH(DNET_ADDR_WIDTH),
		.DNET_DATA_WIDTH(DNET_DATA_WIDTH),
		.DNET_OFFSET(DNET_OFFSET),
		.POS_WIDTH(POS_WIDTH))
	ike (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		
		.start_in(interp_sync_out[0]),
		
		.delta_angle_x_in(delta_angle_x_in),
		.delta_angle_y_in(delta_angle_y_in),
		
		.l1_x_pos_in(l1_x_pos),
		.l1_y_pos_in(l1_y_pos),
		.l1_z_pos_in(l1_z_pos),
		
		.l2_x_pos_in(l2_x_pos),
		.l2_y_pos_in(l2_y_pos),
		.l2_z_pos_in(l2_z_pos),
		
		.l3_x_pos_in(l3_x_pos),
		.l3_y_pos_in(l3_y_pos),
		.l3_z_pos_in(l3_z_pos),
		
		.l4_x_pos_in(l4_x_pos),
		.l4_y_pos_in(l4_y_pos),
		.l4_z_pos_in(l4_z_pos),
		
		.l5_x_pos_in(l5_x_pos),
		.l5_y_pos_in(l5_y_pos),
		.l5_z_pos_in(l5_z_pos),
		
		.l6_x_pos_in(l6_x_pos),
		.l6_y_pos_in(l6_y_pos),
		.l6_z_pos_in(l6_z_pos),

		.done_out(done_out),
		.l1_q1_dyna_out(l1_q1_dyna_out),
		.l1_q2_dyna_out(l1_q2_dyna_out),
		.l1_q3_dyna_out(l1_q3_dyna_out),
		.l2_q1_dyna_out(l2_q1_dyna_out),
		.l2_q2_dyna_out(l2_q2_dyna_out),
		.l2_q3_dyna_out(l2_q3_dyna_out),
		.l3_q1_dyna_out(l3_q1_dyna_out),
		.l3_q2_dyna_out(l3_q2_dyna_out),
		.l3_q3_dyna_out(l3_q3_dyna_out),
		.l4_q1_dyna_out(l4_q1_dyna_out),
		.l4_q2_dyna_out(l4_q2_dyna_out),
		.l4_q3_dyna_out(l4_q3_dyna_out),
		.l5_q1_dyna_out(l5_q1_dyna_out),
		.l5_q2_dyna_out(l5_q2_dyna_out),
		.l5_q3_dyna_out(l5_q3_dyna_out),
		.l6_q1_dyna_out(l6_q1_dyna_out),
		.l6_q2_dyna_out(l6_q2_dyna_out),
		.l6_q3_dyna_out(l6_q3_dyna_out));
		
endmodule
