
// This module calculates the inverse kinematics for a SINGLE hexapod leg (of the type used by the PhantomX II)

	//------------------------------------------------------
	//--				IK Equations				 	  --
	//------------------------------------------------------
	// theta_1 = atan2(y/x)
	// x_1 = LENGTH_1 * cos(theta_1)
	// y_1 = LENGTH_1 * sin(theta_1)
	// xy_squared = (x - x_1)^2 + (y - y_1)^2 
	// c_1 = (xy_squared + z^2 - LENGTH_2^2 - LENGTH_3^2)
	// c_2 = sqrt((4*LENGTH_2^2*LENGTH_3^2) - c_1^2)
	// theta_3 = atan2(c_2/c_1)
	// theta_2 = atan2(z/sqrt(xy_squared)) - atan2(c_2/(2*LENGTH_2^2 + c_1))



	//------------------------------------------------------
	//--				PhantomX Parameters			 	  --
	//------------------------------------------------------
	// Total Movement space = 700mm x 740mm = +-350mm and +-370mm from the centre of the robot
	// Current bitwidth definition: 16sint = +-11.4bits (allows +-2047mm with an accuracy of 0.0625mm)
	// Using this representation, the lengths of each arm are given by:

	//	L1 		= 53		= 848
	//	L2 		= 66.22		= 1060
	//	L2t 	= 13.09		= 
	//	L3 		= 132.67	= 2123
	//	L3t 	= 45.256	= 


	
	
	
	

module fcl_inv_kinematics
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 						DNET_ADDR_WIDTH = 16,
	parameter integer 						DNET_DATA_WIDTH = 32,
	parameter integer 						DNET_OFFSET = 0,
    parameter integer 						POS_WIDTH = 16,		// Bitwidth for spatial coordinates (16 sint) - Do not change
    parameter integer 						DYNA_WIDTH = 10,	
    parameter signed	[(POS_WIDTH-1):0] 	LENGTH_1 = 848,		// 53.00mm - In units of POS_WIDTH
    parameter signed	[(POS_WIDTH-1):0] 	LENGTH_2 = 1060, //1052, //1060,	// 66.25mm - In units of POS_WIDTH	
    parameter signed	[(POS_WIDTH-1):0] 	LENGTH_3 = 2123, //2015, //2123,	// 132.69mm - In units of POS_WIDTH
    parameter signed	[(POS_WIDTH-1):0] 	MIN_RADIUS_LIMIT = 1552,	// 97.00mm - In units of POS_WIDTH
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
	
	input 	wire										start_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		x_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		y_pos_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		z_pos_in,
    input 	wire				[(POS_WIDTH-1):0]		delta_angle_x_in,	// Small angles ONLY!
    input 	wire				[(POS_WIDTH-1):0]		delta_angle_y_in,	// Small angles ONLY!
    input 	wire	signed		[(POS_WIDTH-1):0]		x_pos_offset_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		y_pos_offset_in,
    input 	wire	signed		[(POS_WIDTH-1):0]		z_pos_offset_in,
    input 	wire										q1_dyna_rev_in,
    input 	wire										q2_dyna_rev_in,
    input 	wire										q3_dyna_rev_in,
    input 	wire				[(POS_WIDTH-1):0]		q1_dyna_offset_in,
    input 	wire				[(POS_WIDTH-1):0]		q2_dyna_offset_in,
    input 	wire				[(POS_WIDTH-1):0]		q3_dyna_offset_in,

    output 	reg											done_out,
    output 	reg											no_solution_out,
    output 	wire				[(POS_WIDTH-1):0]		q1_phase_out,
    output 	wire				[(POS_WIDTH-1):0]		q2_phase_out,
    output 	wire				[(POS_WIDTH-1):0]		q3_phase_out,
    output 	reg					[(DYNA_WIDTH-1):0]		q1_dyna_out,
    output 	reg					[(DYNA_WIDTH-1):0]		q2_dyna_out,
    output 	reg					[(DYNA_WIDTH-1):0]		q3_dyna_out
);

	// These have been replaced by constants (as OTHERWISE, ISE seems to get most of these wrong)
    localparam	integer					 		DYNA_SCALE = 1229; //((2**16)*360*(2**DYNA_WIDTH)/(300*(2**POS_WIDTH)));
	localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_23_SQR = 32'h0055EB09; //(LENGTH_2*LENGTH_2)+(LENGTH_3*LENGTH_3);
	localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_2PLUS3_SQR = 32'h009A9821; //(LENGTH_2+LENGTH_3)*(LENGTH_2+LENGTH_3);
	localparam	signed	[((POS_WIDTH*4)-1):0]	LENGTH_SL23_PARAM = 64'h0000126C69C97240; //4*LENGTH_2*LENGTH_2*LENGTH_3*LENGTH_3; // This parameter fails to compute in Xilinx ISE
	localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_2_2_SQR = 32'h00224A20; //2*LENGTH_2*LENGTH_2;
	localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_MIN_RAD_SQR = 32'h0024C100; //MIN_RADIUS_LIMIT*MIN_RADIUS_LIMIT;
	localparam	integer							MULT_PIPELINE = 8;	// Can be adjusted slightly (> 5 and < 14)

	// OLD
	// L2 = 1060
	// L3 = 2123
	// LENGTH_23_SQR = 5630729
	// LENGTH_2PLUS3_SQR = 10131489
	// LENGTH_SL23_PARAM = 20256840580000
	// LENGTH_2_2_SQR = 2247200
	
	// NEW
	// L2 = 1052 = 65.75mm
	// L3 = 2015 = 125.92mm
	// LENGTH_23_SQR = 5166929 = 4ED751
	// LENGTH_2PLUS3_SQR = 9406489 = 8F8819
	// LENGTH_SL23_PARAM = 17973868993600 = 1058DE139C40
	// LENGTH_2_2_SQR = 2213408 = 21C620
	
	//localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_23_SQR = 32'h004ED751; //(LENGTH_2*LENGTH_2)+(LENGTH_3*LENGTH_3);
	//localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_2PLUS3_SQR = 32'h008F8819; //(LENGTH_2+LENGTH_3)*(LENGTH_2+LENGTH_3);
	//localparam	signed	[((POS_WIDTH*4)-1):0]	LENGTH_SL23_PARAM = 64'h00001058DE139C40; //4*LENGTH_2*LENGTH_2*LENGTH_3*LENGTH_3; // This parameter fails to compute in Xilinx ISE
	//localparam	signed	[((POS_WIDTH*2)-1):0]	LENGTH_2_2_SQR = 32'h0021C620; //2*LENGTH_2*LENGTH_2;
	
	
	localparam [5:0]	ks_idle					= 'h00,
						ks_stage_0_a			= 'h01,
						ks_stage_0_b			= 'h02,
						ks_stage_0_c			= 'h03,
						ks_stage_0_d			= 'h04,
						ks_stage_0_e			= 'h05,
						ks_stage_0_f			= 'h06,
						ks_stage_0_g			= 'h07,
						ks_stage_0_h			= 'h08,
						ks_stage_0_i			= 'h09,
						ks_stage_0_j			= 'h0A,
						ks_stage_0_k			= 'h0B,
						ks_stage_0_l			= 'h0C,
						ks_stage_1_a			= 'h0D,
						ks_stage_1_b			= 'h0E,
						ks_stage_1_c			= 'h0F,
						ks_stage_2_a			= 'h10,
						ks_stage_2_b			= 'h11,
						ks_stage_2_c			= 'h12,
						ks_stage_2_d			= 'h13,
						ks_stage_2_e			= 'h14,
						ks_stage_2_f			= 'h15,
						ks_stage_2_g			= 'h16,
						ks_stage_2_h			= 'h17,
						ks_stage_2_i			= 'h18,
						ks_stage_2_j			= 'h19,
						ks_stage_3_a			= 'h1A,
						ks_stage_3_b			= 'h1B,
						ks_stage_3_c			= 'h1C,
						ks_stage_3_d			= 'h1D,
						ks_stage_3_e			= 'h1E,
						ks_stage_4_a			= 'h1F,
						ks_stage_4_b			= 'h20,
						ks_stage_4_c			= 'h21,
						ks_stage_4_d			= 'h22,
						ks_stage_4_e			= 'h23,
						ks_stage_4_f			= 'h24,
						ks_stage_4_g			= 'h25,
						ks_stage_4_h			= 'h26,
						ks_stage_4_i			= 'h27,
						ks_stage_5_a			= 'h28,
						ks_stage_5_b			= 'h29,
						ks_stage_5_c			= 'h2A,
						ks_stage_5_d			= 'h2B;
						
						
	reg		[5:0]		kin_state;	// Controlling State Machine
	
	// Input Buffering
    reg		signed		[(POS_WIDTH-1):0]		x_pos_buf;
    reg		signed		[(POS_WIDTH-1):0]		y_pos_buf;
    reg		signed		[(POS_WIDTH-1):0]		z_pos_buf;        
	reg		signed		[(POS_WIDTH-1):0]		x_pos_offset_buf;
    reg		signed		[(POS_WIDTH-1):0]		y_pos_offset_buf;
    reg		signed		[(POS_WIDTH-1):0]		z_pos_offset_buf;    
    reg					[(POS_WIDTH-1):0]		delta_angle_x_buf;    
    reg					[(POS_WIDTH-1):0]		delta_angle_y_buf;    
    reg											q1_dyna_rev_buf;
    reg											q2_dyna_rev_buf;
    reg											q3_dyna_rev_buf;
	reg					[(POS_WIDTH-1):0]		q1_dyna_offset_buf;
    reg					[(POS_WIDTH-1):0]		q2_dyna_offset_buf;
    reg					[(POS_WIDTH-1):0]		q3_dyna_offset_buf;
	
	reg 	signed 		[((2*POS_WIDTH)-1):0] 	temp_var_a;
	reg 	signed 		[((2*POS_WIDTH)-1):0] 	xy_squared;
	reg 	signed 		[((2*POS_WIDTH)-1):0] 	c_1;
	reg 	signed 		[((2*POS_WIDTH)-1):0] 	c_2;
	
	reg					[(POS_WIDTH-1):0]		theta1_reg;
	reg					[(POS_WIDTH-1):0]		theta2_reg;
	reg					[(POS_WIDTH-1):0]		theta3_reg;
	
	// Signals to help the synthesiser (helps it identify repeated operations)
	wire 	signed 		[((POS_WIDTH*2)-1):0] 	c1_calc_node;
    wire				[(DYNA_WIDTH-1):0]		dyna_angle_node;

	
	// Arc Tan Signals
	wire										atc_start;
	reg		signed		[((2*POS_WIDTH)-1):0]	atc_x_data;
	reg		signed		[((2*POS_WIDTH)-1):0]	atc_y_data;
	wire										atc_done;
	wire				[((2*POS_WIDTH)-1):0]	atc_q_data;

	// Sine and Cosine Signals
	reg					[11:0]				sine_data;
	wire	signed		[(POS_WIDTH-1):0]	sine_q;
	reg					[11:0]				cosine_data;
	wire	signed		[(POS_WIDTH-1):0]	cosine_q;
	
	// Multiplier Signals
	wire 		 								mult_start;
	wire 		 								mult_done;
	reg 		 		[3:0] 					mult_delay;
	reg 	signed 		[((POS_WIDTH*2)-1):0] 	mult_in_a;
	reg 	signed 		[((POS_WIDTH*2)-1):0] 	mult_in_b;
	wire 	signed 		[((POS_WIDTH*4)-1):0] 	mult_out_q;

	// Square Root Signals
	wire										sqrt_start;
	reg					[((POS_WIDTH*4)-1):0]	sqrt_data_in;
	wire										sqrt_done;
	wire				[((POS_WIDTH*2)-1):0]	sqrt_data_out;


	//------------------------------------------------------
	//--					ATAN2 Function			 	  --
	//------------------------------------------------------
	fcl_arctan #(
		.CORDIC_WIDTH(2*POS_WIDTH))
	arctan_cordic (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		
		.start_in(atc_start),
		.x_data_in(atc_x_data),
		.y_data_in(atc_y_data),
		.done_out(atc_done),
		.q_data_out(atc_q_data));
	
	assign atc_start = (kin_state == ks_stage_1_b) || (kin_state == ks_stage_4_b) || (kin_state == ks_stage_4_e) || (kin_state == ks_stage_4_h);


	//------------------------------------------------------
	//--					SINE Function			 	  --
	//------------------------------------------------------
	// LUT - 12bit addressing, 16bit result
    sine_lut sine_function (
		.clka(clk_in),
        .wea(1'b0),
        .dina({16{1'b0}}),
		.addra(sine_data), 
		.douta(sine_q));



	//------------------------------------------------------
	//--					COSINE Function			 	  --
	//------------------------------------------------------
	// LUT - 12bit addressing, 16bit result
    cosine_lut cosine_function (
		.clka(clk_in),
        .wea(1'b0),
        .dina({16{1'b0}}),
		.addra(cosine_data), 
		.douta(cosine_q));

			
	//------------------------------------------------------
	//--				Multiplier Function			 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			mult_delay <= 4'b1111;
		end
		else begin
			if (mult_start) begin
				mult_delay <= 4'b0000;
			end
			else if (!(&mult_delay)) begin
				mult_delay <= mult_delay + 1'b1;
			end
		end
	end
		
	assign mult_start = (kin_state == ks_stage_0_e) || (kin_state == ks_stage_0_k) || (kin_state == ks_stage_2_e) || (kin_state == ks_stage_2_h) || (kin_state == ks_stage_3_b) || (kin_state == ks_stage_5_a); 
	assign mult_done = (mult_delay == (MULT_PIPELINE-1));

    fcl_ike_mult ike_mult_function (
		.clk(clk_in),
		.a(mult_in_a), 
		.b(mult_in_b), 
		.p(mult_out_q));

	
	//------------------------------------------------------
	//--					SQRT Function			 	  --
	//------------------------------------------------------
	fcl_sqrt #(
		.DATA_WIDTH_IN(4*POS_WIDTH),
		.DATA_WIDTH_OUT(2*POS_WIDTH))
	sqrt_function (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		
		.start_in(sqrt_start),
		.data_in(sqrt_data_in),
		.done_out(sqrt_done),
		.data_out(sqrt_data_out));
	
	assign sqrt_start = (kin_state == ks_stage_3_d) || (kin_state == ks_stage_4_b);

	
	//------------------------------------------------------
	//--			Controlling State Machine		 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
            kin_state <= ks_idle;
			
			x_pos_buf <= {POS_WIDTH{1'b0}};
			y_pos_buf <= {POS_WIDTH{1'b0}};
			z_pos_buf <= {POS_WIDTH{1'b0}};			
			x_pos_offset_buf <= {POS_WIDTH{1'b0}};
			y_pos_offset_buf <= {POS_WIDTH{1'b0}};
			z_pos_offset_buf <= {POS_WIDTH{1'b0}};
			delta_angle_x_buf <= {POS_WIDTH{1'b0}};
			delta_angle_y_buf <= {POS_WIDTH{1'b0}};
			q1_dyna_rev_buf <= 1'b0;
			q2_dyna_rev_buf <= 1'b0;
			q3_dyna_rev_buf <= 1'b0;
			q1_dyna_offset_buf <= {POS_WIDTH{1'b0}};
			q2_dyna_offset_buf <= {POS_WIDTH{1'b0}};
			q3_dyna_offset_buf <= {POS_WIDTH{1'b0}};	
			
            atc_x_data <= {POS_WIDTH{1'b0}};
            atc_y_data <= {POS_WIDTH{1'b0}};
            theta1_reg <= {POS_WIDTH{1'b0}};
            theta2_reg <= {POS_WIDTH{1'b0}};
            theta3_reg <= {POS_WIDTH{1'b0}};
            sine_data <= {12{1'b0}};	// This WILL change
            cosine_data <= {12{1'b0}};	// This WILL change
		
            mult_in_a <= {(2*POS_WIDTH){1'b0}};
            mult_in_b <= {(2*POS_WIDTH){1'b0}};
            temp_var_a <= {(2*POS_WIDTH){1'b0}};
            xy_squared <= {(2*POS_WIDTH){1'b0}};
            sqrt_data_in <= {(4*POS_WIDTH){1'b0}};
            c_1 <= {(2*POS_WIDTH){1'b0}};
            c_2 <= {(2*POS_WIDTH){1'b0}};
			
            q1_dyna_out <= {DYNA_WIDTH{1'b0}};
            q2_dyna_out <= {DYNA_WIDTH{1'b0}};
            q3_dyna_out <= {DYNA_WIDTH{1'b0}};
			done_out <= 1'b0;
			no_solution_out <= 1'b0;
        end 
		else begin
			case(kin_state)
				ks_idle: begin
					done_out <= 1'b0;
					no_solution_out <= 1'b0;
					if (start_in) begin
						x_pos_buf <= x_pos_in;	// Sign expansion is automatic for signed values
						y_pos_buf <= y_pos_in;
						z_pos_buf <= z_pos_in;
						x_pos_offset_buf <= x_pos_offset_in;
						y_pos_offset_buf <= y_pos_offset_in;
						z_pos_offset_buf <= z_pos_offset_in;
						delta_angle_x_buf <= delta_angle_x_in;
						delta_angle_y_buf <= delta_angle_y_in;
						q1_dyna_rev_buf <= q1_dyna_rev_in;
						q2_dyna_rev_buf <= q2_dyna_rev_in;
						q3_dyna_rev_buf <= q3_dyna_rev_in;
						q1_dyna_offset_buf <= q1_dyna_offset_in;
						q2_dyna_offset_buf <= q2_dyna_offset_in;
						q3_dyna_offset_buf <= q3_dyna_offset_in;
						kin_state <= ks_stage_0_a;
					end
				end
				
				ks_stage_0_a: begin
					sine_data <= delta_angle_x_buf[15:4];
					kin_state <= ks_stage_0_b;
				end
				ks_stage_0_b: kin_state <= ks_stage_0_c;
				ks_stage_0_c: kin_state <= ks_stage_0_d;
				ks_stage_0_d: begin
					mult_in_a <= x_pos_buf + x_pos_offset_buf;
					mult_in_b <= sine_q;
					kin_state <= ks_stage_0_e;
				end
				ks_stage_0_e: kin_state <= ks_stage_0_f;
				ks_stage_0_f: if (mult_done) begin
					z_pos_buf <= z_pos_buf - (mult_out_q >>> 16);
					kin_state <= ks_stage_0_g;
				end
				ks_stage_0_g: begin
					sine_data <= delta_angle_y_buf[15:4];
					kin_state <= ks_stage_0_h;
				end
				ks_stage_0_h: kin_state <= ks_stage_0_i;
				ks_stage_0_i: kin_state <= ks_stage_0_j;
				ks_stage_0_j: begin
					mult_in_a <= y_pos_buf + y_pos_offset_buf;
					mult_in_b <= sine_q;
					kin_state <= ks_stage_0_k;
				end
				ks_stage_0_k: kin_state <= ks_stage_0_l;
				ks_stage_0_l: if (mult_done) begin
					x_pos_buf <= x_pos_buf - x_pos_offset_buf;
					y_pos_buf <= y_pos_buf - y_pos_offset_buf;
					z_pos_buf <= z_pos_buf - (mult_out_q >>> 16) - z_pos_offset_buf;
					kin_state <= ks_stage_1_a;
				end
				
				ks_stage_1_a: begin
					atc_x_data <= x_pos_buf;
					atc_y_data <= y_pos_buf;
					kin_state <= ks_stage_1_b;
				end
				ks_stage_1_b: kin_state <= ks_stage_1_c;
				ks_stage_1_c: if (atc_done) begin
					theta1_reg <= atc_q_data[31:16];
					kin_state <= ks_stage_2_a;
				end
				ks_stage_2_a: begin
					sine_data <= theta1_reg[15:4];
					cosine_data <= theta1_reg[15:4];
					kin_state <= ks_stage_2_b;
				end
				ks_stage_2_b: kin_state <= ks_stage_2_c;
				ks_stage_2_c: kin_state <= ks_stage_2_d;
				ks_stage_2_d: begin
					mult_in_a <= LENGTH_1;
					mult_in_b <= cosine_q;
					kin_state <= ks_stage_2_e;
				end				
				ks_stage_2_e: begin
					mult_in_a <= LENGTH_1;
					mult_in_b <= sine_q;
					kin_state <= ks_stage_2_f;
				end
				ks_stage_2_f: if (mult_done) begin
					temp_var_a <= mult_out_q;
					kin_state <= ks_stage_2_g;
				end
				ks_stage_2_g: begin
					temp_var_a <= mult_out_q;
					mult_in_a <= x_pos_buf - (temp_var_a >>> (POS_WIDTH-1));
					mult_in_b <= x_pos_buf - (temp_var_a >>> (POS_WIDTH-1));
					kin_state <= ks_stage_2_h;
				end
				ks_stage_2_h: begin
					mult_in_a <= y_pos_buf - (temp_var_a >>> (POS_WIDTH-1));
					mult_in_b <= y_pos_buf - (temp_var_a >>> (POS_WIDTH-1));
					kin_state <= ks_stage_2_i;
				end				
				ks_stage_2_i: begin
					mult_in_a <= z_pos_buf;
					mult_in_b <= z_pos_buf;
					if (mult_done) begin
						temp_var_a <= mult_out_q;
						kin_state <= ks_stage_2_j;
					end
				end
				ks_stage_2_j: begin
					xy_squared <= temp_var_a + mult_out_q;
					kin_state <= ks_stage_3_a;
				end
				ks_stage_3_a: begin
					if (((xy_squared + mult_out_q) < LENGTH_MIN_RAD_SQR) || ((xy_squared + mult_out_q) > LENGTH_2PLUS3_SQR)) begin
						no_solution_out <= 1'b1;
					end
					c_1 <= c1_calc_node;
					mult_in_a <= c1_calc_node;
					mult_in_b <= c1_calc_node;
					kin_state <= ks_stage_3_b;
				end
				ks_stage_3_b: kin_state <= ks_stage_3_c;
				ks_stage_3_c: if (mult_done) begin
					sqrt_data_in <= LENGTH_SL23_PARAM - mult_out_q; // Must be positive
					kin_state <= ks_stage_3_d;
				end
				ks_stage_3_d: kin_state <= ks_stage_3_e;
				ks_stage_3_e: if (sqrt_done) begin
					c_2 <= sqrt_data_out;
					kin_state <= ks_stage_4_a;
				end
				ks_stage_4_a: begin
					atc_x_data <= c_1;
					atc_y_data <= c_2;
					sqrt_data_in <= xy_squared;
					kin_state <= ks_stage_4_b;
				end
				ks_stage_4_b: kin_state <= ks_stage_4_c;
				ks_stage_4_c: begin
					if (sqrt_done) begin
						temp_var_a <= sqrt_data_out;
					end
					
					if (atc_done) begin
						theta3_reg <= atc_q_data[31:16];
						kin_state <= ks_stage_4_d;
					end
				end
				ks_stage_4_d: begin
					atc_x_data <= temp_var_a;
					atc_y_data <= $signed(~$unsigned(z_pos_buf[(POS_WIDTH-1):0]) + 1'b1);
					kin_state <= ks_stage_4_e;
				end
				ks_stage_4_e: kin_state <= ks_stage_4_f;
				ks_stage_4_f: if (atc_done) begin
					theta2_reg <= atc_q_data[31:16];
					kin_state <= ks_stage_4_g;
				end
				ks_stage_4_g: begin
					atc_x_data <= LENGTH_2_2_SQR + c_1;
					atc_y_data <= c_2;
					kin_state <= ks_stage_4_h;
				end
				ks_stage_4_h: kin_state <= ks_stage_4_i;
				ks_stage_4_i: if (atc_done) begin
					theta2_reg <= theta2_reg - atc_q_data[31:16];
					mult_in_a <= $signed(theta1_reg - q1_dyna_offset_buf);
					mult_in_b <= $signed({1'b0, DYNA_SCALE});
					kin_state <= ks_stage_5_a;
				end
				ks_stage_5_a: begin
					mult_in_a <= $signed(theta2_reg - q2_dyna_offset_buf);
					mult_in_b <= $signed({1'b0, DYNA_SCALE});
					kin_state <= ks_stage_5_b;
				end
				ks_stage_5_b: begin
					mult_in_a <= $signed(theta3_reg - q3_dyna_offset_buf);
					mult_in_b <= $signed({1'b0, DYNA_SCALE});
					if (mult_done) begin
						q1_dyna_out <= q1_dyna_rev_buf ? (16'h0200 - dyna_angle_node) : (16'h0200 + dyna_angle_node);
						kin_state <= ks_stage_5_c;
					end
				end
				ks_stage_5_c: begin
					q2_dyna_out <= q2_dyna_rev_buf ? (16'h0200 - dyna_angle_node) : (16'h0200 + dyna_angle_node);
					kin_state <= ks_stage_5_d;
				end
				ks_stage_5_d: begin
					q3_dyna_out <= q3_dyna_rev_buf ? (16'h0200 - dyna_angle_node) : (16'h0200 + dyna_angle_node);
					done_out <= 1'b1;
					kin_state <= ks_idle;
				end

				default: kin_state <= ks_idle;
			endcase
		end
	end
	
	assign c1_calc_node = xy_squared + mult_out_q - LENGTH_23_SQR;
	assign dyna_angle_node = $unsigned(mult_out_q >>> 16);

	assign q1_phase_out = theta1_reg;
	assign q2_phase_out = theta2_reg;
	assign q3_phase_out = theta3_reg;

endmodule
