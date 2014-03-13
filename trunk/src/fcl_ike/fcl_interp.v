
/*
A special function that handles all interpolation of positions.
The feature that makes this interpolation special, is that the 
target position, time and speed can all be adjusted during the 
interpolation operation. 
This makes for a much more dynamic interpolation engine!


*/


module fcl_interp #(
	//------------------------------------------------------
	//--				External Parameters				  --
	//------------------------------------------------------
	parameter	integer	POS_WIDTH	= 16,						// Do not change
	parameter	integer	DEC_WIDTH	= 8,						// Do not change
	parameter	integer	DIV_WIDTH	= POS_WIDTH + DEC_WIDTH,	// Do not change
	parameter	integer	INIT_X_POS	= 0,
	parameter	integer	INIT_Y_POS	= 0,
	parameter	integer	INIT_Z_POS	= 0
)
	//------------------------------------------------------
	//--					Ports						  --
	//------------------------------------------------------
(
	// System Signals
	input	wire									_reset_in,
	input	wire									clk_in,
	
	// Operation Input
	input	wire									sync_pulse_in,
	
	// Target Position
	input	wire									set_target_pos_in,
	input	wire	signed		[(POS_WIDTH-1):0]	target_x_pos_in,
	input	wire	signed		[(POS_WIDTH-1):0]	target_y_pos_in,
	input	wire	signed		[(POS_WIDTH-1):0]	target_z_pos_in,
	
	// Target Speed
	input	wire									set_target_speed_in,
	input	wire				[(POS_WIDTH-1):0]	target_speed_in,	// Defined as a decimal of 8.8
	
	// Target Time
	input	wire									set_target_time_in,
	input	wire				[(POS_WIDTH-1):0]	target_time_in,
	
	// Current Position Input (may be removed later)
	input	wire									set_current_pos_in,	// To be used with feedback or with forward kinematics
	input	wire	signed		[(POS_WIDTH-1):0]	current_x_pos_in,	// To be used with feedback or with forward kinematics
	input	wire	signed		[(POS_WIDTH-1):0]	current_y_pos_in,	// To be used with feedback or with forward kinematics
	input	wire	signed		[(POS_WIDTH-1):0]	current_z_pos_in,	// To be used with feedback or with forward kinematics
	
	// Operation Output
	output 	wire									sync_pulse_out,
	output 	wire									interp_done_out,
	output	wire	signed		[(POS_WIDTH-1):0]	new_x_pos_out,
	output	wire	signed		[(POS_WIDTH-1):0]	new_y_pos_out,
	output	wire	signed		[(POS_WIDTH-1):0]	new_z_pos_out,
	output	wire				[(POS_WIDTH-1):0]	current_time_out
);

	localparam [3:0]		is_idle				= 'h0,
							is_t_comp			= 'h1,
							is_t_div_a			= 'h2,
							is_t_div_b			= 'h3,
							is_t_eval			= 'h4,
							is_x_comp			= 'h5,
							is_x_div_a			= 'h6,
							is_x_div_b			= 'h7,
							is_y_comp			= 'h8,
							is_y_div_a			= 'h9,
							is_y_div_b			= 'hA,
							is_z_comp			= 'hB,
							is_z_div_a			= 'hC,
							is_z_div_b			= 'hD,
							is_done				= 'hE,
							is_final_done		= 'hF;

							
	reg		[5:0]			interp_state;	// Controlling State Machine
	
	reg						negative_diff;		// Keeps track if an ABS has altered the input to the divider

	// Input Buffers
	reg	signed	[(DIV_WIDTH-1):0]	target_x_pos_buf;
	reg	signed	[(DIV_WIDTH-1):0]	target_y_pos_buf;
	reg	signed	[(DIV_WIDTH-1):0]	target_z_pos_buf;
	reg			[(DIV_WIDTH-1):0]	target_time_buf;
	reg			[(DIV_WIDTH-1):0]	target_speed_buf;
	reg	signed	[(DIV_WIDTH-1):0]	current_x_pos_buf;
	reg	signed	[(DIV_WIDTH-1):0]	current_y_pos_buf;
	reg	signed	[(DIV_WIDTH-1):0]	current_z_pos_buf;
	reg			[(DIV_WIDTH-1):0]	current_x_rem_buf;
	reg			[(DIV_WIDTH-1):0]	current_y_rem_buf;
	reg			[(DIV_WIDTH-1):0]	current_z_rem_buf;
	reg			[(DIV_WIDTH-1):0]	recalc_goal_time;
	reg			[(DIV_WIDTH-1):0]	current_time_count;
	
	// Division Signals
	wire						div_start_node;
	wire						div_done_node;
	reg		[(DIV_WIDTH-1):0]	div_num_in;
	reg		[(DIV_WIDTH-1):0]	div_den_in;
	wire	[(DIV_WIDTH-1):0]	div_quotient_node;
	wire	[(DIV_WIDTH-1):0]	div_quotient_product;
	wire	[(DIV_WIDTH-1):0]	div_remainder_node;

	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			target_x_pos_buf <= {INIT_X_POS[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
			target_y_pos_buf <= {INIT_Y_POS[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
			target_z_pos_buf <= {INIT_Z_POS[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
			target_speed_buf <= {{(POS_WIDTH-1){1'b0}}, 1'b1, {DEC_WIDTH{1'b0}}};
			target_time_buf <= {DIV_WIDTH{1'b0}};
			current_x_pos_buf <= {INIT_X_POS[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
			current_y_pos_buf <= {INIT_Y_POS[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
			current_z_pos_buf <= {INIT_Z_POS[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
			current_x_rem_buf <= {DIV_WIDTH{1'b0}};
			current_y_rem_buf <= {DIV_WIDTH{1'b0}};
			current_z_rem_buf <= {DIV_WIDTH{1'b0}};
			current_time_count <= {DIV_WIDTH{1'b0}};
			recalc_goal_time <= {DIV_WIDTH{1'b0}};
			div_num_in <= {DIV_WIDTH{1'b0}};
			div_den_in <= {DIV_WIDTH{1'b0}};
			interp_state <= is_idle;
			negative_diff <= 1'b0;
		end 
		else begin
			case(interp_state)
				is_idle: begin
					if (set_current_pos_in) begin
						current_x_pos_buf <= {current_x_pos_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
						current_y_pos_buf <= {current_y_pos_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
						current_z_pos_buf <= {current_z_pos_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
					end

					if (set_target_pos_in) begin
						target_x_pos_buf <= {target_x_pos_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
						target_y_pos_buf <= {target_y_pos_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
						target_z_pos_buf <= {target_z_pos_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
					end

					if (set_target_time_in) begin
						target_time_buf <= {target_time_in[(POS_WIDTH-1):0], {DEC_WIDTH{1'b0}}};
					end
					
					if (set_target_speed_in) begin
						target_speed_buf <= {{DEC_WIDTH{target_speed_in[POS_WIDTH-1]}}, target_speed_in[(POS_WIDTH-1):0]};
					end
	
					if (set_current_pos_in || set_target_pos_in || set_target_time_in) begin
						current_x_rem_buf <= {DIV_WIDTH{1'b0}};
						current_y_rem_buf <= {DIV_WIDTH{1'b0}};
						current_z_rem_buf <= {DIV_WIDTH{1'b0}};
					end
					
					if (sync_pulse_in) begin
						interp_state <= is_t_comp;
					end
				end
				
				is_t_comp: begin
					if (target_time_buf > current_time_count) begin
						div_num_in <= (target_time_buf - current_time_count);
						div_den_in <= target_speed_buf;
						if (target_speed_buf!={DIV_WIDTH{1'b0}})
							interp_state <= is_t_div_a;
						else
							interp_state <= is_done;
					end
					else begin
						current_x_pos_buf <= target_x_pos_buf;
						current_y_pos_buf <= target_y_pos_buf;
						current_z_pos_buf <= target_z_pos_buf;
						interp_state <= is_final_done;
					end
				end
				is_t_div_a: begin
					interp_state <= is_t_div_b;
				end
				is_t_div_b: if (div_done_node) begin
					recalc_goal_time <= (div_quotient_product + current_time_count);
					interp_state <= is_t_eval;
				end
				is_t_eval: begin
					if ((current_time_count + target_speed_buf) < recalc_goal_time) begin
						current_time_count <= current_time_count + target_speed_buf;
						interp_state <= is_x_comp;
					end
					else begin
						current_x_pos_buf <= target_x_pos_buf;
						current_y_pos_buf <= target_y_pos_buf;
						current_z_pos_buf <= target_z_pos_buf;
						interp_state <= is_final_done;
					end
				end
				
				is_x_comp: begin
					div_den_in <= (recalc_goal_time - current_time_count);

					if (target_x_pos_buf > current_x_pos_buf) begin
						div_num_in <= (target_x_pos_buf - current_x_pos_buf) + current_x_rem_buf;
						negative_diff <= 1'b0;
					end
					else begin
						div_num_in <= (current_x_pos_buf - target_x_pos_buf) + current_x_rem_buf;
						negative_diff <= 1'b1;
					end
					interp_state <= is_x_div_a;
				end
				is_x_div_a: begin
					interp_state <= is_x_div_b;
				end
				is_x_div_b: if (div_done_node) begin
					current_x_pos_buf <= (!negative_diff) ? (current_x_pos_buf + $signed({1'b0,div_quotient_product})) : (current_x_pos_buf - $signed({1'b0,div_quotient_product}));
					current_x_rem_buf <= div_remainder_node;
					interp_state <= is_y_comp;
				end
				
				is_y_comp: begin
					if (target_y_pos_buf > current_y_pos_buf) begin
						div_num_in <= (target_y_pos_buf - current_y_pos_buf) + current_y_rem_buf;
						negative_diff <= 1'b0;
					end
					else begin
						div_num_in <= (current_y_pos_buf - target_y_pos_buf) + current_y_rem_buf;
						negative_diff <= 1'b1;
					end
					interp_state <= is_y_div_a;
				end
				is_y_div_a: begin
					interp_state <= is_y_div_b;
				end
				is_y_div_b: if (div_done_node) begin
					current_y_pos_buf <= (!negative_diff) ? (current_y_pos_buf + $signed({1'b0,div_quotient_product})) : (current_y_pos_buf - $signed({1'b0,div_quotient_product}));
					current_y_rem_buf <= div_remainder_node;
					interp_state <= is_z_comp;
				end
				
				is_z_comp: begin
					if (target_z_pos_buf > current_z_pos_buf) begin
						div_num_in <= (target_z_pos_buf - current_z_pos_buf) + current_z_rem_buf;
						negative_diff <= 1'b0;
					end
					else begin
						div_num_in <= (current_z_pos_buf - target_z_pos_buf) + current_z_rem_buf;
						negative_diff <= 1'b1;
					end
					interp_state <= is_z_div_a;
				end
				is_z_div_a: begin
					interp_state <= is_z_div_b;
				end
				is_z_div_b: if (div_done_node) begin
					if ((current_time_count + target_speed_buf) < recalc_goal_time) begin
						current_z_pos_buf <= (!negative_diff) ? (current_z_pos_buf + $signed({1'b0,div_quotient_product})) : (current_z_pos_buf - $signed({1'b0, div_quotient_product}));
						current_z_rem_buf <= div_remainder_node;
						interp_state <= is_done;
					end
					else begin
						current_x_pos_buf <= target_x_pos_buf;
						current_y_pos_buf <= target_y_pos_buf;
						current_z_pos_buf <= target_z_pos_buf;
						interp_state <= is_final_done;
					end
				end
				
				is_done: begin
					interp_state <= is_idle;
				end
				is_final_done: begin
					current_time_count <= {DIV_WIDTH{1'b0}};
					interp_state <= is_idle;
				end
				default: begin
					interp_state <= is_idle;
				end
			endcase
		end
	end
	
	assign sync_pulse_out = (interp_state == is_done) || (interp_state == is_final_done);
	assign interp_done_out = (interp_state == is_final_done);
	assign div_start_node = (interp_state == is_t_div_a) || (interp_state == is_x_div_a) || (interp_state == is_y_div_a) || (interp_state == is_z_div_a);

	assign new_x_pos_out[0+:POS_WIDTH] = current_x_pos_buf[DEC_WIDTH+:POS_WIDTH];
	assign new_y_pos_out[0+:POS_WIDTH] = current_y_pos_buf[DEC_WIDTH+:POS_WIDTH];
	assign new_z_pos_out[0+:POS_WIDTH] = current_z_pos_buf[DEC_WIDTH+:POS_WIDTH];
	assign current_time_out[0+:POS_WIDTH] = current_time_count[DEC_WIDTH+:POS_WIDTH];
	
	
	// Division Operation
	fcl_divide #(
		.DATA_WIDTH(DIV_WIDTH))
	position_division (
		.clk_in(clk_in),
		._reset_in(_reset_in),
		
		.start_in(div_start_node),
		.numerator_in(div_num_in),
		.denominator_in(div_den_in),
		.done_out(div_done_node),
		.quotient_out(div_quotient_node),
		.remainder_out(div_remainder_node));
		
	assign div_quotient_product[0+:DIV_WIDTH] = {div_quotient_node[0+:POS_WIDTH], {DEC_WIDTH{1'b0}}};	// Multiply the quotient by the decimal value
	
	
endmodule
