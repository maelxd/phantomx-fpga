
// A module written to create the division function!
// This module registers in the input data when start_in goes high. 
// It then processes until the division result is finished and
// outputs a done signal.
// Requires positive inputs
		
module fcl_divide
	//------------------------------------------------------
	//--				External Parameters				  --
	//------------------------------------------------------
#(
    parameter integer 	DATA_WIDTH = 32
)
	//------------------------------------------------------
	//--					Ports						  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset_in,
    input 	wire			clk_in,
	
	input 	wire									start_in,
    input 	wire				[(DATA_WIDTH-1):0]	numerator_in,
    input 	wire				[(DATA_WIDTH-1):0]	denominator_in,
    output 	reg										done_out,
    output 	wire				[(DATA_WIDTH-1):0]	quotient_out,
    output 	wire				[(DATA_WIDTH-1):0]	remainder_out
);

	//------------------------------------------------------
	//--				Local Parameters				  --
	//------------------------------------------------------
	localparam	integer COUNT_WIDTH = clogb2(DATA_WIDTH);
	
	
	//------------------------------------------------------
	//--			Variables and Signals				  --
	//------------------------------------------------------
	genvar	i;
	
	reg					[(COUNT_WIDTH-1):0]		control_count;
	wire										control_count_limit;
	reg											control_count_bit_buf;
	
	reg					[(DATA_WIDTH-1):0]	denominator_in_buf;
	
	reg					[(DATA_WIDTH-1):0]	q_value;
	reg					[DATA_WIDTH:0]		r_value;
	wire				[DATA_WIDTH:0]		r_value_node;

	
	//------------------------------------------------------
	//--				Control Counter					  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			control_count <= (DATA_WIDTH[(COUNT_WIDTH-1):0] - 1'b1);
		end
		else begin
			if (!control_count_limit)
				control_count <= control_count + 1'b1;
			else if (start_in)
				control_count <= {COUNT_WIDTH{1'b0}};
		end
	end
	assign control_count_limit = (control_count == (DATA_WIDTH[(COUNT_WIDTH-1):0] - 1'b1));
	
	//------------------------------------------------------
	//--				Square Root Logic 				  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			denominator_in_buf <= {DATA_WIDTH{1'b0}};
			q_value <= {DATA_WIDTH{1'b0}};
			r_value <= {(DATA_WIDTH+1){1'b0}};
		end
		else begin
			if (control_count_limit && start_in) begin
				denominator_in_buf <= denominator_in;
				q_value <= numerator_in;
				r_value <= {(DATA_WIDTH+1){1'b0}};
			end
			else if ((!control_count_limit) || (!control_count_bit_buf)) begin
				q_value <= {q_value[(DATA_WIDTH-2):0], !r_value_node[DATA_WIDTH]};
				r_value <= r_value_node[DATA_WIDTH] ? {r_value[(DATA_WIDTH-2):0], q_value[DATA_WIDTH-1]} : r_value_node;
			end
		end
	end
	
	assign r_value_node[DATA_WIDTH:0] = {r_value[(DATA_WIDTH-2):0], q_value[DATA_WIDTH-1]} - denominator_in_buf[(DATA_WIDTH-1):0];
	
	assign remainder_out = r_value[(DATA_WIDTH-1):0];
	assign quotient_out = q_value[(DATA_WIDTH-1):0];

	
	//------------------------------------------------------
	//--					Done Out Logic	 			  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			control_count_bit_buf <= 1'b1;
			done_out <= 1'b0;
		end
		else begin
			control_count_bit_buf <= control_count_limit;
			done_out <= control_count_limit && (!control_count_bit_buf);
		end
	end

	
	// Ceil Log 2 Function
	function integer clogb2;input [31:0] value; 
		begin
			value = value - 1'b1;
			for (clogb2=0; value>0; clogb2=clogb2+1)  value = value>>1;
		end
	endfunction
	
endmodule
