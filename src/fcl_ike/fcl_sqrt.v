
// A module written to create the square-root fucntion!
// This module registers in the input data when start_in goes high. 
// It then processes until the square root result is finished and
// outputs a done signal.

module fcl_sqrt
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
    parameter integer 	DATA_WIDTH_IN = 32,
    parameter integer 	DATA_WIDTH_OUT = DATA_WIDTH_IN / 2
)
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset_in,
    input 	wire			clk_in,
	
	input 	wire										start_in,
    input 	wire				[(DATA_WIDTH_IN-1):0]	data_in,
    output 	reg											done_out,
    output 	wire				[(DATA_WIDTH_OUT-1):0]	data_out
);

	//------------------------------------------------------
	//--				Local Parameters			 	  --
	//------------------------------------------------------
	localparam	integer DATA_WIDTH = DATA_WIDTH_IN;
	localparam	integer COUNT_WIDTH = clogb2(DATA_WIDTH_OUT);
	
	
	//------------------------------------------------------
	//--			Variables and Signals			 	  --
	//------------------------------------------------------
	genvar	i;
	
	reg					[(COUNT_WIDTH-1):0]		control_count;
	reg											control_count_bit_buf;
	
	reg					[(DATA_WIDTH_IN-1):0]	data_in_buf;
	reg					[(DATA_WIDTH_OUT-1):0]	q_value;
	reg					[DATA_WIDTH_OUT:0]		r_value;
	wire				[DATA_WIDTH_OUT:0]		r_value_node;

	
	//------------------------------------------------------
	//--				Control Counter				 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			control_count <= {COUNT_WIDTH{1'b0}};
		end
		else begin
			if (|control_count)
				control_count <= control_count - 1'b1;
			else if (start_in)
				control_count <= {COUNT_WIDTH{1'b1}};
		end
	end	
	
	//------------------------------------------------------
	//--				Square Root Logic 			 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			data_in_buf <= {DATA_WIDTH_IN{1'b0}};
			q_value <= {DATA_WIDTH_OUT{1'b0}};
			r_value <= {(DATA_WIDTH_OUT+1){1'b0}};
		end
		else begin
			if ((!(|control_count)) && start_in) begin
				data_in_buf <= data_in;
				q_value <= {DATA_WIDTH_OUT{1'b0}};
				r_value <= {(DATA_WIDTH_OUT+1){1'b0}};
			end
			else if ((|control_count) || control_count_bit_buf) begin
				q_value <= {q_value[(DATA_WIDTH_OUT-2):0], (!r_value_node[DATA_WIDTH_OUT])};
				r_value <= r_value_node;
			end
		end
	end
	
	assign r_value_node = r_value[DATA_WIDTH_OUT] ? {r_value[(DATA_WIDTH_OUT-2):0], data_in_buf[{control_count[(COUNT_WIDTH-1):0],1'b0}+:2]} + {q_value[(DATA_WIDTH_OUT-3):0], 2'b11} : 
							{r_value[(DATA_WIDTH_OUT-2):0], data_in_buf[{control_count[(COUNT_WIDTH-1):0],1'b0}+:2]} - {q_value[(DATA_WIDTH_OUT-3):0], 2'b01};

	assign data_out = q_value[(DATA_WIDTH_OUT-1):0];

	
	//------------------------------------------------------
	//--					Done Out Logic	 		 	  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin 
		if (!_reset_in) begin
			control_count_bit_buf <= 1'b0;
			done_out <= 1'b0;
		end
		else begin
			control_count_bit_buf <= control_count[0];
			done_out <= !(|control_count) && control_count_bit_buf;
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
