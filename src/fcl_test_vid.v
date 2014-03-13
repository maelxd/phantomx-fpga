
// This generates some fake video data (before a camera is hooked up)
// It should generate a horizontally scrolling diagonal pattern
// This should next be connected to a DMA port

module fcl_test_vid
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 	X_RES = 320,
	parameter integer 	Y_RES = 240,
	parameter integer 	X_GAP = 20,
	parameter integer 	Y_GAP = 100,
	parameter integer 	PIX_CLK_FACTOR = 1
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset,
    input 	wire			sys_clk,

	// Tx I/O	
	output 	wire 	[7:0]	v_data_out,	
	output 	wire 			v_fvalid_out,	
	output 	wire 			v_lvalid_out,	
	output 	wire 			v_pvalid_out
);

	localparam X_TOTAL = X_RES + X_GAP;
	localparam X_TOTAL_W = clogb2(X_TOTAL+1);
	localparam Y_TOTAL = Y_RES + Y_GAP;
	localparam Y_TOTAL_W = clogb2(Y_TOTAL+1);
	
	reg [15:0] f_count;
	reg [(X_TOTAL_W-1):0] x_count;
	reg [(Y_TOTAL_W-1):0] y_count;

	reg [7:0] v_data_reg;
	wire p_enable;
	reg p_enable_buf;
	
	reg v_fvalid_reg;
	reg v_lvalid_reg;
	reg v_pvalid_reg;
	reg v_latch;

	assign v_fvalid_out = v_fvalid_reg;
	assign v_lvalid_out = v_lvalid_reg;
	assign v_pvalid_out = v_pvalid_reg;
	assign v_data_out[7:0] = v_pvalid_out ? v_data_reg[7:0] : 8'h00;
	
	generate
	if (PIX_CLK_FACTOR == 0) begin
		assign p_enable = 1'b1;
	end
	else begin
		reg [(PIX_CLK_FACTOR-1):0] p_count;
	
		always @(posedge sys_clk or negedge _reset) begin
			if (!_reset) begin
				p_count <= {PIX_CLK_FACTOR{1'b0}};
			end
			else begin
				p_count <= p_count + 1'b1;	
			end
		end
		
		assign p_enable = &p_count; 
	end
	endgenerate
	
	
	always @(posedge sys_clk or negedge _reset) begin
		if (!_reset) begin
			x_count <= {X_TOTAL_W{1'b1}};
			y_count <= {Y_TOTAL_W{1'b1}};
			f_count <= {16{1'b0}};
			p_enable_buf <= 1'b0;
			v_fvalid_reg <= 1'b0;
			v_lvalid_reg <= 1'b0;
			v_pvalid_reg <= 1'b0;
			v_latch <= 1'b0;
		end
		else begin
			p_enable_buf <= p_enable;
			if (p_enable) begin
				if (x_count<X_TOTAL)
					x_count <= x_count + 1'b1;	
				else begin
					x_count <= {X_TOTAL_W{1'b0}};
					if (y_count<Y_TOTAL)
						y_count <= y_count + 1'b1;	
					else begin
						y_count <= {Y_TOTAL_W{1'b0}};
						f_count <= f_count + 1'b1;	
					end
				end
			end
			
			v_fvalid_reg <= (y_count<Y_RES);
			v_lvalid_reg <= v_fvalid_out && (x_count<X_RES);
			v_pvalid_reg <= v_lvalid_out && p_enable_buf;
			if ((y_count<Y_RES) && (!v_fvalid_reg)) begin
				v_latch <= 1'b1;
			end
			
			if (v_latch) begin
				v_data_reg <= 8'hF4;
				if (v_pvalid_out)
				v_latch <= 1'b0;
			end
			else v_data_reg <= x_count + f_count - 1'b1; // + y_count 
		end
	end

	
	function integer clogb2;input [31:0] value; 
		begin
			value = value - 1'b1;
			for (clogb2=0; value>0; clogb2=clogb2+1)  value = value>>1;
		end
	endfunction

endmodule
