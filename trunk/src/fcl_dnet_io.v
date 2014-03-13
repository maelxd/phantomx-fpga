`timescale 1ns / 1ps
		
module fcl_dnet_io
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
    parameter 		 	TYPE = "REG",	// REG, STATUS, STROBE
	parameter integer 	WIDTH = 32,		// Max of DNET_DATA_WIDTH
	parameter integer 	DNET_ADDR_WIDTH = 16,
	parameter integer 	DNET_DATA_WIDTH = 32,
	parameter integer 	DNET_OFFSET = 0
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
    input 	wire	_reset,
    input 	wire	sys_clk,
	
	input	wire 	[(WIDTH-1):0]	data_in,
	output	wire	[(WIDTH-1):0]	data_out,
	
	output	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_out,
	input	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_in,
	input	wire  	[(DNET_ADDR_WIDTH-1):0] dnet_addr_in,
	input 	wire 							dnet_read,
	input 	wire 							dnet_write,
	output	wire 							dnet_ack
);

	reg [(DNET_DATA_WIDTH-1):0] dnet_data_in_buf; 
	reg [(DNET_ADDR_WIDTH-1):0] dnet_addr_in_buf; 
	reg  dnet_read_buf; 
	reg  dnet_write_buf; 

	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			dnet_data_in_buf <= {DNET_DATA_WIDTH{1'b0}};
			dnet_addr_in_buf <= {DNET_ADDR_WIDTH{1'b0}};
			dnet_read_buf <= 1'b0;
			dnet_write_buf <= 1'b0;
		end
		else begin
			dnet_data_in_buf <= dnet_data_in;
			dnet_addr_in_buf <= dnet_addr_in;
			dnet_read_buf <= dnet_read;
			dnet_write_buf <= dnet_write;
		end
	end
	
	generate
	if (TYPE=="REG") begin
		reg [(WIDTH-1):0] data_reg;
		
		always @(posedge sys_clk or negedge _reset) begin 
			if (!_reset) begin
				data_reg <= {WIDTH{1'b0}};
			end
			else begin
				if (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) begin
					if (dnet_write_buf)	data_reg[(WIDTH-1):0] <= dnet_data_in_buf[(WIDTH-1):0];
				end
			end
		end
		assign dnet_ack = (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) && (dnet_write_buf || dnet_read_buf);
		assign dnet_data_out = (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) ? {{(DNET_DATA_WIDTH-WIDTH){1'b0}},data_reg[(WIDTH-1):0]} : {DNET_DATA_WIDTH{1'b0}};
		assign data_out = data_reg;
	end
	else if (TYPE=="STATUS") begin
		assign dnet_ack = (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) && dnet_read_buf;
		assign dnet_data_out = (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) ? {{(DNET_DATA_WIDTH-WIDTH){1'b0}},data_in[(WIDTH-1):0]} : {DNET_DATA_WIDTH{1'b0}};
		assign data_out = data_in;
	end
	else begin
		reg [(WIDTH-1):0] data_reg;
		
		always @(posedge sys_clk or negedge _reset) begin 
			if (!_reset) begin
				data_reg <= {WIDTH{1'b0}};
			end
			else begin
				if (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) begin
					if (dnet_write_buf)	data_reg[(WIDTH-1):0] <= dnet_data_in_buf[(WIDTH-1):0];
					else  data_reg <= {WIDTH{1'b0}};
				end
				else 
					data_reg <= {WIDTH{1'b0}};
			end
		end
		assign dnet_ack = (dnet_addr_in_buf == DNET_OFFSET[(DNET_ADDR_WIDTH-1):0]) && dnet_write_buf;
		assign dnet_data_out = {DNET_DATA_WIDTH{1'b0}};
		assign data_out = data_reg;
	end
	endgenerate

endmodule
