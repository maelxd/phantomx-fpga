`timescale 1ns / 1ps

// This module will connect DNET to the DDR MIG
// Allows the peeking and poking of 32bits of data to/ from anywhere in DDR
// This is primarily intended to be a debug path, streaming of data to and from DDR
// should use a much higher speed interface.  
		
		
// There should be much more registering in this module!
		
		
module fcl_dnet_ddr_io
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
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

	output	wire		ddr_cmd_clk,
	output	wire		ddr_cmd_en,
	output	wire [2:0]	ddr_cmd_instr,
	output	wire [5:0]	ddr_cmd_bl,
	output	wire [29:0]	ddr_cmd_byte_addr,
	input	wire		ddr_cmd_empty,
	input	wire		ddr_cmd_full,
	
	output	wire		ddr_wr_clk,
	output	wire		ddr_wr_en,
	output	wire [3:0]	ddr_wr_mask,
	output	wire [31:0]	ddr_wr_data,
	input	wire		ddr_wr_full,
	input	wire		ddr_wr_empty,
	input	wire [6:0]	ddr_wr_count,
	input	wire		ddr_wr_underrun,
	input	wire		ddr_wr_error,
	
	output	wire		ddr_rd_clk,
	output	wire		ddr_rd_en,
	input	wire [31:0]	ddr_rd_data,
	input	wire		ddr_rd_full,
	input	wire		ddr_rd_empty,
	input	wire [6:0]	ddr_rd_count,
	input	wire		ddr_rd_overflow,
	input	wire		ddr_rd_error,
	
	output	wire		ddr_error,
	
	output	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_out,
	input	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_in,
	input	wire  	[(DNET_ADDR_WIDTH-1):0] dnet_addr_in,
	input 	wire 							dnet_read,
	input 	wire 							dnet_write,
	output	wire 							dnet_ack
);

	reg		[(DNET_DATA_WIDTH-1):0]		dnet_data_in_buf; 
	reg		[(DNET_ADDR_WIDTH-1):0]		dnet_addr_in_buf; 
	reg 								dnet_read_buf; 
	reg 								dnet_write_buf; 
	
	
	reg									ddr_cmd_full_buf; 
	reg									ddr_wr_full_buf; 
	reg									ddr_wr_underrun_buf; 
	reg									ddr_wr_error_buf; 
	reg									ddr_rd_full_buf; 
	reg									ddr_rd_empty_buf; 
	reg									ddr_rd_overflow_buf; 
	reg									ddr_rd_error_buf; 

	
	// Buffer the input of DNET at the module level (makes it much easier to meet timing)
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
		
	
	// Buffer the input of the DDR interface (to help with timing)
	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			ddr_cmd_full_buf <= 1'b0;  
			ddr_wr_full_buf <= 1'b0; 
			ddr_wr_underrun_buf <= 1'b0;  
			ddr_wr_error_buf <= 1'b0; 
			ddr_rd_full_buf <= 1'b0; 
			ddr_rd_empty_buf <= 1'b0; 
			ddr_rd_overflow_buf <= 1'b0; 
			ddr_rd_error_buf <= 1'b0; 
		end
		else begin
			ddr_cmd_full_buf <= ddr_cmd_full;
			ddr_wr_full_buf <= ddr_wr_full;
			ddr_wr_underrun_buf <= ddr_wr_underrun;
			ddr_wr_error_buf <= ddr_wr_error;
			ddr_rd_full_buf <= ddr_rd_full;
			ddr_rd_empty_buf <= ddr_rd_empty;
			ddr_rd_overflow_buf <= ddr_rd_overflow; 
			ddr_rd_error_buf <= ddr_rd_error;
		end
	end
	
	
	
	// DDR Command Port
	assign ddr_cmd_clk = sys_clk;
	assign ddr_cmd_instr[2:0] = {2'b00, dnet_read_buf}; //Only Read and Write for now
	assign ddr_cmd_byte_addr[29:0] = {dnet_addr_in_buf[27:0], 2'b00}; 
	assign ddr_cmd_bl[5:0] = 6'h00; // Burst length of one (always)
	assign ddr_cmd_en = (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):(DNET_ADDR_WIDTH-4)] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):(DNET_ADDR_WIDTH-4)]) && (dnet_read_buf || dnet_write_buf) && (!ddr_cmd_full_buf);

	// DDR Write Port
	assign ddr_wr_clk = sys_clk;
	assign ddr_wr_mask[3:0] = 4'b0000; // No masking
	assign ddr_wr_data[31:0] = dnet_data_in_buf[31:0];
	assign ddr_wr_en = (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):(DNET_ADDR_WIDTH-4)] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):(DNET_ADDR_WIDTH-4)]) && (dnet_write_buf) && (!ddr_cmd_full_buf) && (!ddr_wr_full_buf);

	// DDR Read Port
	assign ddr_rd_clk = sys_clk;
	assign ddr_rd_en = (!ddr_rd_empty); // Hmmm
	
	reg [31:0] ddr_data_read_buf; 
	reg ddr_read_buf; 
	
	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			ddr_data_read_buf <= {DNET_DATA_WIDTH{1'b0}};
			ddr_read_buf <= 1'b0;
		end
		else begin
			ddr_read_buf <= ddr_rd_en;
			if (ddr_rd_en) ddr_data_read_buf <= ddr_rd_data;
		end
	end
	
	assign dnet_data_out = (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):(DNET_ADDR_WIDTH-4)] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):(DNET_ADDR_WIDTH-4)]) ? ddr_data_read_buf : {DNET_DATA_WIDTH{1'b0}};
	assign dnet_ack = dnet_write_buf || ddr_read_buf;
	
	assign ddr_error = ddr_cmd_full_buf || ddr_wr_error_buf || ddr_wr_underrun_buf || ddr_wr_full_buf || ddr_rd_error_buf || ddr_rd_overflow_buf || ddr_rd_full_buf;

endmodule
