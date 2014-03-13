`timescale 1ns / 1ps

// This module captures frames from the video stream to DDR3
		
module fcl_vid_ddr
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
	parameter integer 	DDR3_OFFSET = 30'h00000000,
	parameter integer 	FIFO_DEPTH = 512
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
	output	reg  [5:0]	ddr_cmd_bl,
	output	wire [29:0]	ddr_cmd_byte_addr,
	input	wire		ddr_cmd_empty,
	input	wire		ddr_cmd_full,
	
	output	wire		ddr_wr_clk,
	output	reg			ddr_wr_en,
	output	wire [3:0]	ddr_wr_mask,
	output	wire [31:0]	ddr_wr_data,
	input	wire		ddr_wr_full,
	input	wire		ddr_wr_empty,
	input	wire [6:0]	ddr_wr_count,
	input	wire		ddr_wr_underrun,
	input	wire		ddr_wr_error,
	
	output	wire		ddr_error,
	
	input wire 			frame_catch_in,

	// Video In	
	input 	wire 	[7:0]	v_data_in,	
	input 	wire 			v_fvalid_in,
	input 	wire 			v_pvalid_in
);

	localparam integer FIFO_DEPTH_W = clogb2(FIFO_DEPTH);
	localparam [3:0] 	vss_idle		= 4'b0001,
						vss_ddr_write1	= 4'b0010,
						vss_ddr_write2	= 4'b0100,
						vss_ddr_write3	= 4'b1000;

	reg [3:0] v_store_state;

	reg v_fvalid_buf;
	wire v_fvalid_fe;
	reg [3:0] v_fvalid_fe_buf;
	reg frame_catch_buf;
	reg capture_frame;

	reg [31:0] v_data_buf;
	reg [1:0] v_data_buf_count;
	reg v_data_buf_valid;
	
	reg [(FIFO_DEPTH_W-1):0] v_wr_addr;
	wire [(FIFO_DEPTH_W-1):0] v_addr_dif;
	
	reg [19:0] ddr_address;
	reg [(FIFO_DEPTH_W-1):0] v_rd_addr;
	reg last_flag;
	
	wire [31:0] v_data_ddr_buf;
	wire v_store_overflow;
	
	
	//------------------------------------------------------
	//--				 Frame Detection 			 	  --
	//------------------------------------------------------
	// This block detects when the module is meant to be capturing a frame
	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			v_fvalid_buf <= 1'b0;
			frame_catch_buf <= 1'b0;
			capture_frame <= 1'b0;
			v_fvalid_fe_buf <= 4'b0000;
		end
		else begin
			v_fvalid_buf <= v_fvalid_in;
			v_fvalid_fe_buf[3:0] <= {v_fvalid_fe_buf[2:0],v_fvalid_fe};
		
			if (frame_catch_in)
				frame_catch_buf <= 1'b1;
			if (v_fvalid_fe_buf[3]) begin
				if (frame_catch_buf) capture_frame <= 1'b1;
				else capture_frame <= 1'b0;
				frame_catch_buf <= 1'b0;
			end
		end
	end
	
	assign v_fvalid_fe = ((v_fvalid_buf && (!v_fvalid_in)));
	
	
	//------------------------------------------------------
	//--				 Pixel Packing	 			 	  --
	//------------------------------------------------------
	// This block will pack pixels to the word size of the RAM interface	
	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			v_data_buf <= {32{1'b0}};
			v_data_buf_valid <= 1'b0;
			v_data_buf_count <= 2'b00;
		end
		else begin
			if (v_fvalid_fe) begin
				v_data_buf <= {32{1'b0}};
				v_data_buf_count <= 2'b00;
			end
			else if (v_pvalid_in) begin
				case(v_data_buf_count)
					2'b00 : v_data_buf[31:0] <= {v_data_in[7:0],{24{1'b0}}};
					2'b01 : v_data_buf[23:16] <= v_data_in[7:0];
					2'b10 : v_data_buf[15:8] <= v_data_in[7:0];
					2'b11 : v_data_buf[7:0] <= v_data_in[7:0];
				endcase
				v_data_buf_count <= v_data_buf_count + 1'b1;
			end
			v_data_buf_valid <=  capture_frame && ((v_pvalid_in && (v_data_buf_count == 2'b11)) || v_fvalid_fe);
		end
	end
		

	//------------------------------------------------------
	//--			 Write Port Addressing 			 	  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			v_wr_addr[(FIFO_DEPTH_W-1):0] <= {{(FIFO_DEPTH_W-1){1'b0}},1'b1};
		end
		else begin
			if (v_store_overflow||(v_store_state==vss_idle)) begin
				v_wr_addr[(FIFO_DEPTH_W-1):0] <= {{(FIFO_DEPTH_W-1){1'b0}},1'b1};
			end
			else if (v_data_buf_valid) begin
				v_wr_addr <= v_wr_addr + 1'b1;
			end
		end
	end
	
	assign v_addr_dif = (v_wr_addr - v_rd_addr); // Address difference (for FIFO like operations)
	

	//------------------------------------------------------
	//--			 Video Store RAM Buffer			 	  --
	//------------------------------------------------------
	fcl_dp_ram #(
		//.RAM_DATA_A_W(32), // To match DDR3 Controller width
		//.RAM_DATA_B_W(32),
		.RAM_DEPTH(FIFO_DEPTH), // Units of 512 (to match Spartan6 Block RAM sizes)
		//.RAM_ADDR_A_W(9),
		//.RAM_ADDR_B_W(9),
		.RAM_TYPE("BLOCK")) // Use Block RAM!
	frame_buffer (
		._reset(_reset),
		
		.wr_clk(sys_clk),
		.wr_en(v_data_buf_valid), 
		.wr_addr_in(v_wr_addr),
		.wr_data_in(v_data_buf),
		
		.rd_clk(sys_clk),
		.rd_addr_in(v_rd_addr),
		.rd_data_out(v_data_ddr_buf));
		
			
	//------------------------------------------------------
	//--		 Video Store State Machine Logic	 	  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
            v_store_state <= vss_idle;
			ddr_wr_en <= 1'b0;
			ddr_address <= {20{1'b0}};
			ddr_cmd_bl <= {6{1'b0}};
			v_rd_addr[(FIFO_DEPTH_W-1):0] <= {FIFO_DEPTH_W{1'b0}};
			last_flag <= 1'b0;
        end 
		else begin
			if (v_store_overflow) begin
				v_store_state <= vss_idle;
			end
			else begin
				case(v_store_state)
					vss_idle: begin 
						ddr_wr_en <= 1'b0;
						ddr_address <= {20{1'b0}};
						ddr_cmd_bl <= {6{1'b0}};
						last_flag <= 1'b0;
						v_rd_addr[(FIFO_DEPTH_W-1):0] <= {FIFO_DEPTH_W{1'b0}};
						if (capture_frame)	v_store_state <= vss_ddr_write1;
					end
					vss_ddr_write1: begin 
						if (v_fvalid_fe_buf[3]) last_flag <= 1'b1;
						ddr_cmd_bl <= {6{1'b0}};
						if (last_flag) begin
							if ((v_addr_dif == 1)||(v_addr_dif == 2))
								v_store_state <= vss_idle;
							else if (ddr_wr_empty) begin
								v_rd_addr <= v_rd_addr + 1'b1;
								v_store_state <= vss_ddr_write2;
							end
						end
						else if ((v_addr_dif >= 64)&&ddr_wr_empty)  begin
							v_rd_addr <= v_rd_addr + 1'b1;
							v_store_state <= vss_ddr_write2;
						end
					end
					vss_ddr_write2: begin
						if (v_fvalid_fe_buf[3]) last_flag <= 1'b1;
						ddr_wr_en <= 1'b1;
						if (ddr_wr_full||(ddr_cmd_bl==6'b111111)) v_store_state <= vss_ddr_write3;
						else if (last_flag&&(v_addr_dif == 1)) begin
							ddr_wr_en <= 1'b0;
							v_store_state <= vss_ddr_write3;
						end
						else begin
							ddr_cmd_bl <= ddr_cmd_bl + 1'b1;
							v_rd_addr <= v_rd_addr + 1'b1;
						end
					end
					vss_ddr_write3: begin 
						if (v_fvalid_fe_buf[3]) last_flag <= 1'b1;
						ddr_wr_en <= 1'b0;
						ddr_address <= ddr_address + ddr_cmd_bl + 1'b1;
						v_store_state <= vss_ddr_write1;
					end
					default : v_store_state <= vss_idle;
				endcase
			end
		end
	end
	
	// DDR Command Port
	assign ddr_cmd_clk = sys_clk;
	assign ddr_cmd_instr[2:0] = 3'b000; //Only Write
	assign ddr_cmd_byte_addr[29:0] = DDR3_OFFSET + {ddr_address[19:0], 2'b00};
	assign ddr_cmd_en = (v_store_state == vss_ddr_write3);

	// DDR Write Port
	assign ddr_wr_clk = sys_clk;
	assign ddr_wr_mask[3:0] = 4'b0000; // No masking
	assign ddr_wr_data[31:0] = v_data_ddr_buf[31:0];
	
	// Errors
	assign v_store_overflow = (!(|v_addr_dif));
	assign ddr_error = ddr_cmd_full || ddr_wr_error || ddr_wr_underrun || ddr_wr_full || v_store_overflow;

	function integer clogb2;input [31:0] value; 
		begin
			value = value - 1'b1;
			for (clogb2=0; value>0; clogb2=clogb2+1)  value = value>>1;
		end
	endfunction
	
endmodule

