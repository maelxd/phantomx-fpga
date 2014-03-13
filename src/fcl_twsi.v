
// This is a new TWSI interface controller (I swear I've written these at least 4-5 times now...)
// It is specifically designed with the Micron Camera control interface in mind (a subset of I2C)

module fcl_twsi
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
    parameter integer 	INPUT_CLOCK_SPEED = 125000000,
	parameter integer 	TWSI_CLOCK_SPEED = 400000,
	parameter integer 	DNET_ADDR_WIDTH = 16,
	parameter integer 	DNET_DATA_WIDTH = 32,
	parameter integer 	DNET_OFFSET = 0,
	parameter [7:0] 	TWSI_DEVICE_ADDR = 8'hBA
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	// System Signals
    input 	wire			_reset,
    input 	wire			sys_clk,
	
	// TWSI Signals
    inout 	wire			twsi_sda,
    input 	wire			twsi_sda_override,
    output 	reg				twsi_scl,
    output 	wire			twsi_sdata_out,
	
	// Debug Signals
    output 	reg				twsi_done_out,
    output 	reg				twsi_error_out,
	output  reg 	[2:0]	twsi_control, // Output to the state machine for now
	
	// DNET Signals
	output	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_out,
	input	wire  	[(DNET_DATA_WIDTH-1):0] dnet_data_in,
	input	wire  	[(DNET_ADDR_WIDTH-1):0] dnet_addr_in,
	input 	wire 							dnet_read,
	input 	wire 							dnet_write,
	output	wire 							dnet_ack
);


	localparam WHOLE_BIT_COUNT = (INPUT_CLOCK_SPEED / TWSI_CLOCK_SPEED)-1;
	localparam HALF_BIT_COUNT = (INPUT_CLOCK_SPEED / (TWSI_CLOCK_SPEED*2))-1;
	localparam Q1_BIT_COUNT = (INPUT_CLOCK_SPEED / (TWSI_CLOCK_SPEED*4))-1;
	localparam Q3_BIT_COUNT = HALF_BIT_COUNT + Q1_BIT_COUNT + 1;
	localparam COUNTER_WIDTH = clogb2(WHOLE_BIT_COUNT+1);

	localparam [2:0] stwsi_idle 	= 3'h0;
	localparam [2:0] stwsi_startbit = 3'h1;
	localparam [2:0] stwsi_writebit = 3'h2;
	localparam [2:0] stwsi_ack 		= 3'h3;
	localparam [2:0] stwsi_stopbit 	= 3'h4;
	localparam [2:0] stwsi_readbit 	= 3'h5;
	localparam [2:0] stwsi_mack 	= 3'h6;



	wire 	twsi_sda_din;
	reg 	twsi_sda_dout;
		
	reg 	[(COUNTER_WIDTH-1):0] 	twsi_clock_div;
	wire 	twsi_wholebit_en;
	wire 	twsi_halfbit_en;
	wire 	twsi_q1bit_en;
	wire 	twsi_q3bit_en;
	

	reg [7:0] twsi_reg_addr;
	reg [15:0] twsi_reg_data_in;
	reg [15:0] twsi_reg_data_out;
	reg twsi_mode_rnw;
	reg twsi_mode_rbit;
	reg [7:0] twsi_data_byte;
	reg [2:0] data_bit_count;
	reg [1:0] data_byte_count;
	reg twsi_sda_ack;

	reg [(DNET_DATA_WIDTH-1):0] dnet_data_in_buf; 
	reg [(DNET_ADDR_WIDTH-1):0] dnet_addr_in_buf; 
	reg  dnet_read_buf; 
	reg  dnet_write_buf; 

	
	//------------------------------------------------------
	//--			Bidirectional Pin Logic				  --
	//------------------------------------------------------
	assign twsi_sda = twsi_sda_dout ? 1'bz : 1'b0;
	assign twsi_sda_din = twsi_sda_override ? 1'b0 : twsi_sda;
	
	assign twsi_sdata_out = twsi_sda_din;
	
	
	//------------------------------------------------------
	//--			Clock Division Logic			 	  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
            twsi_clock_div <= {COUNTER_WIDTH{1'b0}};
        end 
		else begin
			if (twsi_wholebit_en) 
				twsi_clock_div <= {COUNTER_WIDTH{1'b0}};
			else
				twsi_clock_div <= twsi_clock_div + 1'b1;
        end
	end
	
	assign twsi_wholebit_en = (twsi_clock_div==WHOLE_BIT_COUNT);
	assign twsi_halfbit_en = (twsi_clock_div==HALF_BIT_COUNT);
	assign twsi_q1bit_en = (twsi_clock_div==Q1_BIT_COUNT);
	assign twsi_q3bit_en = (twsi_clock_div==Q3_BIT_COUNT);
	
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
            twsi_scl <= 1'b0;
        end 
		else begin
			if (twsi_q1bit_en) 
				twsi_scl <= 1'b1;
			else if (twsi_q3bit_en)
				twsi_scl <= 1'b0;
        end
	end
	

	//------------------------------------------------------
	//--				DNET Interface Logic		 	  --
	//------------------------------------------------------
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

	always @(posedge sys_clk or negedge _reset) begin 
		if (!_reset) begin
			twsi_reg_addr <= {8{1'b0}};
			twsi_reg_data_in <= {16{1'b0}};
		end
		else begin
			if (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):8] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):8]) begin
				if (dnet_write_buf||dnet_read_buf) begin
					twsi_reg_addr[7:0] <= dnet_addr_in_buf[7:0];
					twsi_reg_data_in[15:0] <= dnet_data_in_buf[15:0];
				end
			end
		end
	end
	assign dnet_ack = (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):8] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):8]) && twsi_sda_ack;
	assign dnet_data_out = (dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):8] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):8]) ? {{(DNET_DATA_WIDTH-16){1'b0}},twsi_reg_data_out[15:0]} : {DNET_DATA_WIDTH{1'b0}};
	
	
	
	//------------------------------------------------------
	//--			TWSI State Machine Logic		 	  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
            twsi_control <= stwsi_idle;
			twsi_sda_ack <= 1'b0;
			twsi_sda_dout <= 1'b1;
			twsi_mode_rnw <= 1'b0;
			twsi_mode_rbit <= 1'b0;
			twsi_data_byte <= 8'h00;
			data_bit_count <= 3'h7;
			data_byte_count <= 2'h0;
			twsi_reg_data_out <= 16'h0000;
			twsi_done_out <= 1'b0;
			twsi_error_out <= 1'b0;
        end 
		else begin
			case(twsi_control)
				stwsi_idle: begin
					twsi_done_out <= 1'b0;
					twsi_error_out <= 1'b0;
					twsi_sda_ack <= 1'b0;
					twsi_sda_dout <= 1'b1;
					twsi_mode_rbit <= 1'b0;
					if ((dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):8] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):8]) && dnet_write_buf) begin
						twsi_mode_rnw <= 1'b0;
						twsi_control <= stwsi_startbit;
					end 
					else if ((dnet_addr_in_buf[(DNET_ADDR_WIDTH-1):8] == DNET_OFFSET[(DNET_ADDR_WIDTH-1):8]) && dnet_read_buf) begin
						twsi_mode_rnw <= 1'b1;
						twsi_control <= stwsi_startbit;
					end
				end
				stwsi_startbit: begin	// Start and sync to the I2C timing path
					if (twsi_halfbit_en) begin
						twsi_sda_dout <= 1'b0;
						data_bit_count <= 3'h7;
						data_byte_count <= 2'h0;
						twsi_reg_data_out <= 16'h0000;
						twsi_data_byte[7:0] <= {TWSI_DEVICE_ADDR[7:1],twsi_mode_rbit};
						twsi_control <= stwsi_writebit;
					end
				end
				stwsi_writebit: begin
					if (twsi_wholebit_en) begin
						twsi_sda_dout <= twsi_data_byte[data_bit_count];
						data_bit_count <= data_bit_count - 1'b1;
						if (data_bit_count==3'h0) twsi_control <= stwsi_ack;
					end
				end
				stwsi_ack: begin
					if (twsi_wholebit_en) begin
						data_bit_count <= 3'h6;
						twsi_sda_dout <= 1'b1;
					end
					
					if (twsi_halfbit_en && (data_bit_count==3'h6)) begin
						if (!twsi_sda_din) begin // ACK Success
							data_bit_count <= 3'h7;
							data_byte_count <= data_byte_count + 1'b1;
							if (data_byte_count==2'h0) begin
								twsi_data_byte[7:0] <= twsi_reg_addr[7:0]; 
							end 
							else if (data_byte_count==2'h1) begin
								twsi_data_byte[7:0] <= twsi_reg_data_in[15:8];
							end
							else if (data_byte_count==2'h2) begin
								twsi_data_byte[7:0] <= twsi_reg_data_in[7:0];
							end
							
							if (twsi_mode_rbit) begin
								twsi_control <= stwsi_readbit;
							end 
							else if (twsi_mode_rnw && (data_byte_count==2'h1)) begin
								twsi_mode_rbit <= 1'b1;
								twsi_control <= stwsi_startbit;
							end
							else if (data_byte_count==2'h3) begin
								twsi_control <= stwsi_stopbit;
							end
							else begin
								twsi_control <= stwsi_writebit;
							end
						end
						else begin	// Error (this will trigger a bus error!)
							twsi_error_out <= 1'b1;
							twsi_control <= stwsi_idle;
						end	
					end
				end
				stwsi_stopbit: begin
					if (twsi_wholebit_en) begin
						twsi_sda_dout <= 1'b0;
					end
					if (twsi_halfbit_en) begin
						twsi_sda_dout <= 1'b1;
						twsi_sda_ack <= 1'b1;
						twsi_done_out <= 1'b1;
						twsi_control <= stwsi_idle;
					end
				end
				stwsi_readbit: begin
					if (twsi_halfbit_en) begin
						twsi_data_byte[data_bit_count] <= twsi_sda_din;
						data_bit_count <= data_bit_count - 1'b1;
						if (data_bit_count==3'h0) begin

							twsi_control <= stwsi_mack;
						end
					end
				end
				stwsi_mack: begin
					if (twsi_wholebit_en) begin
						data_bit_count <= 3'h6;
						if (data_byte_count==2'h1) begin
							twsi_sda_dout <= 1'b0;	//ACK
						end
						else begin
							twsi_sda_dout <= 1'b1;	// NACK
						end
					end

					if (twsi_wholebit_en&&(data_bit_count==3'h6)) begin
						data_byte_count <= data_byte_count + 1'b1;
						data_bit_count <= 3'h7;
						if (data_byte_count==2'h1) begin
							twsi_sda_dout <= 1'b1;
							twsi_reg_data_out[15:8] <= twsi_data_byte[7:0];
							twsi_control <= stwsi_readbit;
						end
						else begin
							twsi_sda_dout <= 1'b0;
							twsi_reg_data_out[7:0] <= twsi_data_byte[7:0];
							twsi_control <= stwsi_stopbit;
						end
					end 
				end

				default: twsi_control <= stwsi_idle;
			endcase
        end
	end
	

	
	function integer clogb2;input [31:0] value; 
		begin
			value = value - 1'b1;
			for (clogb2=0; value>0; clogb2=clogb2+1)  value = value>>1;
		end
	endfunction

endmodule
