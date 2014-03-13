`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// A simple rs232 decoder
//
//////////////////////////////////////////////////////////////////////////////////

module fcl_uart_rs232
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
    parameter integer INPUT_CLOCK_SPEED = 50000000,
	parameter integer BAUD_RATE = 115200,
	parameter integer DATA_BITS = 8,
	parameter integer STOP_BITS = 1
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
    input 	wire 			rs232_rx,
    output  wire			rs232_tx,
    output 	wire	[7:0] 	rx_data,
	output 	wire			rx_data_valid,
    input 	wire	[7:0] 	tx_data,
	input 	wire			tx_data_send,
	output 	wire			tx_done,
	output 	wire			busy,
    input 	wire			_reset,
    input 	wire			sys_clk
);
	
	
	//------------------------------------------------------
	//--				Local Parameters			 	  --
	//------------------------------------------------------
	localparam WHOLE_BIT_COUNT = INPUT_CLOCK_SPEED / BAUD_RATE;
	localparam HALF_BIT_COUNT = WHOLE_BIT_COUNT / 2;
	localparam COUNTER_WIDTH = clogb2(WHOLE_BIT_COUNT*(DATA_BITS+STOP_BITS+1));
	
	
	//------------------------------------------------------
	//--				Rx Path Variables			 	  --
	//------------------------------------------------------
	reg [(DATA_BITS-1):0] rx_data_reg;
	reg [(DATA_BITS-1):0] rx_data_buf;
	reg [1:0] rs232_rx_buf = 2'b11;
	wire rs232_rx_fe;
	reg rd_data_valid_buf;
	reg [(COUNTER_WIDTH-1):0] rx_counter;
	
	reg [1:0] rxsm_control;
	localparam  rxsm_idle = 0;
	localparam  rxsm_read = 1;
	localparam  rxsm_done = 2;
	
	
	//------------------------------------------------------
	//--				  Rx Path Logic 		 		  --
	//------------------------------------------------------
	//Falling edge on rs232 rx inticates the beginning of a packet
	always @(posedge sys_clk or negedge _reset) begin
		if (!_reset) begin
			rs232_rx_buf[1:0] <= 2'b11;
		end else begin
			rs232_rx_buf[0] <= rs232_rx;
			rs232_rx_buf[1] <= rs232_rx_buf[0];
		end
	end
	
	assign rs232_rx_fe = rs232_rx_buf[1] && !rs232_rx_buf[0];

    //State Machine
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
			  rx_data_buf[7:0] <= 8'h00;
			  rd_data_valid_buf <= 1'b0;
			  rx_counter <= WHOLE_BIT_COUNT*(DATA_BITS+2);
        end else begin
				case(rxsm_control)
					rxsm_idle: begin
						if (rs232_rx_fe) begin
							rxsm_control <= rxsm_read;
							rx_counter <= WHOLE_BIT_COUNT*(DATA_BITS+1) + HALF_BIT_COUNT;
						end
						rd_data_valid_buf <= 1'b0;
					end
					rxsm_read: begin
						if (rx_counter != 0)  rx_counter <= rx_counter - 1'b1;
						else rxsm_control <= rxsm_done;
					end
					rxsm_done: begin
						rx_data_buf[7:0] <= rx_data_reg[7:0];
						rd_data_valid_buf <= 1'b1;
						rxsm_control <= rxsm_idle;
					end
					default: rxsm_control <= rxsm_idle;
				endcase
        end
	end

   genvar i;
   generate
      for (i=0; i < 8; i = i+1)  begin: rx_read_enables
			always @(posedge sys_clk or negedge _reset) begin
				if (!_reset) begin
					rx_data_reg[i] <= 1'b0;
				end else begin
					if (rx_counter==((WHOLE_BIT_COUNT*(DATA_BITS+1) + HALF_BIT_COUNT) - (WHOLE_BIT_COUNT*(i+1)) - HALF_BIT_COUNT))  rx_data_reg[i] <= rs232_rx;
				end
			end
      end
   endgenerate
		
	assign rx_data[7:0] = rx_data_buf[7:0];
	assign rx_data_valid = rd_data_valid_buf;

	
	//------------------------------------------------------
	//--				Tx Path Variables			 	  --
	//------------------------------------------------------
	reg [9:0] tx_data_packet;
	reg [(COUNTER_WIDTH-1):0] tx_counter;
	reg tx_busy;
	wire [9:0] tx_shift_condition;
	
	wire tx_data_send_re;
	reg tx_data_send_buf;
	reg tx_done_reg;
	
	
	//------------------------------------------------------
	//--				  Tx Path Logic 		 		  --
	//------------------------------------------------------
	always @(posedge sys_clk or negedge _reset) begin
		if (!_reset) begin
			tx_data_send_buf <= 1'b0;
		end else begin
			tx_data_send_buf <= tx_data_send;
		end
	end
	
	assign tx_data_send_re = tx_data_send && !tx_data_send_buf;
	
	
	always @(posedge sys_clk or negedge _reset) begin
		if (!_reset) begin
			tx_data_packet <= 10'b0000000000;
			tx_counter <= ((DATA_BITS+STOP_BITS+1)*WHOLE_BIT_COUNT);
			tx_busy <= 1'b0;
			tx_done_reg <= 1'b0;
		end else begin
			if (tx_busy) begin
				if (tx_counter != 0)  tx_counter <= tx_counter - 1'b1;
				else begin
					tx_done_reg <= 1'b1;
					tx_busy <= 1'b0;
				end
				if (tx_shift_condition[9])  begin
					tx_data_packet[8:0] <= tx_data_packet[9:1];
					tx_data_packet[9] <= 1'b1;
				end
			end
			else begin
				tx_done_reg <= 1'b0;
				if (tx_data_send_re) begin
					tx_data_packet[0] <= 1'b0;
					tx_data_packet[8:1] <= tx_data[7:0];
					tx_data_packet[9] <= 1'b1;
					tx_busy <= 1'b1;
					tx_counter <= ((DATA_BITS+STOP_BITS+1)*WHOLE_BIT_COUNT);
				end
			end
		end
	end

	generate
      for (i=0; i < (DATA_BITS+STOP_BITS+1); i = i+1)  begin: tx_shift_conditions
			if (i==0)  assign tx_shift_condition[i] = (tx_counter==((WHOLE_BIT_COUNT*(DATA_BITS+STOP_BITS+1)) - (WHOLE_BIT_COUNT*(i+1))));
			else assign tx_shift_condition[i] = (tx_counter==((WHOLE_BIT_COUNT*(DATA_BITS+STOP_BITS+1)) - (WHOLE_BIT_COUNT*(i+1)))) || tx_shift_condition[i-1];
      end
	endgenerate
		
	assign rs232_tx = !tx_busy || (tx_busy && tx_data_packet[0]);
	assign busy = tx_busy;
	assign tx_done = tx_done_reg;

	function integer clogb2;input [31:0] value;
		for (clogb2=0; value>0; clogb2=clogb2+1)
		value = value>>1;
	endfunction

endmodule




