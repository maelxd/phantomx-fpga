`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////

module fcl_uart_bidir
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
    parameter integer INPUT_CLOCK_SPEED = 50000000,
	parameter integer BAUD_RATE = 115200,
	parameter integer DATA_BITS = 8,
	parameter integer STOP_BITS = 1,
	parameter integer USE_TRI_BUFFER = 0
)
	
	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
    input 	wire							_reset_in,
    input 	wire							clk_in,
	
    inout 	wire							uart_sda_inout,
    input 	wire							uart_sda_in,
    output 	wire							uart_sda_out,

    output 	reg		[(DATA_BITS-1):0] 		rx_data_out,
	output 	reg								rx_data_valid_out,
    input 	wire	[(DATA_BITS-1):0] 		tx_data_in,
	input 	wire							tx_data_send_in,
	output 	wire							tx_done_out,
	output 	wire							tx_busy_out
);
	
	
	//------------------------------------------------------
	//--				Local Parameters			 	  --
	//------------------------------------------------------
	localparam WHOLE_BIT_COUNT = INPUT_CLOCK_SPEED / BAUD_RATE;
	localparam HALF_BIT_COUNT = WHOLE_BIT_COUNT / 2;
	localparam TOTAL_BITS = DATA_BITS + STOP_BITS + 1;
	localparam COUNTER_WIDTH = clogb2(WHOLE_BIT_COUNT*TOTAL_BITS);
	
	genvar i;
	
	
	//------------------------------------------------------
	//--			Bidirectional Pin Logic				  --
	//------------------------------------------------------
	wire 			rs232_rx;
    wire			rs232_tx;

	generate
	if (USE_TRI_BUFFER) begin
		assign uart_sda_inout = rs232_tx ? 1'bz : 1'b0;
		assign rs232_rx = uart_sda_inout;
		assign uart_sda_out = 1'b0;
	end
	else begin
		assign uart_sda_out = rs232_tx;
		assign rs232_rx = uart_sda_in;
		assign uart_sda_inout = 1'bz;
	end
	endgenerate

	
	//------------------------------------------------------
	//--				Rx Path Variables			 	  --
	//------------------------------------------------------
	reg		[(DATA_BITS-1):0]		rx_data_reg;
	reg		[1:0]					rs232_rx_buf;
	wire							rs232_rx_fe;
	reg		[(COUNTER_WIDTH-1):0]	rx_counter;
	
	reg		[1:0]					rxsm_control;
	localparam  [1:0]	rxsm_idle = 2'b00;
	localparam  [1:0]	rxsm_read = 2'b01;
	localparam  [1:0]	rxsm_done = 2'b10;
	
	
	//------------------------------------------------------
	//--				  Rx Path Logic 		 		  --
	//------------------------------------------------------
	//Falling edge on rs232 rx inticates the beginning of a packet
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			rs232_rx_buf[1:0] <= 2'b11;
		end
		else begin
			if (tx_busy_out) begin
				rs232_rx_buf[1:0] <= 2'b11;
			end
			else begin
				rs232_rx_buf[1:0] <= {rs232_rx_buf[0], rs232_rx};
			end
		end
	end
	
	assign rs232_rx_fe = rs232_rx_buf[1] && !rs232_rx_buf[0];

    //State Machine
	always @(posedge clk_in or negedge _reset_in) begin
        if (!_reset_in) begin
			  rx_data_out[(DATA_BITS-1):0] <= {DATA_BITS{1'b0}};
			  rx_data_valid_out <= 1'b0;
			  rx_counter <= WHOLE_BIT_COUNT*(DATA_BITS+1) + HALF_BIT_COUNT;
        end
		else begin
			if (tx_busy_out) begin
				rx_data_valid_out <= 1'b0;
				rxsm_control <= rxsm_idle;
			end
			else begin
				case(rxsm_control)
					rxsm_idle: begin
						if (rs232_rx_fe) begin
							rxsm_control <= rxsm_read;
							rx_counter <= WHOLE_BIT_COUNT*(DATA_BITS+1) + HALF_BIT_COUNT;
						end
						rx_data_valid_out <= 1'b0;
					end
					rxsm_read: begin
						if (|rx_counter)
							rx_counter <= rx_counter - 1'b1;
						else 
							rxsm_control <= rxsm_done;
					end
					rxsm_done: begin
						rx_data_out[(DATA_BITS-1):0] <= rx_data_reg[(DATA_BITS-1):0];
						rx_data_valid_out <= 1'b1;
						rxsm_control <= rxsm_idle;
					end
					default: rxsm_control <= rxsm_idle;
				endcase
			end
        end
	end


	generate
	for (i=0; i < DATA_BITS; i = i+1)  begin: g_rx_read_enables
		always @(posedge clk_in or negedge _reset_in) begin
			if (!_reset_in) begin
				rx_data_reg[i] <= 1'b0;
			end 
			else if (rx_counter == (WHOLE_BIT_COUNT*(DATA_BITS-i))) begin
				rx_data_reg[i] <= rs232_rx;
			end
		end
	end
	endgenerate


	
	//------------------------------------------------------
	//--				Tx Path Variables			 	  --
	//------------------------------------------------------
	reg		[(TOTAL_BITS-1):0]		tx_data_packet;
	reg		[(COUNTER_WIDTH-1):0]	tx_counter;
	wire	[(TOTAL_BITS-1):0]		tx_shift_condition;

	reg 	[1:0]					txsm_control;
	localparam [1:0] txsm_idle = 2'b00;
	localparam [1:0] txsm_send = 2'b01;
	localparam [1:0] txsm_done = 2'b10;
	
	
	//------------------------------------------------------
	//--				  Tx Path Logic 		 		  --
	//------------------------------------------------------
	always @(posedge clk_in or negedge _reset_in) begin
		if (!_reset_in) begin
			txsm_control <= txsm_idle;
			tx_data_packet <= {TOTAL_BITS{1'b0}};
			tx_counter <= (TOTAL_BITS*WHOLE_BIT_COUNT);
		end
		else begin
			case(txsm_control)
				txsm_idle: begin
					if (tx_data_send_in) begin
						tx_data_packet[(TOTAL_BITS-1):0] <= {{STOP_BITS{1'b1}}, tx_data_in[(DATA_BITS-1):0], 1'b0};
						tx_counter <= (TOTAL_BITS*WHOLE_BIT_COUNT);
						txsm_control <= txsm_send;
					end
				end
				
				txsm_send: begin
					if (|tx_counter) begin
						tx_counter <= tx_counter - 1'b1;
					end
					else begin
						txsm_control <= txsm_done;
					end
					if (tx_shift_condition[TOTAL_BITS-1])  begin
						tx_data_packet[(TOTAL_BITS-1):0] <= {1'b1, tx_data_packet[(TOTAL_BITS-1):1]};
					end
				end
				
				txsm_done: begin
					txsm_control <= txsm_idle;
				end
				
				default: txsm_control <= txsm_idle;
			endcase
		end
	end
	
	assign tx_busy_out = (txsm_control != txsm_idle) || tx_data_send_in;
	assign tx_done_out = (txsm_control == txsm_done);

	generate
	for (i=0; i < TOTAL_BITS; i = i+1)  begin: g_tx_shift_conditions
		if (i==0) begin
			assign tx_shift_condition[i] = (tx_counter == (WHOLE_BIT_COUNT*(TOTAL_BITS-i-1)));
		end
		else begin
			assign tx_shift_condition[i] = (tx_counter == (WHOLE_BIT_COUNT*(TOTAL_BITS-i-1))) || tx_shift_condition[i-1];
		end
	end
	endgenerate
		
	assign rs232_tx = (txsm_control == txsm_idle) ? 1'b1 : tx_data_packet[0];

	
	function integer clogb2;input [31:0] value;
		for (clogb2=0; value>0; clogb2=clogb2+1)
		value = value>>1;
	endfunction

endmodule
