`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Bus Master 	- Controls slave modules
//				- Designed to be used with UART interface
// This module is designed to provide an entry point for debugging and developing
// more complex FPGA code.  This code is not that elegant, but it provides a
// reliable method to communicate with the FPGA until more advanced links are finished.  
//
// DataForm: Command (1 byte), Address (4 bytes), Data (4 bytes)
// Commands : Write, Read and Status (bursting may come later)
//////////////////////////////////////////////////////////////////////////////////

module fcl_bus_master
	//------------------------------------------------------
	//--				External Parameters			 	  --
	//------------------------------------------------------
#(
    parameter integer INPUT_CLOCK_SPEED = 50000000,
	parameter integer UART_BAUD_RATE = 115200,
	parameter integer BUS_TIME_OUT = 100,
	parameter integer UART_TIME_OUT = 1000000 //Should be approx 2 x 10 x 10 x INPUT_CLOCK_SPEED / UART_BAUD_RATE
)

	//------------------------------------------------------
	//--					Ports					 	  --
	//------------------------------------------------------
(
	output	wire 			tx_out,
	input	wire 			rx_in,
	
	input	wire  	[31:0] 	bus_data_in,
	output	wire  	[31:0] 	bus_data_out,
	output	wire  	[31:0] 	bus_addr_out,
	output 	reg 			bus_read,
	output 	reg 			bus_write,
	input	wire 			bus_ack,

	output	wire 			bus_error_led,  // Goes high for 1 second with UART Timeout errors, and 250ms for BUS Timeout errors

	input	wire 			_reset,
	input	wire 			sys_clk
);

	
	//------------------------------------------------------
	//--		Local Parameters and Variables		 	  --
	//------------------------------------------------------
	localparam integer SECOND_COUNT = INPUT_CLOCK_SPEED;
	localparam integer SECOND_COUNT_W = clogb2(SECOND_COUNT);
	
	localparam integer BUS_TIME_OUT_W = clogb2(BUS_TIME_OUT);
	localparam integer UART_TIME_OUT_W = clogb2(UART_TIME_OUT);
	
	
	localparam [7:0] UART_WRITE_CODE = 8'h80;
	localparam [7:0] UART_READ_CODE = 8'h00;
	localparam [7:0] UART_STATUS_CODE = 8'h40;

	reg [7:0] uart_tx_data;
	wire [7:0] uart_rx_data;
	reg [7:0] uart_rx_data_latch;
	wire uart_rx_data_valid;
	reg tx_data_send;
	wire uart_tx_done;
	
	reg bus_error_flag;
	reg uart_error_flag;
				
	reg [3:0] sbm_control;
	localparam [3:0] sbms_idle = 4'h0;
	localparam [3:0] sbms_command_init = 4'h1;
	localparam [3:0] sbms_write1 = 4'h2;
	localparam [3:0] sbms_write2 = 4'h3;
	localparam [3:0] sbms_read1 = 4'h4;
	localparam [3:0] sbms_read2 = 4'h5;
	localparam [3:0] sbms_read3 = 4'h6;
	localparam [3:0] sbms_status1 = 4'h7;
	localparam [3:0] sbms_status2 = 4'h8;
	
	wire valid_command_node;
	reg [3:0] byte_counter;
	reg [7:0] addr_reg[3:0];
	reg [7:0] data_reg[3:0];
	
	reg [(BUS_TIME_OUT_W-1):0] bus_watchdog_counter;
	reg [(UART_TIME_OUT_W-1):0] uart_watchdog_counter;
	reg [7:0] bus_error_counter;
	reg [7:0] uart_error_counter;
	reg [(SECOND_COUNT_W-1):0] bus_error_ind_counter; 

	reg [15:0] uplink_timer; //Link Time in seconds
	reg [(SECOND_COUNT_W-1):0] second_counter; 
	reg [31:0] read_buffer;
	

	
	//------------------------------------------------------
	//--				 	 Logic 				 		  --
	//------------------------------------------------------
    fcl_uart_rs232  #(
		.INPUT_CLOCK_SPEED(INPUT_CLOCK_SPEED),
		.BAUD_RATE(UART_BAUD_RATE))
	serial_port (
        .rs232_rx(rx_in), .rs232_tx(tx_out),  
		.rx_data(uart_rx_data), .rx_data_valid(rx_data_valid),
		.tx_data(uart_tx_data), .tx_data_send(tx_data_send),
		.tx_done(uart_tx_done), .busy(),
		._reset(_reset), .sys_clk(sys_clk));

	always @(posedge sys_clk or negedge _reset) begin
		if (!_reset) begin
			uplink_timer <= {16{1'b0}};
			second_counter <= {SECOND_COUNT_W{1'b0}};
		end 
		else begin
			if (second_counter == SECOND_COUNT[(SECOND_COUNT_W-1):0]) begin
				second_counter <= {SECOND_COUNT_W{1'b0}};
				if (!(&uplink_timer)) uplink_timer <= uplink_timer + 1'b1;
			end
			else second_counter <= second_counter + 1'b1;
		end
	end
	
	// Error Counters and Displays
	always @(posedge sys_clk or negedge _reset) begin
		if (!_reset) begin
			bus_error_counter <= {8{1'b0}};
			uart_error_counter <= {8{1'b0}};
			bus_error_ind_counter <= {SECOND_COUNT_W{1'b0}};
		end 
		else begin
			if ((uart_error_flag)&&(sbm_control!=sbms_idle)&&(!(&uart_error_counter))) begin
				uart_error_counter <= uart_error_counter + 1'b1;
			end
			if ((bus_error_flag)&&(sbm_control!=sbms_idle)&&(!(&bus_error_counter))) begin
				bus_error_counter <= bus_error_counter + 1'b1;
			end
			
			if (bus_error_ind_counter != {SECOND_COUNT_W{1'b0}}) begin
				bus_error_ind_counter <= bus_error_ind_counter - 1'b1;
			end
			else if (uart_error_flag) begin
				bus_error_ind_counter <= SECOND_COUNT[(SECOND_COUNT_W-1):0];
			end
			else if (bus_error_flag) begin
				bus_error_ind_counter <= (SECOND_COUNT[(SECOND_COUNT_W-1):0]>>2);
			end
		end
	end
	assign bus_error_led = (bus_error_ind_counter != {SECOND_COUNT_W{1'b0}});
					
	assign valid_command_node = ((uart_rx_data_latch == UART_WRITE_CODE) || (uart_rx_data_latch == UART_READ_CODE) || (uart_rx_data_latch == UART_STATUS_CODE));
					
    //State Machine
	always @(posedge sys_clk or negedge _reset) begin
        if (!_reset) begin
            sbm_control <= sbms_idle;
				byte_counter[3:0] <= 4'h0;
				bus_write <= 1'b0;
				bus_read <= 1'b0;
				bus_watchdog_counter <= {BUS_TIME_OUT_W{1'b0}};
				uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
				bus_error_flag <= 1'b0;
				uart_error_flag <= 1'b0;
				tx_data_send <= 1'b0;
			
				addr_reg[0][7:0] <= 8'h00;
				addr_reg[1][7:0] <= 8'h00;
				addr_reg[2][7:0] <= 8'h00;
				addr_reg[3][7:0] <= 8'h00;
				data_reg[0][7:0] <= 8'h00;
				data_reg[1][7:0] <= 8'h00;
				data_reg[2][7:0] <= 8'h00;
				data_reg[3][7:0] <= 8'h00;
        end 
		else begin
			if (bus_error_flag||uart_error_flag) begin
				bus_error_flag <= 1'b0;
				uart_error_flag <= 1'b0;
				sbm_control <= sbms_idle;
			end
			else begin
				case(sbm_control)
					sbms_idle: begin
						tx_data_send <= 1'b0;
						bus_write <= 1'b0;
						if (rx_data_valid) begin 
							sbm_control <= sbms_command_init;
							uart_rx_data_latch <= uart_rx_data;
						end
						bus_watchdog_counter <= {BUS_TIME_OUT_W{1'b0}};
						uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
						bus_error_flag <= 1'b0;
						uart_error_flag <= 1'b0;
				
						addr_reg[0][7:0] <= 8'h00;
						addr_reg[1][7:0] <= 8'h00;
						addr_reg[2][7:0] <= 8'h00;
						addr_reg[3][7:0] <= 8'h00;
						data_reg[0][7:0] <= 8'h00;
						data_reg[1][7:0] <= 8'h00;
						data_reg[2][7:0] <= 8'h00;
						data_reg[3][7:0] <= 8'h00;
					end
					sbms_command_init: begin
						uart_watchdog_counter <= uart_watchdog_counter + 1'b1;
						if (uart_watchdog_counter > UART_TIME_OUT) uart_error_flag <= 1'b1;
						else begin
							if (valid_command_node) begin
								uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
								byte_counter[3:0] <= 4'h0;
							end
							case(uart_rx_data_latch)
								UART_WRITE_CODE 	:  sbm_control <= sbms_write1;
								UART_READ_CODE 		:  sbm_control <= sbms_read1; 
								UART_STATUS_CODE 	:  sbm_control <= sbms_status1; 
							endcase
						end
					end
					sbms_write1: begin
						uart_watchdog_counter <= uart_watchdog_counter + 1'b1;
						if (uart_watchdog_counter > UART_TIME_OUT) uart_error_flag <= 1'b1;
						else begin
							if (byte_counter > 4'h7) begin
								sbm_control <= sbms_write2;
								uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
								bus_write <= 1'b1;
							end
							if (rx_data_valid) begin 
								case(byte_counter)
									4'h0 :  addr_reg[3] <= uart_rx_data;
									4'h1 :  addr_reg[2] <= uart_rx_data;
									4'h2 :  addr_reg[1] <= uart_rx_data;
									4'h3 :  addr_reg[0] <= uart_rx_data;
									4'h4 :  data_reg[3] <= uart_rx_data;
									4'h5 :  data_reg[2] <= uart_rx_data;
									4'h6 :  data_reg[1] <= uart_rx_data;
									4'h7 :  data_reg[0] <= uart_rx_data;
								endcase
								byte_counter <= byte_counter + 1'b1;
								uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
							end
						end
					end
					sbms_write2: begin
						bus_watchdog_counter <= bus_watchdog_counter + 1'b1;
						if (bus_watchdog_counter > BUS_TIME_OUT) bus_error_flag <= 1'b1;
						else begin
							bus_write <= 1'b0;
							if (bus_ack) begin
								sbm_control <= sbms_idle;
								bus_watchdog_counter <= {BUS_TIME_OUT_W{1'b0}};
							end
						end
					end

					sbms_read1: begin
						uart_watchdog_counter <= uart_watchdog_counter + 1'b1;
						if (uart_watchdog_counter > UART_TIME_OUT) uart_error_flag <= 1'b1;
						else begin
							if (byte_counter > 4'h3) begin
								sbm_control <= sbms_read2;
								uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
								bus_read <= 1'b1;
							end
							if (rx_data_valid) begin 
								case(byte_counter)
									4'h0 :  addr_reg[3] <= uart_rx_data;
									4'h1 :  addr_reg[2] <= uart_rx_data;
									4'h2 :  addr_reg[1] <= uart_rx_data;
									4'h3 :  addr_reg[0] <= uart_rx_data;
								endcase
								byte_counter <= byte_counter + 1'b1;
								uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
							end
						end
					end
					sbms_read2: begin
						bus_watchdog_counter <= bus_watchdog_counter + 1'b1;
						if (bus_watchdog_counter > BUS_TIME_OUT) bus_error_flag <= 1'b1;
						else begin
							byte_counter[3:0] <= 4'h0;
							bus_read <= 1'b0;
							if (bus_ack) begin
								read_buffer <= bus_data_in; //Buffer data in
								sbm_control <= sbms_read3;
								bus_watchdog_counter <= {BUS_TIME_OUT_W{1'b0}};
								tx_data_send <= 1'b1;
								uart_tx_data <= bus_data_in[31:24]; 
							end
						end
					end
					sbms_read3: begin	
						uart_watchdog_counter <= uart_watchdog_counter + 1'b1;
						if (byte_counter == 4'h0) uart_tx_data <= read_buffer[23:16];
						if (byte_counter == 4'h1) uart_tx_data <= read_buffer[15:8];
						if (byte_counter == 4'h2) uart_tx_data <= read_buffer[7:0];
						if (uart_watchdog_counter > UART_TIME_OUT) uart_error_flag <= 1'b1;
						else begin
							if (uart_tx_done) begin
								if (byte_counter < 4'h3) begin
									byte_counter <= byte_counter + 1'b1;
									tx_data_send <= 1'b1;
									uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
								end
								else sbm_control <= sbms_idle;
							end
							else  tx_data_send <= 1'b0;
						end
					end
					
					sbms_status1: begin
						tx_data_send <= 1'b1;
						uart_tx_data <= bus_error_counter;
						sbm_control <= sbms_status2;
					end	
					sbms_status2: begin	
						uart_watchdog_counter <= uart_watchdog_counter + 1'b1;
						if (byte_counter == 4'h0) uart_tx_data <= uart_error_counter;
						if (byte_counter == 4'h1) uart_tx_data <= uplink_timer[15:8];
						if (byte_counter == 4'h2) uart_tx_data <= uplink_timer[7:0];
						if (uart_watchdog_counter > UART_TIME_OUT) uart_error_flag <= 1'b1;
						else begin
							if (uart_tx_done) begin
								if (byte_counter < 4'h3) begin
									byte_counter <= byte_counter + 1'b1;
									uart_watchdog_counter <= {UART_TIME_OUT_W{1'b0}};
									tx_data_send <= 1'b1;
								end
								else sbm_control <= sbms_idle;
							end
							else  tx_data_send <= 1'b0;
						end
					end

					default: sbm_control <= sbms_idle;
				endcase
			end
        end
	end
	
	assign bus_addr_out[7:0] = addr_reg[0][7:0];
	assign bus_addr_out[15:8] = addr_reg[1][7:0];
	assign bus_addr_out[23:16] = addr_reg[2][7:0];
	assign bus_addr_out[31:24] = addr_reg[3][7:0];

	assign bus_data_out[7:0] = data_reg[0][7:0];
	assign bus_data_out[15:8] = data_reg[1][7:0];
	assign bus_data_out[23:16] = data_reg[2][7:0];
	assign bus_data_out[31:24] = data_reg[3][7:0];

	function integer clogb2;input [31:0] value;
		for (clogb2=0; value>0; clogb2=clogb2+1)
		value = value>>1;
	endfunction
	
endmodule
