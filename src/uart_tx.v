`timescale 1ns / 1ps

// =============================================================================
// Module: uart_tx
// Description: UART Transmitter module. Serializes 8-bit data into a bitstream
//              with Start and Stop bits. Configurable baud rate.
// =============================================================================

module uart_tx #(
    parameter CLK_FREQ  = 27_000_000,
    parameter BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start_i,     // 1-cycle pulse to initiate transmission
    input  wire [7:0] data_i,         // 8-bit data to transmit
    output reg        uart_tx_o,      // UART TX serial output pin
    output reg        tx_busy_o       // High when transmission is in progress
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // =========================================================
    // FSM STATES
    // =========================================================
    localparam S_IDLE  = 2'b00;
    localparam S_START = 2'b01;
    localparam S_DATA  = 2'b10;
    localparam S_STOP  = 2'b11;

    reg [1:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  data_reg;

    // =========================================================
    // TX FSM LOGIC
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            uart_tx_o <= 1'b1;        // TX line is high when idle
            tx_busy_o <= 1'b0;
            clk_count <= 16'd0;
            bit_index <= 3'd0;
            data_reg  <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    uart_tx_o <= 1'b1;
                    tx_busy_o <= 1'b0;
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;
                    if (tx_start_i) begin
                        data_reg  <= data_i;
                        tx_busy_o <= 1'b1;
                        state     <= S_START;
                    end
                end

                S_START: begin
                    uart_tx_o <= 1'b0; // Pull TX low for Start bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_DATA;
                    end
                end

                S_DATA: begin
                    uart_tx_o <= data_reg[bit_index]; // Transmit data bit by bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin 
                        clk_count <= 16'd0;
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 3'd1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= S_STOP;
                        end
                    end
                end

                S_STOP: begin
                    uart_tx_o <= 1'b1; // Pull TX high for Stop bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        state     <= S_IDLE;
                    end
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule