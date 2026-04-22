`timescale 1ns / 1ps

// =============================================================================
// Module: uart_rx
// Description: UART Receiver module. Samples serial data using oversampling 
//              at the middle of the bit period. Configurable baud rate.
// =============================================================================

module uart_rx #(
    parameter CLK_FREQ  = 50_000_000, 
    parameter BAUD_RATE = 115200      
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx_i,      // UART RX serial input pin
    output reg  [7:0] data_o,         // 8-bit received data
    output reg        rx_done_o       // 1-cycle pulse indicating reception complete
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

    // =========================================================
    // 2-FF SYNCHRONIZER
    // =========================================================
    // Synchronize the RX signal to prevent metastability since it 
    // comes from an asynchronous clock domain.
    reg rx_sync1;
    reg rx_sync2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= uart_rx_i;
            rx_sync2 <= rx_sync1;
        end
    end

    // =========================================================
    // RX FSM LOGIC
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_count <= 16'd0;
            bit_index <= 3'd0;
            data_o    <= 8'd0;
            rx_done_o <= 1'b0;
        end else begin
            rx_done_o <= 1'b0; // Default to 0, asserting 1-cycle pulse only when done

            case (state)
                S_IDLE: begin
                    clk_count <= 16'd0;
                    bit_index <= 3'd0;
                    // Detect Start bit (transition to 0)
                    if (rx_sync2 == 1'b0) begin 
                        state <= S_START;
                    end
                end
                
                S_START: begin
                    // Wait until the middle of the Start bit period
                    if (clk_count == (CLKS_PER_BIT / 2) - 1) begin
                        // Re-verify Start bit to filter out noise/glitches
                        if (rx_sync2 == 1'b0) begin 
                            clk_count <= 16'd0;
                            state     <= S_DATA;
                        end else begin
                            state     <= S_IDLE; // Glitch detected, return to IDLE
                        end
                    end else begin
                        clk_count <= clk_count + 16'd1;
                    end
                end
                
                S_DATA: begin
                    // Wait for a full bit period to sample at the middle of Data bits
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        data_o[bit_index] <= rx_sync2; // Store the sampled bit
                        
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 3'd1;
                        end else begin
                            bit_index <= 3'd0;
                            state     <= S_STOP;
                        end
                    end
                end
                
                S_STOP: begin
                    // Wait for a full bit period for the Stop bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 16'd1;
                    end else begin
                        clk_count <= 16'd0;
                        rx_done_o <= 1'b1; // Trigger done pulse indicating valid data
                        state     <= S_IDLE;
                    end
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule