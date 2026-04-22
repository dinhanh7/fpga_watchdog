`timescale 1ns/1ns

// =============================================================================
// Module: sync_debounce
// Description: Synchronizes asynchronous input signals using a 2-FF 
//              synchronizer and filters out bounces using a counter.
//              Also includes a falling edge detector.
// =============================================================================

module sync_debounce #(
    parameter DELAY_CYCLES = 20'd1_000_000 // 20ms at 50MHz (20ns period)
)(
    input  wire clk,
    input  wire rst_n,          // Active-low asynchronous reset
    input  wire button_i,       // Asynchronous active-low button input
    
    output reg  button_o,       // Synchronized and debounced button state
    output wire falling_edge    // 1-cycle pulse upon falling edge detection
);

    // =========================================================
    // 2-FF SYNCHRONIZER
    // =========================================================
    // Mitigates metastability when crossing clock domains.
    reg sync1;
    reg sync2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Buttons typically have pull-ups, so default unpressed state is 1
            {sync2, sync1} <= 2'b11; 
        end else begin
            // Shift the input signal through two flip-flops
            {sync2, sync1} <= {sync1, button_i};
        end
    end

    // =========================================================
    // DEBOUNCER COUNTER
    // =========================================================
    reg [19:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 20'd0;
            button_o <= 1'b1; // Default unpressed
        end else begin
            if (sync2 == button_o) begin
                // If synchronized input matches current output, reset counter
                cnt <= 20'd0;
            end else begin
                // If there's a difference (button being pressed/released), increment
                cnt <= cnt + 1'b1;
                
                // If the new state is maintained for the delay duration
                if (cnt >= (DELAY_CYCLES - 1'b1)) begin
                    button_o <= sync2; // Update the stable output
                    cnt      <= 20'd0;
                end
            end
        end
    end

    // =========================================================
    // FALLING EDGE DETECTOR
    // =========================================================
    reg button_o_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_o_prev <= 1'b1;
        end else begin
            button_o_prev <= button_o;
        end
    end
    
    // Generate a 1-cycle pulse when transitioning from 1 to 0
    assign falling_edge = (button_o_prev == 1'b1) && (button_o == 1'b0);

endmodule