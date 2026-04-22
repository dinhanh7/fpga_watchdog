`timescale 1ns / 1ps

// =============================================================================
// Module: top_watchdog
// Description: Top-level module integrating the entire Watchdog Monitor System.
//
// Sub-modules:
//   1. sync_debounce (x2) - Debounce/sync for S1 (WDI) and S2 (EN) hardware buttons.
//   2. uart_rx            - Receives UART serial data from PC.
//   3. uart_tx            - Transmits UART serial data to PC.
//   4. uart_frame_parser  - Parses UART frames, dispatches to regfile.
//   5. regfile            - Configuration and status register map.
//   6. watchdog_core      - FSM and Timer simulating TPS3431 behavior.
// =============================================================================

module top_watchdog (
    input  wire clk,           // 27MHz system clock from onboard oscillator

    // =========================================================
    // HARDWARE BUTTONS
    // =========================================================
    input  wire wdi_pin,       // S1 Button: WDI Kick (active-low, pull-up)
    input  wire en_hw_pin,     // S2 Button: Hardware Enable (active-low, pull-up)

    // =========================================================
    // UART INTERFACE
    // =========================================================
    input  wire uart_rx_pin,   // UART RX input pin
    output wire uart_tx_pin,   // UART TX output pin

    // =========================================================
    // PHYSICAL OUTPUTS
    // =========================================================
    output wire wdo_pin,       // WDO LED (1=OK, 0=Fault)
    output wire enout_pin      // ENOUT LED (1=Active, 0=Disabled)
);

    // =========================================================
    // POWER-ON RESET GENERATOR
    // =========================================================
    // Creates an internal active-low reset signal upon power-up.
    reg [7:0] rst_cnt = 8'd0;
    reg       rst_n   = 1'b0;

    always @(posedge clk) begin
        if (rst_cnt != 8'hFF) begin
            rst_cnt <= rst_cnt + 8'd1;
        end else begin
            rst_n <= 1'b1;
        end
    end

    // =========================================================
    // BLOCK 1 & 2: BUTTON SYNC & DEBOUNCE
    // =========================================================
    // Debouncer for S1 (WDI Kick)
    wire wdi_debounced;
    wire wdi_falling;

    sync_debounce #(
        .DELAY_CYCLES(20'd540_000)  // ~20ms at 27MHz
    ) u_debounce_wdi (
        .clk          (clk),
        .rst_n        (rst_n),
        .button_i     (wdi_pin),
        .button_o     (wdi_debounced),
        .falling_edge (wdi_falling)
    );

    // Debouncer for S2 (Hardware Enable)
    wire en_hw_debounced;
    wire en_hw_falling; // Unused, but wired for completeness

    sync_debounce #(
        .DELAY_CYCLES(20'd540_000)  // ~20ms at 27MHz
    ) u_debounce_en (
        .clk          (clk),
        .rst_n        (rst_n),
        .button_i     (en_hw_pin),
        .button_o     (en_hw_debounced),
        .falling_edge (en_hw_falling)
    );

    // S2 is active-low (pressed = 0). We invert it to active-high internal logic:
    // When S2 pressed (en_hw_debounced=0) -> en_hw=1 (Watchdog enabled)
    wire en_hw = ~en_hw_debounced;

    // =========================================================
    // BLOCK 3: UART RX
    // =========================================================
    wire [7:0] rx_data;
    wire       rx_done;

    uart_rx #(
        .CLK_FREQ(27_000_000), 
        .BAUD_RATE(115200)
    ) u_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .uart_rx_i (uart_rx_pin),
        .data_o    (rx_data),
        .rx_done_o (rx_done)
    );

    // =========================================================
    // BLOCK 4: UART TX
    // =========================================================
    wire [7:0] tx_data;
    wire       tx_en;
    wire       tx_busy;

    uart_tx #(
        .CLK_FREQ(27_000_000), 
        .BAUD_RATE(115200)
    ) u_tx (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_i     (tx_data),
        .tx_start_i (tx_en),
        .uart_tx_o  (uart_tx_pin),
        .tx_busy_o  (tx_busy)
    );

    // =========================================================
    // BLOCK 5: UART FRAME PARSER
    // =========================================================
    wire [7:0]  reg_addr;
    wire        reg_we;
    wire        reg_re;
    wire [31:0] reg_wdata;
    wire [31:0] reg_rdata;
    wire        uart_kick_pulse;
    
    // Wires from Regfile to Core and Parser
    wire        en_sw;
    wire        wdi_src;
    wire        clr_fault;
    wire [31:0] tWD_ms;
    wire [31:0] tRST_ms;
    wire [15:0] arm_delay_us;

    uart_frame_parser u_parser (
        .clk               (clk),
        .rst_n             (rst_n),
        .rx_data_i         (rx_data),
        .rx_done_i         (rx_done),
        .tx_data_o         (tx_data),
        .tx_en_o           (tx_en),
        .tx_busy_i         (tx_busy),
        .reg_addr_o        (reg_addr),
        .reg_we_o          (reg_we),
        .reg_re_o          (reg_re),
        .reg_wdata_o       (reg_wdata),
        .reg_rdata_i       (reg_rdata),
        .uart_kick_pulse_o (uart_kick_pulse),
        .wdi_src_i         (wdi_src)
    );

    // =========================================================
    // BLOCK 6: REGISTER FILE
    // =========================================================
    wire        en_effective;
    wire        fault_active;
    wire        enout_state;
    wire        wdo_state;
    wire        last_kick_src;

    regfile u_regfile (
        .clk             (clk),
        .rst_n           (rst_n),
        .addr_i          (reg_addr),
        .we_i            (reg_we),
        .re_i            (reg_re),
        .wdata_i         (reg_wdata),
        .rdata_o         (reg_rdata),
        // Outputs to Core
        .en_sw_o         (en_sw),
        .wdi_src_o       (wdi_src),
        .clr_fault_o     (clr_fault),
        .tWD_ms_o        (tWD_ms),
        .tRST_ms_o       (tRST_ms),
        .arm_delay_us_o  (arm_delay_us),
        // Inputs from Core
        .en_effective_i  (en_effective),
        .fault_active_i  (fault_active),
        .enout_state_i   (enout_state),
        .wdo_state_i     (wdo_state),
        .last_kick_src_i (last_kick_src)
    );

    // =========================================================
    // BLOCK 7: WATCHDOG CORE
    // =========================================================
    watchdog_core #(
        .CLK_FREQ(27_000_000)
    ) u_watchdog (
        .clk               (clk),
        .rst_n             (rst_n),
        // Hardware Inputs
        .en_hw_i           (en_hw),
        .wdi_falling_hw_i  (wdi_falling),
        // Software Inputs
        .uart_kick_pulse_i (uart_kick_pulse),
        // Regfile Config
        .en_sw_i           (en_sw),
        .wdi_src_i         (wdi_src),
        .clr_fault_i       (clr_fault),
        .tWD_ms_i          (tWD_ms),
        .tRST_ms_i         (tRST_ms),
        .arm_delay_us_i    (arm_delay_us),
        // Regfile Status
        .en_effective_o    (en_effective),
        .fault_active_o    (fault_active),
        .enout_state_o     (enout_state),
        .wdo_state_o       (wdo_state),
        .last_kick_src_o   (last_kick_src),
        // Physical Outputs
        .wdo_pin_o         (wdo_pin),
        .enout_pin_o       (enout_pin)
    );

endmodule