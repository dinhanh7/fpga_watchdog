`timescale 1ns / 1ps

// =============================================================================
// Module: watchdog_core
// Description: Core FSM and Timers for the Watchdog Monitor System.
//              Simulates the behavior of the TPS3431 Watchdog IC on an FPGA.
//
// FSM States:
//   DISABLE  -> EN=0: WDO=1 (OK), ENOUT=0, all kicks ignored.
//   ARMING   -> EN asserted: counts arm_delay_us, kicks ignored.
//   MONITOR  -> Normal monitoring: counts tWD_ms, valid kicks reset timer.
//   FAULT    -> Timeout occurred: WDO=0 (Fault), counts tRST_ms then recovers.
//              CLR_FAULT pulse can assert recovery immediately.
//
// EN Source:  en_hw (Hardware S2 button) OR en_sw (UART CTRL[0] bit)
// WDI Source: Depends on wdi_src (CTRL[1] bit):
//             wdi_src=0 -> Accepts kicks from S1 (Hardware) only.
//             wdi_src=1 -> Accepts kicks from UART (Software) only.
// =============================================================================

module watchdog_core #(
    parameter CLK_FREQ = 27_000_000  // System clock frequency (Hz)
)(
    input  wire        clk,
    input  wire        rst_n,

    // =========================================================
    // HARDWARE INPUTS (From sync_debounce, active-high)
    // =========================================================
    input  wire        en_hw_i,         // EN from S2 button (1 = Enabled)
    input  wire        wdi_falling_hw_i,// WDI kick pulse from S1 (1-cycle pulse)

    // =========================================================
    // SOFTWARE INPUTS (From uart_frame_parser)
    // =========================================================
    input  wire        uart_kick_pulse_i, // WDI kick pulse from UART CMD 0x03

    // =========================================================
    // CONFIGURATION (From regfile)
    // =========================================================
    input  wire        en_sw_i,         // Software Enable (CTRL bit 0)
    input  wire        wdi_src_i,       // WDI Source: 0=HW only, 1=SW only
    input  wire        clr_fault_i,     // Clear Fault pulse (W1C from CTRL bit 2)
    input  wire [31:0] tWD_ms_i,        // Watchdog timeout duration (ms)
    input  wire [31:0] tRST_ms_i,       // WDO fault holding duration (ms)
    input  wire [15:0] arm_delay_us_i,  // Initial arming delay (us)

    // =========================================================
    // STATUS TO REGFILE
    // =========================================================
    output wire        en_effective_o,  // Watchdog passed Arming phase
    output wire        fault_active_o,  // Watchdog is in Fault state
    output wire        enout_state_o,   // Current value of ENOUT pin
    output wire        wdo_state_o,     // Current value of WDO pin
    output reg         last_kick_src_o, // Source of last kick: 0=HW/S1, 1=SW/UART

    // =========================================================
    // PHYSICAL OUTPUTS
    // =========================================================
    output reg         wdo_pin_o,       // WDO: 1=OK (Hi-Z), 0=Fault (pulled low)
    output reg         enout_pin_o      // ENOUT: 1=System active, 0=Disabled
);

    // =========================================================
    // FSM STATES
    // =========================================================
    localparam S_DISABLE = 2'd0;  // Watchdog disabled
    localparam S_ARMING  = 2'd1;  // Waiting for arming delay
    localparam S_MONITOR = 2'd2;  // Normal monitoring (counting tWD)
    localparam S_FAULT   = 2'd3;  // Fault detected (counting tRST)

    reg [1:0] state;

    // Reset signal for Prescalers
    reg reset_prescalers;

    // =========================================================
    // 1 MICROSECOND TICK GENERATOR (us_tick)
    // =========================================================
    // Divides system clock to generate a 1-cycle pulse every 1us.
    localparam US_DIV = CLK_FREQ / 1_000_000;

    reg [7:0] us_cnt;   // 8-bit allows up to 255MHz system clock
    reg       us_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            us_cnt  <= 8'd0;
            us_tick <= 1'b0;
        end else if (reset_prescalers) begin
            us_cnt  <= 8'd0;
            us_tick <= 1'b0;
        end else begin
            if (us_cnt == US_DIV - 1) begin
                us_cnt  <= 8'd0;
                us_tick <= 1'b1;
            end else begin
                us_cnt  <= us_cnt + 8'd1;
                us_tick <= 1'b0;
            end
        end
    end

    // =========================================================
    // 1 MILLISECOND TICK GENERATOR (ms_tick)
    // =========================================================
    // Counts 1000 us_tick pulses to generate a 1-cycle pulse every 1ms.
    reg [9:0] ms_sub_cnt;  // 10-bit counter (0 to 999)
    reg       ms_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_sub_cnt <= 10'd0;
            ms_tick    <= 1'b0;
        end else if (reset_prescalers) begin
            ms_sub_cnt <= 10'd0;
            ms_tick    <= 1'b0;
        end else begin
            ms_tick <= 1'b0;  // Default down
            if (us_tick) begin
                if (ms_sub_cnt == 10'd999) begin
                    ms_sub_cnt <= 10'd0;
                    ms_tick    <= 1'b1;
                end else begin
                    ms_sub_cnt <= ms_sub_cnt + 10'd1;
                end
            end
        end
    end

    // =========================================================
    // ENABLE & KICK COMBINATORIAL LOGIC
    // =========================================================
    // Watchdog enabled either by Hardware (S2) OR Software (CTRL bit 0)
    wire en_combined = en_hw_i | en_sw_i;

    // Valid kick depends on wdi_src configuration:
    //   wdi_src = 0 -> Hardware S1 button only
    //   wdi_src = 1 -> Software UART kick only
    wire kick_valid = (wdi_src_i == 1'b0) ? wdi_falling_hw_i : uart_kick_pulse_i;

    // =========================================================
    // MULTIPURPOSE TIMER COUNTER
    // =========================================================
    // Used to count arm_delay, tWD, and tRST respectively based on state.
    reg [31:0] timer_cnt;

    // =========================================================
    // MAIN FSM LOGIC
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Safe initialization: Watchdog is fully disabled
            state            <= S_DISABLE;
            timer_cnt        <= 32'd0;
            wdo_pin_o        <= 1'b1;   // WDO default high (No Fault)
            enout_pin_o      <= 1'b0;   // ENOUT default low (Disabled)
            last_kick_src_o  <= 1'b0;
            reset_prescalers <= 1'b0;
        end else begin
            reset_prescalers <= 1'b0; // Default down

            // =========================================================
            // GLOBAL OVERRIDE: If EN goes low at ANY time
            //   -> Immediately return to DISABLE state.
            // =========================================================
            if (!en_combined) begin
                state            <= S_DISABLE;
                timer_cnt        <= 32'd0;
                wdo_pin_o        <= 1'b1;   // Release WDO fault
                enout_pin_o      <= 1'b0;   // Turn off ENOUT
                reset_prescalers <= 1'b1;   // Keep prescalers reset while disabled
            end else begin
                // EN is asserted (en_combined = 1)
                case (state)

                    // -------------------------------------------------
                    // DISABLE: Watchdog is off, waiting for EN to go high.
                    // -------------------------------------------------
                    S_DISABLE: begin
                        wdo_pin_o        <= 1'b1;
                        enout_pin_o      <= 1'b0;
                        timer_cnt        <= 32'd0;
                        reset_prescalers <= 1'b1;
                        // Since we are in the `else` branch of `!en_combined`, 
                        // EN is high -> transition to ARMING.
                        state <= S_ARMING;
                    end

                    // -------------------------------------------------
                    // ARMING: Count arm_delay_us, ignore all WDI kicks.
                    // After delay -> Enable ENOUT, transition to MONITOR.
                    // -------------------------------------------------
                    S_ARMING: begin
                        if (us_tick) begin
                            // Use timer_cnt + 1 to prevent Underflow when arm_delay_us_i = 0
                            if (timer_cnt + 32'd1 >= {16'd0, arm_delay_us_i}) begin
                                // Arming delay finished
                                timer_cnt        <= 32'd0;
                                enout_pin_o      <= 1'b1;   // Assert ENOUT, system is ready
                                reset_prescalers <= 1'b1;
                                state            <= S_MONITOR;
                            end else begin
                                timer_cnt <= timer_cnt + 32'd1;
                            end
                        end
                        // All kicks ignored during Arming
                    end

                    // -------------------------------------------------
                    // MONITOR: Normal system monitoring.
                    //   - Valid kick -> Reset timeout counter
                    //   - tWD_ms expires without kick -> Transition to FAULT
                    // -------------------------------------------------
                    S_MONITOR: begin
                        if (kick_valid) begin
                            // Valid kick received -> Reset timeout counter
                            timer_cnt        <= 32'd0;
                            reset_prescalers <= 1'b1; // Fix Prescaler Drift
                            // Record the source of the successful kick
                            last_kick_src_o  <= wdi_src_i;  // 0=HW, 1=SW
                        end else if (ms_tick) begin
                            // Use timer_cnt + 1 to prevent Underflow when tWD_ms_i = 0
                            if (timer_cnt + 32'd1 >= tWD_ms_i) begin
                                // TIMEOUT! No kick received within tWD
                                timer_cnt        <= 32'd0;
                                wdo_pin_o        <= 1'b0;   // Assert WDO low (Fault)
                                reset_prescalers <= 1'b1;
                                state            <= S_FAULT;
                            end else begin
                                timer_cnt <= timer_cnt + 32'd1;
                            end
                        end
                    end

                    // -------------------------------------------------
                    // FAULT: WDO is pulled low (Fault state).
                    //   - CLR_FAULT -> Release WDO immediately
                    //   - tRST_ms expires -> Release WDO automatically, return to MONITOR
                    //   - Kicks are ignored
                    // -------------------------------------------------
                    S_FAULT: begin
                        if (clr_fault_i) begin
                            // Received software clear fault command (CTRL bit 2)
                            wdo_pin_o        <= 1'b1;   // Release WDO immediately
                            timer_cnt        <= 32'd0;
                            reset_prescalers <= 1'b1;
                            state            <= S_MONITOR;
                        end else if (ms_tick) begin
                            // Use timer_cnt + 1 to prevent Underflow when tRST_ms_i = 0
                            if (timer_cnt + 32'd1 >= tRST_ms_i) begin
                                // Reset delay duration tRST finished
                                wdo_pin_o        <= 1'b1;   // Release WDO
                                timer_cnt        <= 32'd0;
                                reset_prescalers <= 1'b1;
                                state            <= S_MONITOR;
                            end else begin
                                timer_cnt <= timer_cnt + 32'd1;
                            end
                        end
                        // All kicks ignored during Fault
                    end

                    default: state <= S_DISABLE;
                endcase
            end
        end
    end

    // =========================================================
    // STATUS SIGNAL ASSIGNMENTS FOR REGFILE
    // =========================================================
    // en_effective = 1 when Watchdog is actively monitoring or faulting
    assign en_effective_o = (state == S_MONITOR) || (state == S_FAULT);

    // fault_active = 1 when currently in Fault state
    assign fault_active_o = (state == S_FAULT);

    // Pass physical output pin states
    assign enout_state_o = enout_pin_o;
    assign wdo_state_o   = wdo_pin_o;

endmodule
