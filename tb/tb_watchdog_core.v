`timescale 1ns / 1ps

module tb_watchdog_core;

    // =========================================================================
    // THAM SỐ MÔ PHỎNG
    // Dùng CLK_FREQ = 1MHz để chạy nhanh: 1 clock = 1us, ms_tick mỗi 1000 clk
    // =========================================================================
    parameter CLK_FREQ       = 1_000_000;
    parameter CLK_PERIOD     = 1000;       // 1us = 1000ns
    parameter SIM_TWD_MS     = 5;
    parameter SIM_TRST_MS    = 2;
    parameter SIM_ARM_US     = 50;

    // =========================================================================
    // TÍN HIỆU
    // =========================================================================
    reg        clk;
    reg        rst_n;
    reg        en_hw;
    reg        wdi_falling_hw;
    reg        uart_kick_pulse;
    reg        en_sw;
    reg        wdi_src;
    reg        clr_fault;
    reg [31:0] tWD_ms;
    reg [31:0] tRST_ms;
    reg [15:0] arm_delay_us;

    wire       en_effective;
    wire       fault_active;
    wire       enout_state;
    wire       wdo_state;
    wire       last_kick_src;
    wire       wdo_pin;
    wire       enout_pin;

    integer pass_cnt = 0;
    integer fail_cnt = 0;

    // =========================================================================
    // DUT
    // =========================================================================
    watchdog_core #(.CLK_FREQ(CLK_FREQ)) dut (
        .clk            (clk),
        .rst_n        (rst_n),
        .en_hw          (en_hw),
        .wdi_falling_hw (wdi_falling_hw),
        .uart_kick_pulse(uart_kick_pulse),
        .en_sw          (en_sw),
        .wdi_src        (wdi_src),
        .clr_fault      (clr_fault),
        .tWD_ms         (tWD_ms),
        .tRST_ms        (tRST_ms),
        .arm_delay_us   (arm_delay_us),
        .en_effective   (en_effective),
        .fault_active   (fault_active),
        .enout_state    (enout_state),
        .wdo_state      (wdo_state),
        .last_kick_src  (last_kick_src),
        .wdo_pin        (wdo_pin),
        .enout_pin      (enout_pin)
    );

    // =========================================================================
    // CLOCK
    // =========================================================================
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // HELPER TASKS
    // =========================================================================
    task reset_all;
    begin
        rst_n <= 0; en_hw <= 0; en_sw <= 0;
        wdi_falling_hw <= 0; uart_kick_pulse <= 0;
        wdi_src <= 0; clr_fault <= 0;
        tWD_ms <= SIM_TWD_MS; tRST_ms <= SIM_TRST_MS; arm_delay_us <= SIM_ARM_US;
        repeat(5) @(posedge clk);
        rst_n <= 1;
        @(posedge clk);
    end
    endtask

    task hw_kick;
    begin
        @(posedge clk); wdi_falling_hw <= 1;
        @(posedge clk); wdi_falling_hw <= 0;
    end
    endtask

    task sw_kick;
    begin
        @(posedge clk); uart_kick_pulse <= 1;
        @(posedge clk); uart_kick_pulse <= 0;
    end
    endtask

    task pulse_clr;
    begin
        @(posedge clk); clr_fault <= 1;
        @(posedge clk); clr_fault <= 0;
    end
    endtask

    task wait_ms; input integer n;
    begin repeat(n * 1000) @(posedge clk); end
    endtask

    task wait_us; input integer n;
    begin repeat(n) @(posedge clk); end
    endtask

    task check;
        input [255:0] label;  // 32 chars max
        input expected;
        input actual;
    begin
        if (expected === actual) begin
            $display("[PASS] %0s (exp=%b got=%b)", label, expected, actual);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] %0s (exp=%b got=%b)", label, expected, actual);
            fail_cnt = fail_cnt + 1;
        end
    end
    endtask

    // =========================================================================
    // FSM MONITOR: In log mỗi khi trạng thái thay đổi
    // =========================================================================
    reg [1:0] prev_st;
    always @(posedge clk) begin
        prev_st <= dut.state;
        if (dut.state !== prev_st && rst_n) begin
            case (dut.state)
                2'd0: $display("  [%0t] FSM -> DISABLE",  $time);
                2'd1: $display("  [%0t] FSM -> ARMING",   $time);
                2'd2: $display("  [%0t] FSM -> MONITOR",  $time);
                2'd3: $display("  [%0t] FSM -> FAULT",    $time);
            endcase
        end
    end

    // =========================================================================
    // MAIN TEST SEQUENCE
    // =========================================================================
    initial begin
        $dumpfile("tb_watchdog_core.vcd");
        $dumpvars(0, tb_watchdog_core);

        $display("============================================================");
        $display("  TB: watchdog_core | CLK=%0dHz tWD=%0dms tRST=%0dms arm=%0dus",
                 CLK_FREQ, SIM_TWD_MS, SIM_TRST_MS, SIM_ARM_US);
        $display("============================================================");

        // ==========================================================
        // TEST 1: DISABLE (EN=0) - WDO=1, ENOUT=0, kick ignored
        // ==========================================================
        $display("\n--- TEST 1: DISABLE (EN=0) ---");
        reset_all;
        hw_kick;
        wait_us(10);
        check("T1.1 WDO=1 khi EN=0",       1'b1, wdo_pin);
        check("T1.2 ENOUT=0 khi EN=0",      1'b0, enout_pin);
        check("T1.3 en_effective=0",         1'b0, en_effective);
        check("T1.4 fault_active=0",         1'b0, fault_active);

        // ==========================================================
        // TEST 2: ARMING (EN 0->1, arm_delay, kick ignored)
        // ==========================================================
        $display("\n--- TEST 2: ARMING ---");
        reset_all;
        en_hw <= 1;
        @(posedge clk);

        wait_us(SIM_ARM_US / 2);
        check("T2.1 ENOUT=0 mid-arm",       1'b0, enout_pin);

        // Kick during arming -> must be ignored
        hw_kick;
        wait_us(5);
        check("T2.2 kick ignored in ARM",   1'b0, enout_pin);

        // Wait rest of arm_delay
        wait_us(SIM_ARM_US / 2 + 10);
        check("T2.3 ENOUT=1 post-arm",      1'b1, enout_pin);
        check("T2.4 WDO=1 post-arm",        1'b1, wdo_pin);
        check("T2.5 en_effective=1",         1'b1, en_effective);

        // ==========================================================
        // TEST 3: NORMAL KICK (periodic kick < tWD, WDO stays high)
        // ==========================================================
        $display("\n--- TEST 3: NORMAL KICK ---");
        reset_all;
        en_hw <= 1;
        wait_us(SIM_ARM_US + 20);

        repeat(5) begin
            wait_ms(SIM_TWD_MS - 2);
            hw_kick;
        end
        check("T3.1 WDO=1 after 5 kicks",   1'b1, wdo_pin);
        check("T3.2 fault_active=0",         1'b0, fault_active);
        check("T3.3 ENOUT=1",               1'b1, enout_pin);

        // ==========================================================
        // TEST 4: TIMEOUT (no kick -> WDO=0, then auto-release)
        // ==========================================================
        $display("\n--- TEST 4: TIMEOUT ---");
        reset_all;
        en_hw <= 1;
        wait_us(SIM_ARM_US + 20);

        // No kick, wait past tWD
        wait_ms(SIM_TWD_MS + 1);
        check("T4.1 WDO=0 after timeout",   1'b0, wdo_pin);
        check("T4.2 fault_active=1",         1'b1, fault_active);

        // Wait tRST -> auto release
        wait_ms(SIM_TRST_MS + 1);
        check("T4.3 WDO=1 after tRST",      1'b1, wdo_pin);
        check("T4.4 fault_active=0",         1'b0, fault_active);
        check("T4.5 back to MONITOR",        1'b1, en_effective);

        // ==========================================================
        // TEST 5: CLR_FAULT (immediate WDO release)
        // ==========================================================
        $display("\n--- TEST 5: CLR_FAULT ---");
        reset_all;
        en_hw <= 1;
        wait_us(SIM_ARM_US + 20);
        wait_ms(SIM_TWD_MS + 1);
        check("T5.1 WDO=0 in FAULT",        1'b0, wdo_pin);

        pulse_clr;
        repeat(3) @(posedge clk);
        check("T5.2 WDO=1 after CLR",       1'b1, wdo_pin);
        check("T5.3 fault_active=0",         1'b0, fault_active);

        // ==========================================================
        // TEST 6: EN off mid-MONITOR -> immediate DISABLE
        // ==========================================================
        $display("\n--- TEST 6: EN off mid-MONITOR ---");
        reset_all;
        en_hw <= 1;
        wait_us(SIM_ARM_US + 20);

        wait_ms(SIM_TWD_MS - 2);
        en_hw <= 0;
        repeat(3) @(posedge clk);
        check("T6.1 WDO=1 on EN off",       1'b1, wdo_pin);
        check("T6.2 ENOUT=0",               1'b0, enout_pin);
        check("T6.3 en_effective=0",         1'b0, en_effective);

        // Re-enable -> must re-arm
        en_hw <= 1;
        wait_us(SIM_ARM_US / 2);
        check("T6.4 re-arming (ENOUT=0)",    1'b0, enout_pin);
        wait_us(SIM_ARM_US / 2 + 10);
        check("T6.5 re-armed (ENOUT=1)",     1'b1, enout_pin);

        // ==========================================================
        // TEST 7: WDI_SRC=1 (SW only) - HW kick rejected
        // ==========================================================
        $display("\n--- TEST 7: WDI_SRC=1 (SW only) ---");
        reset_all;
        wdi_src <= 1; en_hw <= 1;
        wait_us(SIM_ARM_US + 20);

        // HW kick should be ignored -> timeout
        wait_ms(SIM_TWD_MS - 2);
        hw_kick;
        wait_ms(3);
        check("T7.1 HW kick ignored",       1'b0, wdo_pin);

        // Reset, test SW kick works
        reset_all;
        wdi_src <= 1; en_hw <= 1;
        wait_us(SIM_ARM_US + 20);
        repeat(3) begin
            wait_ms(SIM_TWD_MS - 2);
            sw_kick;
        end
        check("T7.2 SW kick accepted",      1'b1, wdo_pin);
        check("T7.3 last_kick=1(SW)",       1'b1, last_kick_src);

        // ==========================================================
        // TEST 8: WDI_SRC=0 (HW only) - SW kick rejected
        // ==========================================================
        $display("\n--- TEST 8: WDI_SRC=0 (HW only) ---");
        reset_all;
        wdi_src <= 0; en_hw <= 1;
        wait_us(SIM_ARM_US + 20);

        wait_ms(SIM_TWD_MS - 2);
        sw_kick;
        wait_ms(3);
        check("T8.1 SW kick ignored",       1'b0, wdo_pin);

        reset_all;
        wdi_src <= 0; en_hw <= 1;
        wait_us(SIM_ARM_US + 20);
        repeat(3) begin
            wait_ms(SIM_TWD_MS - 2);
            hw_kick;
        end
        check("T8.2 HW kick accepted",      1'b1, wdo_pin);
        check("T8.3 last_kick=0(HW)",       1'b0, last_kick_src);

        // ==========================================================
        // TEST 9: Software Enable (en_sw=1)
        // ==========================================================
        $display("\n--- TEST 9: Software Enable ---");
        reset_all;
        en_sw <= 1;
        wait_us(SIM_ARM_US + 20);
        check("T9.1 ENOUT=1 via en_sw",     1'b1, enout_pin);
        check("T9.2 en_effective=1",         1'b1, en_effective);

        // ==========================================================
        // TEST 10: Change tWD mid-run (UART config simulation)
        // ==========================================================
        $display("\n--- TEST 10: Change tWD mid-run ---");
        reset_all;
        en_hw <= 1;
        wait_us(SIM_ARM_US + 20);

        tWD_ms <= 32'd2;  // Reduce timeout to 2ms
        @(posedge clk);

        wait_ms(3);  // > 2ms new timeout
        check("T10.1 timeout w/ new tWD=2", 1'b0, wdo_pin);
        check("T10.2 fault_active=1",       1'b1, fault_active);

        // ==========================================================
        // SUMMARY
        // ==========================================================
        $display("\n============================================================");
        $display("  RESULT: %0d PASS, %0d FAIL (Total: %0d)",
                 pass_cnt, fail_cnt, pass_cnt + fail_cnt);
        if (fail_cnt == 0)
            $display("  >>> ALL TESTS PASSED! <<<");
        else
            $display("  >>> %0d TEST(S) FAILED! <<<", fail_cnt);
        $display("============================================================\n");
        $finish;
    end

endmodule
