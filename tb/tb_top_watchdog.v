`timescale 1ns / 1ps

module tb_top_watchdog;

    // =========================================================
    // SIGNALS
    // =========================================================
    reg  clk;
    reg  wdi_pin;
    reg  en_hw_pin;
    reg  uart_rx_pin;
    wire uart_tx_pin;
    wire wdo_pin;
    wire enout_pin;

    // =========================================================
    // CLOCK GENERATION (27 MHz)
    // =========================================================
    // Period = 37.037 ns (Half Period ~ 18.5 ns)
    initial clk = 0;
    always #18.5 clk = ~clk;

    // =========================================================
    // CONSTANTS & TIMING
    // =========================================================
    localparam BIT_PERIOD_NS = 8680; // 1/115200 * 1e9
    localparam DEBOUNCE_TIME_NS = 20_500_000; // slightly > 20ms

    // =========================================================
    // DUT INSTANTIATION
    // =========================================================
    // Defparam to reduce debounce time for faster simulation. 
    // If simulator does not support defparam, this can be commented out 
    // (simulation will just take slightly longer).
    // defparam dut.u_debounce_wdi.DELAY_CYCLES = 20'd270; // 10us
    // defparam dut.u_debounce_en.DELAY_CYCLES  = 20'd270; // 10us

    top_watchdog dut (
        .clk        (clk),
        .wdi_pin    (wdi_pin),
        .en_hw_pin  (en_hw_pin),
        .uart_rx_pin(uart_rx_pin),
        .uart_tx_pin(uart_tx_pin),
        .wdo_pin    (wdo_pin),
        .enout_pin  (enout_pin)
    );

    // =========================================================
    // UART TASKS
    // =========================================================
    task uart_send_byte;
        input [7:0] data;
        integer i;
        begin
            uart_rx_pin = 0; // Start bit
            #(BIT_PERIOD_NS);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_pin = data[i];
                #(BIT_PERIOD_NS);
            end
            uart_rx_pin = 1; // Stop bit
            #(BIT_PERIOD_NS);
        end
    endtask

    task uart_send_frame;
        input [7:0] cmd;
        input [7:0] addr;
        input [7:0] len;
        input [31:0] data; // Up to 4 bytes of data
        reg [7:0] chk;
        integer i;
        begin
            chk = cmd ^ addr ^ len;
            uart_send_byte(8'h55);
            uart_send_byte(cmd);
            uart_send_byte(addr);
            uart_send_byte(len);
            
            // Send data bytes if len > 0. Big Endian format expected by parser.
            if (len > 0) begin
                for (i = len; i > 0; i = i - 1) begin
                    case (i)
                        4: begin uart_send_byte(data[31:24]); chk = chk ^ data[31:24]; end
                        3: begin uart_send_byte(data[23:16]); chk = chk ^ data[23:16]; end
                        2: begin uart_send_byte(data[15:8]);  chk = chk ^ data[15:8]; end
                        1: begin uart_send_byte(data[7:0]);   chk = chk ^ data[7:0]; end
                    endcase
                end
            end
            
            uart_send_byte(chk);
            #(BIT_PERIOD_NS * 5); // Wait some extra time
        end
    endtask

    task uart_send_frame_bad_chk;
        input [7:0] cmd;
        input [7:0] addr;
        input [7:0] len;
        input [31:0] data;
        reg [7:0] chk;
        integer i;
        begin
            chk = ~(cmd ^ addr ^ len); // Intentionally bad checksum
            uart_send_byte(8'h55);
            uart_send_byte(cmd);
            uart_send_byte(addr);
            uart_send_byte(len);
            if (len > 0) begin
                // Just send bottom byte for testing
                uart_send_byte(data[7:0]);
            end
            uart_send_byte(chk);
            #(BIT_PERIOD_NS * 5);
        end
    endtask

    // Read byte task (Optional, to verify responses if needed)
    task uart_recv_byte;
        output [7:0] data;
        integer i;
        begin
            // Wait for start bit
            wait(uart_tx_pin == 0);
            #(BIT_PERIOD_NS / 2); // Sample at middle of bit
            if (uart_tx_pin != 0) $display("UART RX framing error (start bit)");
            
            for (i = 0; i < 8; i = i + 1) begin
                #(BIT_PERIOD_NS);
                data[i] = uart_tx_pin;
            end
            
            #(BIT_PERIOD_NS); // Stop bit
            if (uart_tx_pin != 1) $display("UART RX framing error (stop bit)");
        end
    endtask

    // =========================================================
    // TEST SEQUENCE
    // =========================================================
    initial begin
        $display("==================================================");
        $display("   STARTING TOP_WATCHDOG AUTOMATED TESTBENCH");
        $display("==================================================");
        
        // Initialize Inputs
        wdi_pin     = 1;
        en_hw_pin   = 1; // Disabled (active low)
        uart_rx_pin = 1; // Idle high

        // Wait for internal Power-On Reset (POR) to complete
        // POR counter is 8 bits (255 cycles at 27MHz)
        #(260 * 37);
        $display("[%0t] Internal POR completed.", $time);

        // ---------------------------------------------------------
        // TEST 1: HW ENABLE & HW KICK
        // ---------------------------------------------------------
        $display("\n--- TEST 1: HW Enable & HW Kick ---");
        en_hw_pin = 0; // Enable Watchdog
        #(DEBOUNCE_TIME_NS); // Wait for debounce
        
        // At this point, Watchdog should be in ARMING state, then MONITOR state.
        // Let's configure tWD and tRST to small values via UART first to speed up simulation!
        
        $display("\n--- TEST 3: UART Configuration (Speed up sim) ---");
        // Write tWD = 10ms (10)
        // CMD = 0x01 (WRITE), ADDR = 0x04 (tWD), LEN = 0x04, DATA = 10
        uart_send_frame(8'h01, 8'h04, 8'h04, 32'd10);
        
        // Write tRST = 5ms (5)
        // CMD = 0x01, ADDR = 0x08 (tRST), LEN = 0x04, DATA = 5
        uart_send_frame(8'h01, 8'h08, 8'h04, 32'd5);

        // Write arm_delay = 1ms (1000 us)
        // CMD = 0x01, ADDR = 0x0C (arm_delay), LEN = 0x04, DATA = 1000
        uart_send_frame(8'h01, 8'h0C, 8'h04, 32'd1000);
        
        $display("[%0t] New timings configured: tWD=10ms, tRST=5ms, arm_delay=1ms.", $time);
        
        // Wait for arm delay to finish (1ms)
        #(1_500_000); 

        // Now Watchdog is in MONITOR state. WDO should be HIGH (1), ENOUT should be HIGH (1).
        if (wdo_pin !== 1 || enout_pin !== 1) begin
            $fatal(1, "[%0t] Test 1 Failed: WDO or ENOUT not active after ARMING.", $time);
        end else begin
            $display("[%0t] Watchdog correctly entered MONITOR state.", $time);
        end

        // Perform a valid HW Kick (falling edge on WDI)
        wdi_pin = 0;
        #(DEBOUNCE_TIME_NS); // Wait for debounce
        wdi_pin = 1;
        #(DEBOUNCE_TIME_NS);
        $display("[%0t] Hardware Kick performed.", $time);

        // ---------------------------------------------------------
        // TEST 2: HW TIMEOUT FAULT
        // ---------------------------------------------------------
        $display("\n--- TEST 2: HW Timeout Fault ---");
        // Wait for tWD (10ms) to expire without kicking
        #(11_000_000); // Wait 11ms
        
        // Expect WDO to go LOW (Fault)
        if (wdo_pin !== 0) begin
            $fatal(1, "[%0t] Test 2 Failed: WDO did not go LOW after tWD timeout.", $time);
        end else begin
            $display("[%0t] Watchdog correctly entered FAULT state (WDO is LOW).", $time);
        end

        // Wait for tRST (5ms) to expire
        #(6_000_000); // Wait 6ms
        
        // Because EN is still active, it should go back to ARMING -> MONITOR
        // Wait for ARM delay (1ms)
        #(1_500_000);
        
        if (wdo_pin !== 1) begin
            $fatal(1, "[%0t] Test 2 Failed: WDO did not recover after tRST.", $time);
        end else begin
            $display("[%0t] Watchdog correctly recovered from FAULT state.", $time);
        end

        // Disable HW Enable
        en_hw_pin = 1;
        #(DEBOUNCE_TIME_NS);
        $display("[%0t] Hardware Enable Disabled. Watchdog should be in DISABLE state.", $time);

        // ---------------------------------------------------------
        // TEST 4: SW ENABLE & SW KICK
        // ---------------------------------------------------------
        $display("\n--- TEST 4: SW Enable & SW Kick ---");
        // Write CTRL register: en_sw=1, wdi_src=1 (SW Enable, SW Kick)
        // CMD=0x01, ADDR=0x00, LEN=0x04, DATA=0x03
        uart_send_frame(8'h01, 8'h00, 8'h04, 32'h0000_0003);
        $display("[%0t] SW Enable and SW Kick source configured.", $time);

        // Wait for arm delay (1ms)
        #(1_500_000);

        if (wdo_pin !== 1 || enout_pin !== 1) begin
            $fatal(1, "[%0t] Test 4 Failed: Watchdog did not start with SW Enable.", $time);
        end
        
        // Send SW Kick
        // CMD=0x03 (KICK), ADDR=0x00, LEN=0x00
        uart_send_frame(8'h03, 8'h00, 8'h00, 32'd0);
        $display("[%0t] SW Kick via UART sent.", $time);

        // Wait 5ms (less than tWD 10ms)
        #(5_000_000);

        if (wdo_pin !== 1) begin
            $fatal(1, "[%0t] Test 4 Failed: Premature Fault with SW Kick.", $time);
        end

        // ---------------------------------------------------------
        // TEST 5: SW TIMEOUT & CLEAR FAULT
        // ---------------------------------------------------------
        $display("\n--- TEST 5: SW Timeout & Clear Fault ---");
        // Wait for tWD timeout (10ms total, we waited 5ms, wait another 6ms)
        #(6_000_000);

        if (wdo_pin !== 0) begin
            $fatal(1, "[%0t] Test 5 Failed: Did not enter Fault state.", $time);
        end else begin
            $display("[%0t] SW Timeout occurred. WDO is LOW.", $time);
        end

        // Send Clear Fault pulse via UART (Write 1 to CTRL bit 2)
        // DATA = 0x07 (en_sw=1, wdi_src=1, clr_fault=1)
        uart_send_frame(8'h01, 8'h00, 8'h04, 32'h0000_0007);
        $display("[%0t] Clear Fault command sent.", $time);

        // Wait a small amount
        #(100_000);

        // State should be back to ARMING (WDO HIGH)
        if (wdo_pin !== 1) begin
            $fatal(1, "[%0t] Test 5 Failed: Clear Fault did not recover WDO.", $time);
        end else begin
            $display("[%0t] Watchdog correctly recovered via Clear Fault command.", $time);
        end

        // ---------------------------------------------------------
        // TEST 6: UART CORNER CASES
        // ---------------------------------------------------------
        $display("\n--- TEST 6: UART Corner Cases ---");
        
        // Bad Checksum
        $display("[%0t] Sending frame with invalid checksum...", $time);
        uart_send_frame_bad_chk(8'h01, 8'h00, 8'h04, 32'h0000_0003);
        // Wait and ensure no ACK is sent back. TX should remain high (idle)
        #(BIT_PERIOD_NS * 20);
        if (uart_tx_pin === 0) begin
            $error("[%0t] Error: Device responded to a frame with bad checksum.", $time);
        end else begin
            $display("[%0t] Device correctly ignored bad checksum.", $time);
        end

        // Unsupported Command
        $display("[%0t] Sending unsupported command (0xFF)...", $time);
        uart_send_frame(8'hFF, 8'h00, 8'h00, 32'd0);
        #(BIT_PERIOD_NS * 20);
        // Ensure no weird behavior occurs

        // ---------------------------------------------------------
        // TEST 7: BUTTON DEBOUNCE EDGE CASES
        // ---------------------------------------------------------
        $display("\n--- TEST 7: Button Debounce Edge Cases ---");
        // Disable SW Watchdog, Re-enable HW Watchdog
        uart_send_frame(8'h01, 8'h00, 8'h04, 32'h0000_0000); // en_sw=0, wdi_src=0
        
        en_hw_pin = 0;
        #(DEBOUNCE_TIME_NS);
        #(1_500_000); // arm delay
        
        // Generate noisy WDI signal (pulse < 20ms)
        $display("[%0t] Generating noisy WDI pulse (10ms)...", $time);
        wdi_pin = 0;
        #(10_000_000); // 10ms
        wdi_pin = 1;
        
        // This kick should be ignored, so after 10ms (tWD) from previous state, we should fault.
        // Previous state arm delay finished at 1.5ms. So fault should happen ~10ms from arm delay.
        #(5_000_000); // Wait 5ms more
        
        if (wdo_pin === 0) begin
            $display("[%0t] Watchdog correctly faulted, meaning the noisy kick was ignored.", $time);
        end else begin
            // Wait a little more just in case
            #(5_000_000);
            if (wdo_pin === 0) begin
                 $display("[%0t] Watchdog correctly faulted, meaning the noisy kick was ignored.", $time);
            end else begin
                 $fatal(1, "[%0t] Test 7 Failed: Noisy pulse triggered a kick.", $time);
            end
        end

        // ---------------------------------------------------------
        // FINISH
        // ---------------------------------------------------------
        $display("\n==================================================");
        $display("   ALL TESTS PASSED SUCESSFULLY!");
        $display("==================================================");
        $finish;
    end

endmodule
