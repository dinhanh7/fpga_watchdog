`timescale 1ns / 1ps

module tb_uart_frame;

    // Tham so cau hinh
    parameter CLK_FREQ  = 50_000_000; 
    parameter BAUD_RATE = 115200;
    localparam CLK_PERIOD = 20;

    // Tin hieu Testbench
    reg clk;
    reg rst_n;
    
    // Tin hieu TX (PC)
    reg        tx_start;
    reg  [7:0] tx_data_in;
    wire       tx_serial;
    wire       tx_busy;
    
    // Tin hieu RX (FPGA)
    wire [7:0] rx_data_out;
    wire       rx_done;

    // --- BIẾN ĐỂ TỰ ĐỘNG KIỂM TRA (AUTO-CHECKER) ---
    reg [7:0] expected_bytes [0:255]; // Mang luu data du kien (toi da 256 byte)
    integer head_ptr; // Con tro ghi (khi PC bat dau gui)
    integer tail_ptr; // Con tro doc (khi FPGA nhan duoc)

    // 1. Module UART TX (PC)
    uart_tx #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)
    ) u_pc_tx (
        .clk(clk), .rst_n(rst_n), .tx_start(tx_start),
        .data_in(tx_data_in), .tx(tx_serial), .tx_busy(tx_busy)
    );

    // 2. Module UART RX (FPGA)
    uart_rx #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE)
    ) u_fpga_rx (
        .clk(clk), .rst_n(rst_n), .rx(tx_serial),
        .data_out(rx_data_out), .rx_done(rx_done)
    );

    // 3. Tao xung clock
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2.0) clk = ~clk;
    end

    // ========================================================
    // TASK: Gui 1 byte qua UART TX va luu vao bo Kiem tra
    // ========================================================
    task send_byte;
        input [7:0] data;
        begin
            // 1. Luu byte nay vao mang de lat nua check
            expected_bytes[head_ptr] = data;
            head_ptr = head_ptr + 1;

            // 2. Thuc hien gui qua UART
            @(posedge clk);
            tx_data_in = data;
            tx_start   = 1;
            
            @(posedge clk);
            tx_start   = 0;
            
            @(negedge tx_busy);
            #20000; 
        end
    endtask

    // ========================================================
    // TASK: Dong goi va gui 1 Frame Watchdog hoan chinh
    // ========================================================
    task send_wd_frame;
        input [7:0] cmd;
        input [7:0] addr;
        input [7:0] len;
        input [31:0] data_32b; 
        reg [7:0] chk;
        begin
            $display("\n[%0t] === BAT DAU GUI FRAME: ADDR=0x%0h, LEN=%0d ===", $time, addr, len);
            
            send_byte(8'h55);
            chk = 8'h55;
            
            send_byte(cmd);
            chk = chk ^ cmd; 
            
            send_byte(addr);
            chk = chk ^ addr;
            
            send_byte(len);
            chk = chk ^ len;
            
            if (len >= 4) begin send_byte(data_32b[31:24]); chk = chk ^ data_32b[31:24]; end
            if (len >= 3) begin send_byte(data_32b[23:16]); chk = chk ^ data_32b[23:16]; end
            if (len >= 2) begin send_byte(data_32b[15:8]);  chk = chk ^ data_32b[15:8];  end
            if (len >= 1) begin send_byte(data_32b[7:0]);   chk = chk ^ data_32b[7:0];   end
            
            send_byte(chk);
            $display("[%0t] === PC DA GUI XONG FRAME. CHO FPGA NHAN... ===", $time);
        end
    endtask

    // ========================================================
    // MONITOR: Tu dong kiem tra du lieu RX nhan duoc
    // ========================================================
    always @(posedge rx_done) begin
        if (head_ptr > tail_ptr) begin
            if (rx_data_out == expected_bytes[tail_ptr]) begin
                $display("   -> [PASS] FPGA nhan dung: 0x%0h", rx_data_out);
            end else begin
                $display("   -> [FAIL] FPGA nhan SAI! Nhan: 0x%0h, Mong doi: 0x%0h", rx_data_out, expected_bytes[tail_ptr]);
            end
            tail_ptr = tail_ptr + 1;
        end else begin
            $display("   -> [ERROR] FPGA nhan duoc byte 0x%0h nhung PC khong he gui!", rx_data_out);
        end
    end

    // ========================================================
    // KICH BAN TEST CHINH
    // ========================================================
    initial begin
        // Khoi tao cac bien
        rst_n      = 0;
        tx_start   = 0;
        tx_data_in = 8'd0;
        head_ptr   = 0;
        tail_ptr   = 0;

        #100;
        rst_n = 1;
        #100;

        // CASE 1: Cau hinh tWD = 1600ms
        send_wd_frame(8'h01, 8'h04, 8'h04, 32'h00000640);
        #50000;

        // CASE 2: Enable Watchdog (CTRL = 1)
        send_wd_frame(8'h01, 8'h00, 8'h01, 32'h00000001);
        #50000;

        // TONG KET KET QUA TEST
        $display("\n=============================================");
        if (head_ptr == tail_ptr && head_ptr > 0)
            $display("KET LUAN: HOAN HAO! DA TRUYEN/NHAN CHINH XAC %0d BYTES.", head_ptr);
        else
            $display("KET LUAN: CO LOI! SO BYTE NHAN (%0d) KHAC SO BYTE GUI (%0d).", tail_ptr, head_ptr);
        $display("=============================================\n");

        $finish; 
    end

    initial begin
        $dumpfile("tb_uart_frame.vcd");
        $dumpvars(0, tb_uart_frame);
    end

endmodule