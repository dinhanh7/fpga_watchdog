`timescale 1ns / 1ps

module tb_uart_frame();

    // Clock and Reset
    reg clk;
    reg rst_n;

    // Giao tiếp uart_rx (giả lập)
    reg [7:0] rx_data;
    reg rx_done;

    // Giao tiếp uart_tx
    wire [7:0] tx_data;
    wire tx_en;
    reg tx_busy;

    // Giao tiếp regfile
    wire [7:0]  reg_addr;
    wire        reg_we;
    wire        reg_re;
    wire [31:0] reg_wdata;
    wire [31:0] reg_rdata;

    // Khác
    wire uart_kick_pulse;
    wire clr_fault;

    // 1. Khởi tạo UART Frame Parser
    uart_frame_parser u_parser (
        .clk(clk),
        .rst_n(rst_n),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .tx_data(tx_data),
        .tx_en(tx_en),
        .tx_busy(tx_busy),
        .reg_addr(reg_addr),
        .reg_we(reg_we),
        .reg_re(reg_re),
        .reg_wdata(reg_wdata),
        .reg_rdata(reg_rdata),
        .uart_kick_pulse(uart_kick_pulse)
    );

    // 2. Khởi tạo Regfile để test Read/Write
    regfile u_regfile (
        .clk(clk),
        .rst_n(rst_n),
        .addr(reg_addr),
        .we(reg_we),
        .re(reg_re),
        .wdata(reg_wdata),
        .rdata(reg_rdata),
        .en_sw(),
        .wdi_src(),
        .clr_fault(clr_fault),
        .tWD_ms(),
        .tRST_ms(),
        .arm_delay_us(),
        // Giả lập trạng thái từ Watchdog Core
        .en_effective(1'b1),
        .fault_active(1'b0),
        .enout_state(1'b1),
        .wdo_state(1'b1),
        .last_kick_src(1'b0)
    );

    // Tạo Clock (50MHz)
    always #10 clk = ~clk;

    // Task gửi 1 byte giả lập cờ rx_done từ uart_rx
    task send_rx_byte(input [7:0] data);
        begin
            @(posedge clk);
            rx_data = data;
            rx_done = 1'b1;
            @(posedge clk);
            rx_done = 1'b0;
            #100; // Giãn cách giữa các byte một chút
        end
    endtask

    // Task gửi nguyên 1 khung ghi (WRITE) tự động tính Checksum
    task send_write_frame(
        input [7:0] addr, 
        input [31:0] data, 
        input [7:0] len
    );
        reg [7:0] chk;
        begin
            chk = 8'h01 ^ addr ^ len;
            send_rx_byte(8'h55); // 0x55
            send_rx_byte(8'h01); // CMD = 0x01 (WRITE)
            send_rx_byte(addr);  // ADDR
            send_rx_byte(len);   // LEN

            if (len == 4) begin
                send_rx_byte(data[31:24]); chk = chk ^ data[31:24];
                send_rx_byte(data[23:16]); chk = chk ^ data[23:16];
                send_rx_byte(data[15:8]);  chk = chk ^ data[15:8];
                send_rx_byte(data[7:0]);   chk = chk ^ data[7:0];
            end else if (len == 2) begin
                send_rx_byte(data[15:8]);  chk = chk ^ data[15:8];
                send_rx_byte(data[7:0]);   chk = chk ^ data[7:0];
            end else if (len == 1) begin
                send_rx_byte(data[7:0]);   chk = chk ^ data[7:0];
            end
            
            send_rx_byte(chk); // CHK
        end
    endtask

    // Task gửi nguyên 1 khung không kèm DATA (READ, KICK, STATUS)
    task send_cmd_frame(input [7:0] cmd, input [7:0] addr);
        reg [7:0] chk;
        begin
            chk = cmd ^ addr ^ 8'h00;
            send_rx_byte(8'h55); // 0x55
            send_rx_byte(cmd);   // CMD
            send_rx_byte(addr);  // ADDR
            send_rx_byte(8'h00); // LEN = 0
            send_rx_byte(chk);   // CHK
        end
    endtask

    // Giả lập Module uart_tx xử lý và đẩy dữ liệu
    always @(posedge clk) begin
        if (tx_en && !tx_busy) begin
            tx_busy <= 1'b1;
            // Thời gian giả lập uart_tx đang phát dữ liệu (tùy baud rate)
            #1000;
            @(posedge clk);
            tx_busy <= 1'b0;
        end
    end

    // Giám sát dữ liệu module Parser trả về (TX Monitor)
    always @(posedge clk) begin
        if (tx_en) begin
            $display("[%0t] UART TX: Gửi byte 0x%h", $time, tx_data);
        end
    end

    // Test Sequence
    initial begin
        // Khởi tạo
        clk = 0;
        rst_n = 0;
        rx_data = 0;
        rx_done = 0;
        tx_busy = 0;

        // Bắt đầu nhả Reset
        #100;
        rst_n = 1;
        #200;

        $display("\n--- KỊCH BẢN 1: TEST LỆNH WRITE ---");
        // Ghi vào thanh ghi tWD_ms (Địa chỉ 0x04) giá trị 0x12345678 (4 byte)
        send_write_frame(8'h04, 32'h12345678, 8'd4);
        #20000; // Đợi quá trình ghi và phản hồi ACK

        $display("\n--- KỊCH BẢN 2: TEST LỆNH READ ---");
        // Đọc lại giá trị từ thanh ghi tWD_ms (Địa chỉ 0x04)
        send_cmd_frame(8'h02, 8'h04);
        #20000; // Đợi UART Parser đọc regfile và gửi trả 9 byte

        $display("\n--- KỊCH BẢN 3: TEST LỆNH KICK ---");
        // Gửi lệnh KICK (Địa chỉ không quan trọng, len = 0)
        send_cmd_frame(8'h03, 8'h00);
        #20000;

        $display("\n--- KỊCH BẢN 4: TEST LỆNH GET STATUS ---");
        // Gửi lệnh GET STATUS (Mã 0x04)
        send_cmd_frame(8'h04, 8'h00);
        #20000;

        $display("\n--- KỊCH BẢN 5: TEST CHECKSUM SAI ---");
        // Chủ động gửi gói tin với byte cuối cùng là Checksum sai (hủy gói)
        send_rx_byte(8'h55);
        send_rx_byte(8'h03);
        send_rx_byte(8'h00);
        send_rx_byte(8'h00);
        send_rx_byte(8'hFF); // CHK sai (đúng là 0x03)
        #10000;
        
        $display("\n--- TEST KẾT THÚC ---");
        $finish;
    end

endmodule
