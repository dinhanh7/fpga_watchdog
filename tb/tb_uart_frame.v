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
    wire wdi_src;

    // Giả lập Module uart_tx xử lý và đẩy dữ liệu
    reg mock_tx_done;

    always @(posedge clk) begin
        if (rst_n) begin
            if (tx_en && !tx_busy) begin
                tx_busy <= 1'b1;
                // Thời gian giả lập uart_tx đang phát dữ liệu (tùy baud rate)
                #1000;
                @(posedge clk);
                tx_busy <= 1'b0;
                mock_tx_done <= 1'b1;
            end else if (mock_tx_done) begin
                // Pulsed only 1 cycle
                mock_tx_done <= 1'b0;
            end
        end else begin
            tx_busy <= 0;
            mock_tx_done <= 0;
        end
    end

    // 1. Khởi tạo UART Frame Parser
    uart_frame_parser u_parser (
        .clk(clk),
        .rst_n(rst_n),
        .rx_data_i(rx_data),
        .rx_done_i(rx_done),
        .tx_data_o(tx_data),
        .tx_en_o(tx_en),
        .tx_busy_i(tx_busy),
        .tx_done_i(mock_tx_done), // Mock tx_done 1-cycle pulse
        .reg_addr_o(reg_addr),
        .reg_we_o(reg_we),
        .reg_re_o(reg_re),
        .reg_wdata_o(reg_wdata),
        .reg_rdata_i(reg_rdata),
        .uart_kick_pulse_o(uart_kick_pulse),
        .wdi_src_i(wdi_src)
    );

    // 2. Khởi tạo Regfile để test Read/Write
    regfile u_regfile (
        .clk(clk),
        .rst_n(rst_n),
        .addr_i(reg_addr),
        .we_i(reg_we),
        .re_i(reg_re),
        .wdata_i(reg_wdata),
        .rdata_o(reg_rdata),
        .en_sw_o(),
        .wdi_src_o(wdi_src),
        .clr_fault_o(clr_fault),
        .tWD_ms_o(),
        .tRST_ms_o(),
        .arm_delay_us_o(),
        // Giả lập trạng thái từ Watchdog Core
        .en_effective_i(1'b1),
        .fault_active_i(1'b0),
        .enout_state_i(1'b1),
        .wdo_state_i(1'b1),
        .last_kick_src_i(1'b0)
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

    // (Moved mock_tx block to the top)
    
    // Giám sát dữ liệu module Parser trả về (TX Monitor)
    reg [7:0] tx_monitor_buf [0:15];
    integer tx_monitor_cnt;

    always @(posedge clk) begin
        if (tx_en) begin
            $display("[%0t] UART TX: Gửi byte 0x%h", $time, tx_data);
            tx_monitor_buf[tx_monitor_cnt] = tx_data;
            tx_monitor_cnt = tx_monitor_cnt + 1;
        end
    end

    // Bộ Check tự động cho TX Response
    task check_tx_response(input integer expected_len, input [7:0] expected_cmd, input [7:0] expected_addr);
        reg [7:0] calc_chk;
        integer i;
        begin
            if (tx_monitor_cnt !== expected_len) begin
                $display("--> [FAIL] Phản hồi sai số lượng byte! (Kỳ vọng: %0d, Nhận: %0d)", expected_len, tx_monitor_cnt);
            end else begin
                if (tx_monitor_buf[0] !== 8'h55) $display("--> [FAIL] Lỗi Start Byte: 0x%h", tx_monitor_buf[0]);
                if (tx_monitor_buf[1] !== expected_cmd) $display("--> [FAIL] Lỗi CMD: 0x%h", tx_monitor_buf[1]);
                if (tx_monitor_buf[2] !== expected_addr) $display("--> [FAIL] Lỗi ADDR: 0x%h", tx_monitor_buf[2]);
                
                calc_chk = 0;
                for (i=0; i<expected_len-1; i=i+1) calc_chk = calc_chk ^ tx_monitor_buf[i];
                if (tx_monitor_buf[expected_len-1] !== calc_chk) 
                    $display("--> [FAIL] Lỗi Checksum: Nhận 0x%h, Kỳ vọng 0x%h", tx_monitor_buf[expected_len-1], calc_chk);
                else if (tx_monitor_buf[0]==8'h55 && tx_monitor_buf[1]==expected_cmd && tx_monitor_buf[2]==expected_addr)
                    $display("--> [PASS] Phản hồi chính xác! (%0d bytes, CMD=0x%h, ADDR=0x%h)", expected_len, expected_cmd, expected_addr);
            end
            tx_monitor_cnt = 0; // Xóa buffer chuẩn bị cho frame tiếp theo
        end
    endtask

    // Test Sequence
    initial begin
        // Khởi tạo
        clk = 0;
        rst_n = 0;
        rx_data = 0;
        rx_done = 0;
        //tx_busy is handled in always block
        tx_monitor_cnt = 0;

        // Bắt đầu nhả Reset
        #100;
        rst_n = 1;
        #200;

        $display("\n--- KỊCH BẢN 1: TEST LỆNH WRITE THÔNG THƯỜNG ---");
        // Ghi vào thanh ghi tWD_ms (Địa chỉ 0x04) giá trị 0x12345678 (4 byte)
        send_write_frame(8'h04, 32'h12345678, 8'd4);
        #20000; 
        check_tx_response(5, 8'h01, 8'h04);

        $display("\n--- KỊCH BẢN 2: TEST LỆNH READ THÔNG THƯỜNG ---");
        // Đọc lại giá trị từ thanh ghi tWD_ms (Địa chỉ 0x04)
        send_cmd_frame(8'h02, 8'h04);
        #20000;
        // ACK của READ sẽ trả về 9 bytes (55, CMD, ADDR, LEN=4, DATAx4, CHK)
        check_tx_response(9, 8'h02, 8'h04);

        $display("\n--- KỊCH BẢN 3A: TEST LỆNH KICK KHI BỊ CẤM (wdi_src=0) ---");
        // Mặc định wdi_src = 0. Gửi KICK sẽ bị lờ đi.
        send_cmd_frame(8'h03, 8'h00);
        #20000;
        if (tx_monitor_cnt == 0) $display("--> [PASS] Không có phản hồi, lệnh KICK đã bị chặn an toàn.");
        else $display("--> [FAIL] Có phản hồi dù lệnh KICK bị chặn!");
        tx_monitor_cnt = 0;

        $display("\n--- KỊCH BẢN 3B: BẬT TÍNH NĂNG SW KICK (wdi_src=1) ---");
        // Ghi vào thanh ghi CTRL (Addr=0x00), Set Bit 1 (wdi_src) = 1
        send_write_frame(8'h00, 32'h00000002, 8'd4);
        #20000;
        check_tx_response(5, 8'h01, 8'h00);

        $display("\n--- KỊCH BẢN 3C: TEST LỆNH KICK KHI ĐƯỢC CHO PHÉP (wdi_src=1) ---");
        send_cmd_frame(8'h03, 8'h00);
        #20000;
        check_tx_response(5, 8'h03, 8'h00);

        $display("\n--- KỊCH BẢN 4: TEST LỆNH GET STATUS ---");
        // Gửi lệnh GET STATUS (Mã 0x04)
        send_cmd_frame(8'h04, 8'h00);
        #20000;
        // Firmware phản hồi địa chỉ STATUS là 0x10, trả về 9 bytes
        check_tx_response(9, 8'h04, 8'h10);

        $display("\n--- KỊCH BẢN 5: TEST CHECKSUM SAI ---");
        // Chủ động gửi gói tin với byte Checksum sai (hủy gói)
        send_rx_byte(8'h55);
        send_rx_byte(8'h03);
        send_rx_byte(8'h00);
        send_rx_byte(8'h00);
        send_rx_byte(8'hFF); // CHK sai (đáng lẽ phải là 0x03)
        #10000;
        if (tx_monitor_cnt == 0) $display("--> [PASS] Phớt lờ thành công frame bị sai checksum.");
        else $display("--> [FAIL] Vẫn lấy dữ liệu hoặc phản hồi frame lỗi!");
        tx_monitor_cnt = 0;

        $display("\n--- KỊCH BẢN 6: TEST NHIỄU RÁC (GARBAGE) TRƯỚC FRAME ---");
        // Gửi ngẫu nhiên các byte rác trước khi gửi Start Byte 0x55
        send_rx_byte(8'hAA);
        send_rx_byte(8'h12);
        send_rx_byte(8'h99); 
        // Vẫn gửi theo sau đó là 1 frame Write đúng
        send_write_frame(8'h0C, 32'h00000100, 8'd4); 
        #20000;
        check_tx_response(5, 8'h01, 8'h0C);

        $display("\n--- KỊCH BẢN 7: TEST INVALID CMD ---");
        // Khung tin hợp lệ về định dạng, nhưng chứa mã lệnh không ai biết 0x99
        send_cmd_frame(8'h99, 8'h00); 
        #20000;
        if (tx_monitor_cnt == 0) $display("--> [PASS] Đã xả bỏ lệnh rác an toàn.");
        else $display("--> [FAIL] FSM bị mắc kẹt / phản hồi sai lệnh rác!");
        tx_monitor_cnt = 0;

        $display("\n--- TEST KẾT THÚC ---");
        $finish;
    end

endmodule
