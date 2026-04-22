`timescale 1ns / 1ps

// =============================================================================
// Module: watchdog_core
// Mô tả: Trái tim FSM + Timers của hệ thống Watchdog Monitor.
//         Mô phỏng hành vi IC TPS3431 trên FPGA.
//
// FSM gồm 4 trạng thái:
//   DISABLE  -> EN=0: WDO=1 (OK), ENOUT=0, mọi kick bị bỏ qua.
//   ARMING   -> EN vừa lên 1: đếm arm_delay_us, kick vẫn bị bỏ qua.
//   MONITOR  -> Giám sát bình thường: đếm tWD_ms, kick hợp lệ reset timer.
//   FAULT    -> Timeout xảy ra: WDO=0 (lỗi), đếm tRST_ms rồi tự nhả.
//              CLR_FAULT có thể nhả WDO ngay lập tức.
//
// Nguồn EN:  en_hw (nút S2 phần cứng) OR en_sw (bit CTRL[0] từ UART)
// Nguồn WDI: Phụ thuộc bit wdi_src (CTRL[1]):
//             wdi_src=0 -> Chỉ nhận kick từ S1 (phần cứng)
//             wdi_src=1 -> Chỉ nhận kick từ UART (phần mềm)
// =============================================================================

module watchdog_core #(
    parameter CLK_FREQ = 50_000_000  // Tần số xung nhịp hệ thống (Hz)
)(
    input  wire        clk,
    input  wire        reset_n,

    // -------------------------------------------------------
    // Tín hiệu phần cứng (từ sync_debounce, đã active-high)
    // -------------------------------------------------------
    input  wire        en_hw,           // EN từ nút S2 (1 = bật, 0 = tắt)
    input  wire        wdi_falling_hw,  // Xung cạnh xuống WDI từ S1 (1 chu kỳ)

    // -------------------------------------------------------
    // Tín hiệu phần mềm (từ uart_frame_parser)
    // -------------------------------------------------------
    input  wire        uart_kick_pulse, // Xung kick WDI từ lệnh UART CMD 0x03

    // -------------------------------------------------------
    // Cấu hình từ regfile
    // -------------------------------------------------------
    input  wire        en_sw,           // Software Enable (CTRL bit 0)
    input  wire        wdi_src,         // Nguồn WDI: 0=HW only, 1=SW only
    input  wire        clr_fault,       // Xung xóa lỗi (CTRL bit 2, W1C)
    input  wire [31:0] tWD_ms,          // Thời gian timeout Watchdog (ms)
    input  wire [31:0] tRST_ms,         // Thời gian giữ lỗi WDO (ms)
    input  wire [15:0] arm_delay_us,    // Thời gian chờ khởi động (us)

    // -------------------------------------------------------
    // Trạng thái xuất ra regfile (STATUS register)
    // -------------------------------------------------------
    output wire        en_effective,    // Watchdog đã qua giai đoạn Arming
    output wire        fault_active,    // Đang ở trạng thái Fault
    output wire        enout_state,     // Giá trị chân ENOUT hiện tại
    output wire        wdo_state,       // Giá trị chân WDO hiện tại
    output reg         last_kick_src,   // Nguồn kick cuối: 0=HW/S1, 1=SW/UART

    // -------------------------------------------------------
    // Ngõ ra vật lý tới chân FPGA
    // -------------------------------------------------------
    output reg         wdo_pin,         // WDO: 1=OK (Hi-Z), 0=Fault (kéo thấp)
    output reg         enout_pin        // ENOUT: 1=Hệ thống hoạt động, 0=Vô hiệu
);

    // =========================================================================
    // KHAI BÁO TRẠNG THÁI FSM
    // =========================================================================
    localparam S_DISABLE = 2'd0;  // Watchdog tắt
    localparam S_ARMING  = 2'd1;  // Đang chờ khởi động (arm_delay)
    localparam S_MONITOR = 2'd2;  // Giám sát bình thường (đếm tWD)
    localparam S_FAULT   = 2'd3;  // Phát hiện lỗi (WDO kéo thấp, đếm tRST)

    reg [1:0] state;

    // =========================================================================
    // BỘ TẠO XUNG 1 MICROSECOND (us_tick)
    // =========================================================================
    // Chia xung nhịp hệ thống xuống tạo 1 xung mỗi 1us.
    // Ví dụ: CLK_FREQ = 50MHz -> US_DIV = 50 chu kỳ/us.
    localparam US_DIV = CLK_FREQ / 1_000_000;

    reg [7:0] us_cnt;   // 8-bit đủ cho US_DIV tối đa 255 (hỗ trợ đến 255MHz)
    reg       us_tick;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            us_cnt  <= 8'd0;
            us_tick <= 1'b0;
        end else begin
            if (us_cnt == US_DIV - 1) begin
                us_cnt  <= 8'd0;
                us_tick <= 1'b1;
            end else begin
                us_cnt  <= us_cnt + 1'b1;
                us_tick <= 1'b0;
            end
        end
    end

    // =========================================================================
    // BỘ TẠO XUNG 1 MILLISECOND (ms_tick)
    // =========================================================================
    // Đếm 1000 xung us_tick để tạo ra 1 xung mỗi 1ms.
    reg [9:0] ms_sub_cnt;  // 10-bit đếm từ 0 đến 999
    reg       ms_tick;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ms_sub_cnt <= 10'd0;
            ms_tick    <= 1'b0;
        end else begin
            ms_tick <= 1'b0;  // Mặc định hạ xung
            if (us_tick) begin
                if (ms_sub_cnt == 10'd999) begin
                    ms_sub_cnt <= 10'd0;
                    ms_tick    <= 1'b1;
                end else begin
                    ms_sub_cnt <= ms_sub_cnt + 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // LOGIC KẾT HỢP TÍN HIỆU ENABLE & KICK
    // =========================================================================
    // EN kích hoạt từ phần cứng (S2) HOẶC phần mềm (CTRL bit 0)
    wire en_combined = en_hw | en_sw;

    // Kick hợp lệ phụ thuộc vào bit wdi_src:
    //   wdi_src = 0 -> Chỉ chấp nhận kick từ nút S1 (phần cứng)
    //   wdi_src = 1 -> Chỉ chấp nhận kick từ UART (phần mềm)
    wire kick_valid = (wdi_src == 1'b0) ? wdi_falling_hw : uart_kick_pulse;

    // =========================================================================
    // BỘ ĐẾM TIMER ĐA NĂNG (dùng chung cho arm_delay, tWD, tRST)
    // =========================================================================
    reg [31:0] timer_cnt;

    // =========================================================================
    // MÁY TRẠNG THÁI CHÍNH (FSM)
    // =========================================================================
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Khởi động an toàn: Watchdog ở trạng thái vô hiệu hóa
            state         <= S_DISABLE;
            timer_cnt     <= 32'd0;
            wdo_pin       <= 1'b1;   // WDO mặc định cao (không lỗi)
            enout_pin     <= 1'b0;   // ENOUT mặc định thấp (chưa hoạt động)
            last_kick_src <= 1'b0;
        end else begin

            // =============================================================
            // KIỂM TRA TOÀN CỤC: Nếu EN bị hạ xuống 0 ở BẤT KỲ lúc nào
            //   -> Lập tức trở về trạng thái DISABLE
            // Đây là hành vi ưu tiên cao nhất (override mọi state khác)
            // =============================================================
            if (!en_combined) begin
                state     <= S_DISABLE;
                timer_cnt <= 32'd0;
                wdo_pin   <= 1'b1;   // Nhả WDO
                enout_pin <= 1'b0;   // Tắt ENOUT
            end else begin
                // EN đang bật (en_combined = 1)
                case (state)

                    // -------------------------------------------------
                    // DISABLE: Watchdog đang tắt, chờ EN lên 1
                    // -------------------------------------------------
                    S_DISABLE: begin
                        wdo_pin   <= 1'b1;
                        enout_pin <= 1'b0;
                        timer_cnt <= 32'd0;
                        // EN vừa lên 1 (vì ta đã ở trong else của en_combined)
                        // -> Chuyển sang ARMING
                        state <= S_ARMING;
                    end

                    // -------------------------------------------------
                    // ARMING: Đếm arm_delay_us, phớt lờ mọi WDI kick
                    // Sau khi hết arm_delay -> bật ENOUT, sang MONITOR
                    // -------------------------------------------------
                    S_ARMING: begin
                        if (us_tick) begin
                            if (timer_cnt >= {16'd0, arm_delay_us} - 1) begin
                                // Hết thời gian chờ khởi động
                                timer_cnt <= 32'd0;
                                enout_pin <= 1'b1;   // Bật ENOUT báo hệ thống sẵn sàng
                                state     <= S_MONITOR;
                            end else begin
                                timer_cnt <= timer_cnt + 1'b1;
                            end
                        end
                        // Mọi kick trong giai đoạn này đều bị bỏ qua (không xử lý)
                    end

                    // -------------------------------------------------
                    // MONITOR: Giám sát bình thường
                    //   - Kick hợp lệ -> reset timer về 0
                    //   - Hết tWD_ms mà không có kick -> FAULT
                    // -------------------------------------------------
                    S_MONITOR: begin
                        if (kick_valid) begin
                            // Nhận được kick hợp lệ -> Reset bộ đếm timeout
                            timer_cnt     <= 32'd0;
                            // Ghi nhận nguồn kick cuối cùng
                            last_kick_src <= wdi_src;  // 0=HW, 1=SW
                        end else if (ms_tick) begin
                            if (timer_cnt >= tWD_ms - 1) begin
                                // TIMEOUT! Không nhận kick trong thời gian cho phép
                                timer_cnt <= 32'd0;
                                wdo_pin   <= 1'b0;   // Kéo WDO xuống thấp (báo lỗi)
                                state     <= S_FAULT;
                            end else begin
                                timer_cnt <= timer_cnt + 1'b1;
                            end
                        end
                    end

                    // -------------------------------------------------
                    // FAULT: WDO đang ở mức thấp (lỗi)
                    //   - CLR_FAULT -> nhả WDO ngay lập tức
                    //   - Hết tRST_ms -> tự động nhả WDO, quay về MONITOR
                    //   - Mọi kick bị bỏ qua
                    // -------------------------------------------------
                    S_FAULT: begin
                        if (clr_fault) begin
                            // Nhận lệnh xóa lỗi từ UART (ghi 1 vào CTRL bit 2)
                            wdo_pin   <= 1'b1;   // Nhả WDO ngay
                            timer_cnt <= 32'd0;
                            state     <= S_MONITOR;
                        end else if (ms_tick) begin
                            if (timer_cnt >= tRST_ms - 1) begin
                                // Hết thời gian giữ lỗi tRST
                                wdo_pin   <= 1'b1;   // Nhả WDO
                                timer_cnt <= 32'd0;
                                state     <= S_MONITOR;
                            end else begin
                                timer_cnt <= timer_cnt + 1'b1;
                            end
                        end
                        // Kick bị bỏ qua hoàn toàn khi đang Fault
                    end

                    default: state <= S_DISABLE;
                endcase
            end
        end
    end

    // =========================================================================
    // GÁN TÍN HIỆU TRẠNG THÁI CHO REGFILE (STATUS REGISTER)
    // =========================================================================
    // en_effective = 1 khi Watchdog đã qua giai đoạn Arming (đang Monitor hoặc Fault)
    assign en_effective = (state == S_MONITOR) || (state == S_FAULT);

    // fault_active = 1 khi đang ở trạng thái lỗi
    assign fault_active = (state == S_FAULT);

    // Trạng thái thực tế của các chân ngõ ra
    assign enout_state = enout_pin;
    assign wdo_state   = wdo_pin;

endmodule
