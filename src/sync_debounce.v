`timescale 1ns/1ns

module sync_debounce #(
    // Với Clock 50MHz (chu kỳ 20ns), 20ms = 1_000_000 chu kỳ.
    // Dùng parameter để dễ dàng thu nhỏ giá trị đếm khi chạy mô phỏng (Testbench).
    parameter DELAY_CYCLES = 20'd1_000_000 
)(
    input wire clk,
    input wire reset_n,      // Tích cực mức thấp (Active-low)
    input wire btn_in,       // Tín hiệu thô từ nút nhấn (Active-low)
    
    output reg btn_out,      // Tín hiệu nút nhấn đã được đồng bộ và lọc nhiễu
    output wire falling_edge // Xung đơn 1 chu kỳ khi phát hiện sườn xuống (rất hữu ích cho WDI kick)
);

    //---------------------------------------------------------
    // 1. Khối đồng bộ 2 tầng (2-FF Synchronizer)
    //---------------------------------------------------------
    reg sync1, sync2;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Nút nhấn có pull-up, mặc định không bấm là mức 1
            {sync2, sync1} <= 2'b11; 
        end else begin
            // Dịch bit tín hiệu vào qua 2 Flip-Flop
            {sync2, sync1} <= {sync1, btn_in};
        end
    end

    //---------------------------------------------------------
    // 2. Khối đếm thời gian lọc nhiễu (Debouncer)
    //---------------------------------------------------------
    reg [19:0] cnt;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            cnt <= 20'd0;
            btn_out <= 1'b1; // Mặc định chưa bấm
        end else begin
            // Nếu trạng thái nút đang đồng bộ giống với ngõ ra hiện tại
            // -> Trạng thái ổn định, reset bộ đếm
            if (sync2 == btn_out) begin
                cnt <= 20'd0;
            end else begin
                // Nếu có sự khác biệt (đang bấm hoặc đang nhả), tăng bộ đếm
                cnt <= cnt + 1'b1;
                // Nếu duy trì trạng thái mới đủ số chu kỳ DELAY_CYCLES
                if (cnt >= (DELAY_CYCLES - 1'b1)) begin
                    btn_out <= sync2; // Cập nhật ngõ ra
                    cnt <= 20'd0;
                end
            end
        end
    end

    //---------------------------------------------------------
    // 3. Khối phát hiện sườn xuống (Falling Edge Detector)
    //---------------------------------------------------------
    reg btn_out_prev;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            btn_out_prev <= 1'b1;
        end else begin
            btn_out_prev <= btn_out;
        end
    end
    
    // Tạo xung kích mức 1 khi trạng thái trước là 1 và trạng thái nay là 0
    assign falling_edge = (btn_out_prev == 1'b1) && (btn_out == 1'b0);

endmodule