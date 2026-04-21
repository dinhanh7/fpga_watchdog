`timescale 1ns / 1ps

module uart_rx #(
    parameter CLK_FREQ  = 50_000_000, 
    parameter BAUD_RATE = 115200      
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,             // Chân nhận tín hiệu UART RX
    output reg [7:0]  data_out,       // Dữ liệu 8-bit nhận được
    output reg        rx_done         // Xung báo hiệu đã nhận xong 1 byte
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam S_IDLE  = 2'b00;
    localparam S_START = 2'b01;
    localparam S_DATA  = 2'b10;
    localparam S_STOP  = 2'b11;

    reg [1:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;

    // ----- Bộ đồng bộ (2-FF Synchronizer) -----
    // Tín hiệu RX đi từ clock domain khác vào, cần đồng bộ để tránh metastability
    reg rx_sync1, rx_sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx;
            rx_sync2 <= rx_sync1;
        end
    end

    // ----- FSM Nhận dữ liệu -----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_count <= 0;
            bit_index <= 0;
            data_out  <= 8'd0;
            rx_done   <= 1'b0;
        end else begin
            rx_done <= 1'b0; // Mặc định là 0, chỉ tạo xung 1 chu kỳ khi nhận xong

            case (state)
                S_IDLE: begin
                    clk_count <= 0;
                    bit_index <= 0;
                    if (rx_sync2 == 1'b0) begin // Phát hiện Start bit (kéo xuống 0)
                        state <= S_START;
                    end
                end
                
                S_START: begin
                    // Đợi đến giữa chu kỳ của Start bit
                    if (clk_count == (CLKS_PER_BIT / 2) - 1) begin
                        if (rx_sync2 == 1'b0) begin // Xác nhận lại đúng là Start bit (lọc nhiễu)
                            clk_count <= 0;
                            state     <= S_DATA;
                        end else begin
                            state <= S_IDLE; // Nhiễu gai (glitch), quay lại IDLE
                        end
                    end else begin
                        clk_count <= clk_count + 1;
                    end
                end
                
                S_DATA: begin
                    // Đợi trọn 1 chu kỳ bit để lấy mẫu ở chính giữa các Data bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 0;
                        data_out[bit_index] <= rx_sync2; // Lưu bit thu được vào thanh ghi
                        
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1;
                        end else begin
                            bit_index <= 0;
                            state     <= S_STOP;
                        end
                    end
                end
                
                S_STOP: begin
                    // Đợi 1 chu kỳ bit cho Stop bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1;
                    end else begin
                        clk_count <= 0;
                        rx_done   <= 1'b1; // Kích hoạt cờ hoàn thành báo hiệu đã có data hợp lệ
                        state     <= S_IDLE;
                    end
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule