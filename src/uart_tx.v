module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD_RATE = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire tx_start,
    input wire [7:0] data_in,
    output reg tx,
    output reg tx_busy
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam S_IDLE  = 2'b00;
    localparam S_START = 2'b01;
    localparam S_DATA  = 2'b10;
    localparam S_STOP  = 2'b11;

    reg [1:0] state;
    reg [15:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            tx <= 1'b1;
            tx_busy <= 1'b0;
            clk_count <= 0;
            bit_index <= 0;
            data_reg <= 8'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    tx <= 1'b1;
                    tx_busy <= 1'b0;
                    clk_count <= 0;
                    bit_index <= 0;
                    if (tx_start) begin
                        data_reg <= data_in;
                        tx_busy <= 1'b1;
                        state <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0;
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count+1;
                    end else begin
                        clk_count <= 0;
                        state <= S_DATA;
                    end
                end

                S_DATA: begin
                    tx <= data_reg[bit_index];
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count+1;
                    end else begin 
                        clk_count <= 0;
                        if (bit_index < 7) begin
                            bit_index <= bit_index+1;
                        end else begin
                            bit_index <= 0;
                            state <= S_STOP;
                        end
                    end
                end

                S_STOP: begin
                    tx <= 1'b1;
                    if (clk_count < CLKS_PER_BIT-1) begin
                        clk_count <= clk_count+1;
                    end else begin
                        clk_count <= 0;
                        state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule