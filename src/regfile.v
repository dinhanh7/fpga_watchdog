module regfile (
    input  wire        clk,
    input  wire        rst_n,

    // Giao tiep voi Parser (UART)
    input  wire        i_wr_en,
    input  wire [7:0]  i_addr,
    input  wire [31:0] i_wr_data,
    output reg  [31:0] o_rd_data,

    // Tin hieu trang thai tu Watchdog Core (vao thanh ghi STATUS 0x10)
    input  wire        i_en_effective,
    input  wire        i_fault_active,
    input  wire        i_enout_state,
    input  wire        i_wdo_state,
    input  wire        i_last_kick_src,

    // Cau hinh xuat ra Watchdog Core
    output reg         o_wd_en,       // CTRL bit 0
    output reg         o_wdi_src,     // CTRL bit 1 (0: S1, 1: UART)
    output reg         o_clr_fault,   // CTRL bit 2 (Xung xoa loi)
    output reg  [31:0] o_twd_ms,
    output reg  [31:0] o_trst_ms,
    output reg  [15:0] o_arm_delay_us // Do rong 16-bit
);

    // --- Logic Ghi (Write Path) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_wd_en        <= 1'b0;
            o_wdi_src      <= 1'b0;
            o_clr_fault    <= 1'b0; 
            o_twd_ms       <= 32'd1600;
            o_trst_ms      <= 32'd200;
            o_arm_delay_us <= 16'd150;
        end else begin
            // Co che Write-1-to-clear: Tu dong don bit clr_fault ve 0 de tao xung 1 chu ky
            o_clr_fault <= 1'b0; 
            
            if (i_wr_en) begin
                case (i_addr)
                    8'h00: begin 
                        o_wd_en     <= i_wr_data[0];
                        o_wdi_src   <= i_wr_data[1];
                        o_clr_fault <= i_wr_data[2];
                    end
                    8'h04: o_twd_ms       <= i_wr_data;
                    8'h08: o_trst_ms      <= i_wr_data;
                    8'h0C: o_arm_delay_us <= i_wr_data[15:0];
                    default: ; 
                endcase
            end
        end
    end

    // --- Logic Doc (Read Path) ---
    always @(posedge clk) begin
        case (i_addr)
            // Doc CTRL: clr_fault luon doc la 0 (chuan Write-to-clear)
            8'h00: o_rd_data = {29'd0, 1'b0, o_wdi_src, o_wd_en}; 
            8'h04: o_rd_data = o_twd_ms;
            8'h08: o_rd_data = o_trst_ms;
            8'h0C: o_rd_data = {16'd0, o_arm_delay_us};
            8'h10: o_rd_data = {27'd0, i_last_kick_src, i_wdo_state, 
                                i_enout_state, i_fault_active, i_en_effective};
            default: o_rd_data = 32'hDEADBEEF; // Bao hieu dia chi sai
        endcase
    end
endmodule