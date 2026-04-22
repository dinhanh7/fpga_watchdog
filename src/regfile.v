<<<<<<< HEAD
`timescale 1ns / 1ps

// =============================================================================
// Module: regfile
// Description: Register file to store system configuration and status.
//              Provides an interface for the UART parser to read/write registers,
//              and interfaces with the watchdog core to apply configurations
//              and read status flags.
// =============================================================================

=======
>>>>>>> 699ba6d65a31e1d6fbf9d9726b025265a61b2653
module regfile (
    input  wire        clk,
    input  wire        rst_n,

<<<<<<< HEAD
    // =========================================================
    // UART PARSER INTERFACE
    // =========================================================
    input  wire [7:0]  addr_i,        // Register address (0x00, 0x04, 0x08, 0x0C, 0x10)
    input  wire        we_i,          // Write Enable
    input  wire        re_i,          // Read Enable
    input  wire [31:0] wdata_i,       // Data to write from UART
    output reg  [31:0] rdata_o,       // Data to read back to UART

    // =========================================================
    // WATCHDOG CORE CONFIGURATION (OUTPUTS TO CORE)
    // =========================================================
    output wire        en_sw_o,       // Software Enable (CTRL Bit 0)
    output wire        wdi_src_o,     // WDI Source: 0=HW, 1=SW (CTRL Bit 1)
    output reg         clr_fault_o,   // Clear Fault Pulse (W1C on CTRL Bit 2)
    output wire [31:0] tWD_ms_o,      // Watchdog Timeout duration in ms
    output wire [31:0] tRST_ms_o,     // Reset holding duration in ms
    output wire [15:0] arm_delay_us_o,// Initial arming delay in us

    // =========================================================
    // WATCHDOG CORE STATUS (INPUTS FROM CORE)
    // =========================================================
    input  wire        en_effective_i,// Watchdog is actively monitoring/faulting
    input  wire        fault_active_i,// Watchdog is currently in FAULT state
    input  wire        enout_state_i, // Current physical state of ENOUT pin
    input  wire        wdo_state_i,   // Current physical state of WDO pin
    input  wire        last_kick_src_i// Source of last kick (0: HW/S1, 1: SW/UART)
);

    // =========================================================
    // INTERNAL REGISTERS
    // =========================================================
    reg [31:0] ctrl_reg;
    reg [31:0] twd_reg;
    reg [31:0] trst_reg;
    reg [15:0] arm_delay_reg;

    // Continuous assignments mapping internal registers to core configuration
    assign en_sw_o        = ctrl_reg[0];
    assign wdi_src_o      = ctrl_reg[1];
    assign tWD_ms_o       = twd_reg;
    assign tRST_ms_o      = trst_reg;
    assign arm_delay_us_o = arm_delay_reg;

    // Construct the STATUS register by concatenating status flags
    wire [31:0] status_reg = {
        27'd0, 
        last_kick_src_i,  // bit 4
        wdo_state_i,      // bit 3
        enout_state_i,    // bit 2
        fault_active_i,   // bit 1
        en_effective_i    // bit 0
    };

    // =========================================================
    // WRITE LOGIC (FROM UART PARSER)
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset to default TPS3431 parameters
            ctrl_reg      <= 32'd0;
            twd_reg       <= 32'd1600;  // Default tWD = 1600 ms
            trst_reg      <= 32'd200;   // Default tRST = 200 ms
            arm_delay_reg <= 16'd150;   // Default arming delay = 150 us
            clr_fault_o   <= 1'b0;
        end else begin
            clr_fault_o <= 1'b0; // Default clr_fault pulse to 0

            if (we_i) begin
                case (addr_i)
                    8'h00: begin // Write to CTRL register
                        // Only bits 0 and 1 (en_sw, wdi_src) are standard R/W
                        ctrl_reg[1:0] <= wdata_i[1:0];
                        
                        // Bit 2 is Write-1-to-Clear (W1C) for CLR_FAULT
                        if (wdata_i[2] == 1'b1) begin
                            clr_fault_o <= 1'b1; // Generate a 1-cycle pulse
                        end
                    end
                    
                    8'h04: twd_reg       <= wdata_i;        // tWD_ms
                    8'h08: trst_reg      <= wdata_i;        // tRST_ms
                    8'h0C: arm_delay_reg <= wdata_i[15:0];  // arm_delay_us
                    
                    // Address 0x10 is STATUS (Read-Only), ignore writes
=======
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
>>>>>>> 699ba6d65a31e1d6fbf9d9726b025265a61b2653
                    default: ; 
                endcase
            end
        end
    end

<<<<<<< HEAD
    // =========================================================
    // READ LOGIC (TO UART PARSER)
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_o <= 32'd0;
        end else begin
            if (re_i) begin
                case (addr_i)
                    8'h00: rdata_o <= ctrl_reg;
                    8'h04: rdata_o <= twd_reg;
                    8'h08: rdata_o <= trst_reg;
                    8'h0C: rdata_o <= {16'd0, arm_delay_reg}; // Zero-pad the upper 16 bits
                    8'h10: rdata_o <= status_reg;             // Read STATUS directly from HW flags
                    default: rdata_o <= 32'd0;
                endcase
            end
        end
    end

=======
    // --- Logic Doc (Read Path) ---
    always @(posedge clk) begin
        case (i_addr)
            // Doc CTRL: clr_fault luon doc la 0 (chuan Write-to-clear)
            8'h00: o_rd_data <= {29'd0, 1'b0, o_wdi_src, o_wd_en}; 
            8'h04: o_rd_data <= o_twd_ms;
            8'h08: o_rd_data <= o_trst_ms;
            8'h0C: o_rd_data <= {16'd0, o_arm_delay_us};
            8'h10: o_rd_data <= {27'd0, i_last_kick_src, i_wdo_state, 
                                i_enout_state, i_fault_active, i_en_effective};
            default: o_rd_data <= 32'hDEADBEEF; // Bao hieu dia chi sai
        endcase
    end
>>>>>>> 699ba6d65a31e1d6fbf9d9726b025265a61b2653
endmodule