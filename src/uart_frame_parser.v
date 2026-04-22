`timescale 1ns / 1ps

// =============================================================================
// Module: uart_frame_parser
// Description: Implements the Finite State Machine (FSM) to parse incoming 
//              UART frames and dispatch commands. Frame structure:
//              [0x55] [CMD] [ADDR] [LEN] [DATA...] [CHK]
//              Provides an interface to the register file and the watchdog core.
//              Generates corresponding ACK or response frames.
// =============================================================================

module uart_frame_parser (
    input  wire        clk,
    input  wire        rst_n,

    // =========================================================
    // UART RX INTERFACE
    // =========================================================
    input  wire [7:0]  rx_data_i,
    input  wire        rx_done_i,

    // =========================================================
    // UART TX INTERFACE
    // =========================================================
    output reg  [7:0]  tx_data_o,
    output reg         tx_en_o,
    input  wire        tx_busy_i,

    // =========================================================
    // REGFILE INTERFACE
    // =========================================================
    output reg  [7:0]  reg_addr_o,
    output reg         reg_we_o,
    output reg         reg_re_o,
    output reg  [31:0] reg_wdata_o,
    input  wire [31:0] reg_rdata_i,

    // =========================================================
    // WATCHDOG CORE INTERFACE
    // =========================================================
    output reg         uart_kick_pulse_o,
    input  wire        wdi_src_i          // Needs to know if SW kick is enabled
);

    // =========================================================
    // FSM STATES
    // =========================================================
    localparam S_WAIT_55 = 4'd0;
    localparam S_CMD     = 4'd1;
    localparam S_ADDR    = 4'd2;
    localparam S_LEN     = 4'd3;
    localparam S_DATA    = 4'd4;
    localparam S_CHK     = 4'd5;
    localparam S_EXEC    = 4'd6;
    localparam S_WAIT_RD = 4'd7;
    localparam S_TX_PREP = 4'd8;
    localparam S_TX_SEND = 4'd9;

    reg [3:0] state;

    // Temporary registers to hold incoming frame components
    reg [7:0] cmd_reg;
    reg [7:0] len_reg;
    reg [7:0] calc_chk;      // Checksum accumulator
    reg [2:0] byte_cnt;      // Counter for DATA bytes received
    
    // Transmission buffer for the response frame
    reg [7:0] tx_buf [0:7];
    reg [3:0] tx_idx;        // Index of the byte currently being transmitted
    reg [3:0] tx_len_total;  // Total number of bytes in the response frame

    // Edge detector for tx_busy to know when a byte transmission finishes
    reg tx_busy_d1;
    wire tx_busy_falling = (tx_busy_d1 == 1'b1 && tx_busy_i == 1'b0);

    // =========================================================
    // PARSER FSM LOGIC
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_WAIT_55;
            reg_we_o          <= 1'b0;
            reg_re_o          <= 1'b0;
            reg_addr_o        <= 8'd0;
            reg_wdata_o       <= 32'd0;
            uart_kick_pulse_o <= 1'b0;
            tx_en_o           <= 1'b0;
            tx_busy_d1        <= 1'b0;
            calc_chk          <= 8'd0;
            byte_cnt          <= 3'd0;
        end else begin
            tx_busy_d1        <= tx_busy_i;
            
            // Ensure pulses are only 1 clock cycle wide
            uart_kick_pulse_o <= 1'b0; 
            reg_we_o          <= 1'b0;
            reg_re_o          <= 1'b0;
            tx_en_o           <= 1'b0;

            case (state)
                // ----------------------------------------------------
                // PHASE 1: RX FRAME PARSING
                // ----------------------------------------------------
                S_WAIT_55: begin
                    if (rx_done_i && rx_data_i == 8'h55) begin
                        state <= S_CMD;
                    end
                end

                S_CMD: begin
                    if (rx_done_i) begin
                        cmd_reg  <= rx_data_i;
                        calc_chk <= rx_data_i; // Initialize Checksum XOR sequence
                        state    <= S_ADDR;
                    end
                end

                S_ADDR: begin
                    if (rx_done_i) begin
                        reg_addr_o <= rx_data_i;
                        calc_chk   <= calc_chk ^ rx_data_i;
                        state      <= S_LEN;
                    end
                end

                S_LEN: begin
                    if (rx_done_i) begin
                        len_reg  <= rx_data_i;
                        calc_chk <= calc_chk ^ rx_data_i;
                        byte_cnt <= 3'd0;
                        if (rx_data_i > 0) state <= S_DATA;
                        else               state <= S_CHK; // Skip DATA if length is 0
                    end
                end

                S_DATA: begin
                    if (rx_done_i) begin
                        // Assemble incoming bytes in Big-Endian format (MSB first)
                        reg_wdata_o <= {reg_wdata_o[23:0], rx_data_i}; 
                        calc_chk    <= calc_chk ^ rx_data_i;
                        byte_cnt    <= byte_cnt + 3'd1;
                        if (byte_cnt == len_reg - 1'b1) begin
                            state <= S_CHK;
                        end
                    end
                end

                S_CHK: begin
                    if (rx_done_i) begin
                        if (rx_data_i == calc_chk) begin
                            state <= S_EXEC; // Checksum matches -> Execute command
                        end else begin
                            state <= S_WAIT_55; // Checksum mismatch -> Drop frame
                        end
                    end
                end

                // ----------------------------------------------------
                // PHASE 2: COMMAND EXECUTION
                // ----------------------------------------------------
                S_EXEC: begin
                    case (cmd_reg)
                        8'h01: begin // WRITE Command
                            reg_we_o <= 1'b1;
                            state    <= S_TX_PREP; // Prepare ACK
                        end
                        8'h02, 8'h04: begin // READ or GET_STATUS Command
                            reg_re_o <= 1'b1;
                            if (cmd_reg == 8'h04) reg_addr_o <= 8'h10; // Hardcode STATUS address
                            state <= S_WAIT_RD; // Wait for regfile to return data
                        end
                        8'h03: begin // KICK Command
                            if (wdi_src_i == 1'b1) begin
                                uart_kick_pulse_o <= 1'b1;
                                state             <= S_TX_PREP;
                            end else begin
                                // SW Kicks disabled -> Drop frame silently, causing PC timeout
                                state             <= S_WAIT_55;
                            end
                        end
                        default: state <= S_WAIT_55;
                    endcase
                end

                S_WAIT_RD: begin
                    // Wait 1 clock cycle for reg_rdata_i to update
                    state <= S_TX_PREP;
                end

                // ----------------------------------------------------
                // PHASE 3: TX RESPONSE PACKAGING
                // ----------------------------------------------------
                S_TX_PREP: begin
                    tx_buf[0] <= 8'h55;
                    tx_buf[1] <= cmd_reg;
                    tx_buf[2] <= reg_addr_o;
                    
                    if (cmd_reg == 8'h02 || cmd_reg == 8'h04) begin
                        // Read response: 4 bytes of data
                        tx_buf[3] <= 8'd4; // LEN = 4
                        tx_buf[4] <= reg_rdata_i[31:24];
                        tx_buf[5] <= reg_rdata_i[23:16];
                        tx_buf[6] <= reg_rdata_i[15:8];
                        tx_buf[7] <= reg_rdata_i[7:0];
                        tx_len_total <= 4'd9; // 55, CMD, ADDR, LEN, 4xDATA, CHK
                    end else begin
                        // Write/Kick response: No data (Empty ACK)
                        tx_buf[3] <= 8'd0; // LEN = 0
                        tx_len_total <= 4'd5; // 55, CMD, ADDR, LEN, CHK
                    end
                    
                    tx_idx <= 4'd0;
                    state  <= S_TX_SEND;
                end

                S_TX_SEND: begin
                    // When the transmission of a byte finishes, increment index
                    if (tx_busy_falling) begin
                        if (tx_idx == tx_len_total - 1'b1) begin
                            state <= S_WAIT_55; // Finished transmitting -> Wait for new frame
                        end else begin
                            tx_idx <= tx_idx + 4'd1;
                        end
                    end 
                    // Feed new data if TX module is completely idle
                    else if (!tx_busy_i && !tx_en_o && !tx_busy_d1) begin
                        // If we're at the final byte (CHK)
                        if (tx_idx == tx_len_total - 1'b1) begin
                            if (tx_len_total == 4'd9) begin // Data read ACK
                                tx_data_o <= tx_buf[1] ^ tx_buf[2] ^ tx_buf[3] ^ 
                                             tx_buf[4] ^ tx_buf[5] ^ tx_buf[6] ^ tx_buf[7];
                            end else begin // Empty ACK
                                tx_data_o <= tx_buf[1] ^ tx_buf[2] ^ tx_buf[3];
                            end
                        end else begin
                            tx_data_o <= tx_buf[tx_idx];
                        end

                        tx_en_o <= 1'b1; // Pulse TX enable
                    end
                end

                default: state <= S_WAIT_55;
            endcase
        end
    end

endmodule
