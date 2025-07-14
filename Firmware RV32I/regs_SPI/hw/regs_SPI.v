// Created with Corsair v1.0.4

module regs_SPI #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // SPI_TX_LOW.DATA_LOW
    output [31:0] csr_spi_tx_low_data_low_out,

    // SPI_TX_HIGH.DATA_HIGH
    output [31:0] csr_spi_tx_high_data_high_out,

    // SPI_RX_LOW.DATA
    input [31:0] csr_spi_rx_low_data_in,

    // SPI_RX_HIGH.DATA
    input [31:0] csr_spi_rx_high_data_in,

    // CONTROL_REG.BPT
    output [5:0] csr_control_reg_bpt_out,
    // CONTROL_REG.CPOL
    output  csr_control_reg_cpol_out,
    // CONTROL_REG.CPHA
    output  csr_control_reg_cpha_out,
    // CONTROL_REG.ENSPI
    output  csr_control_reg_enspi_out,
    // CONTROL_REG.CSS
    output  csr_control_reg_css_out,
    // CONTROL_REG.NUMSS
    output [3:0] csr_control_reg_numss_out,
    // CONTROL_REG.MSB
    output  csr_control_reg_msb_out,
    // CONTROL_REG.STRX
    output  csr_control_reg_strx_out,
    // CONTROL_REG.DIV
    output [9:0] csr_control_reg_div_out,
    // CONTROL_REG.MODE
    output  csr_control_reg_mode_out,

    // STATUS_REG.RXNE
    input  csr_status_reg_rxne_in,
    // STATUS_REG.TXE
    input  csr_status_reg_txe_in,
    // STATUS_REG.ERR
    input  csr_status_reg_err_in,
    // STATUS_REG.BUSY
    input  csr_status_reg_busy_in,

    // Local Bus
    input  [ADDR_W-1:0] waddr,
    input  [DATA_W-1:0] wdata,
    input               wen,
    input  [STRB_W-1:0] wstrb,
    output              wready,
    input  [ADDR_W-1:0] raddr,
    input               ren,
    output [DATA_W-1:0] rdata,
    output              rvalid
);
//------------------------------------------------------------------------------
// CSR:
// [0x0] - SPI_TX_LOW - Transmit data register low (lower 32 bits)
//------------------------------------------------------------------------------
wire [31:0] csr_spi_tx_low_rdata;

wire csr_spi_tx_low_wen;
assign csr_spi_tx_low_wen = wen && (waddr == 32'h0);

wire csr_spi_tx_low_ren;
assign csr_spi_tx_low_ren = ren && (raddr == 32'h0);
reg csr_spi_tx_low_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_spi_tx_low_ren_ff <= 1'b0;
    end else begin
        csr_spi_tx_low_ren_ff <= csr_spi_tx_low_ren;
    end
end
//---------------------
// Bit field:
// SPI_TX_LOW[31:0] - DATA_LOW - Data to transmit (lower 32 bits)
// access: rw, hardware: o
//---------------------
reg [31:0] csr_spi_tx_low_data_low_ff;

assign csr_spi_tx_low_rdata[31:0] = csr_spi_tx_low_data_low_ff;

assign csr_spi_tx_low_data_low_out = csr_spi_tx_low_data_low_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_spi_tx_low_data_low_ff <= 32'h0;
    end else  begin
     if (csr_spi_tx_low_wen) begin
            if (wstrb[0]) begin
                csr_spi_tx_low_data_low_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_spi_tx_low_data_low_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_spi_tx_low_data_low_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_spi_tx_low_data_low_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_spi_tx_low_data_low_ff <= csr_spi_tx_low_data_low_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - SPI_TX_HIGH - Transmit data register high (higher 32 bits)
//------------------------------------------------------------------------------
wire [31:0] csr_spi_tx_high_rdata;

wire csr_spi_tx_high_wen;
assign csr_spi_tx_high_wen = wen && (waddr == 32'h4);

wire csr_spi_tx_high_ren;
assign csr_spi_tx_high_ren = ren && (raddr == 32'h4);
reg csr_spi_tx_high_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_spi_tx_high_ren_ff <= 1'b0;
    end else begin
        csr_spi_tx_high_ren_ff <= csr_spi_tx_high_ren;
    end
end
//---------------------
// Bit field:
// SPI_TX_HIGH[31:0] - DATA_HIGH - Data to transmit (higher 32 bits)
// access: rw, hardware: o
//---------------------
reg [31:0] csr_spi_tx_high_data_high_ff;

assign csr_spi_tx_high_rdata[31:0] = csr_spi_tx_high_data_high_ff;

assign csr_spi_tx_high_data_high_out = csr_spi_tx_high_data_high_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_spi_tx_high_data_high_ff <= 32'h0;
    end else  begin
     if (csr_spi_tx_high_wen) begin
            if (wstrb[0]) begin
                csr_spi_tx_high_data_high_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_spi_tx_high_data_high_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_spi_tx_high_data_high_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_spi_tx_high_data_high_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_spi_tx_high_data_high_ff <= csr_spi_tx_high_data_high_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - SPI_RX_LOW - Receive data register low (lower 32 bits)
//------------------------------------------------------------------------------
wire [31:0] csr_spi_rx_low_rdata;

wire csr_spi_rx_low_wen;
assign csr_spi_rx_low_wen = wen && (waddr == 32'h8);

wire csr_spi_rx_low_ren;
assign csr_spi_rx_low_ren = ren && (raddr == 32'h8);
reg csr_spi_rx_low_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_spi_rx_low_ren_ff <= 1'b0;
    end else begin
        csr_spi_rx_low_ren_ff <= csr_spi_rx_low_ren;
    end
end
//---------------------
// Bit field:
// SPI_RX_LOW[31:0] - DATA - Data received (lower 32 bits)
// access: rw, hardware: i
//---------------------
reg [31:0] csr_spi_rx_low_data_ff;

assign csr_spi_rx_low_rdata[31:0] = csr_spi_rx_low_data_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_spi_rx_low_data_ff <= 32'h0;
    end else  begin
     if (csr_spi_rx_low_wen) begin
            if (wstrb[0]) begin
                csr_spi_rx_low_data_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_spi_rx_low_data_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_spi_rx_low_data_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_spi_rx_low_data_ff[31:24] <= wdata[31:24];
            end
        end else         begin            csr_spi_rx_low_data_ff <= csr_spi_rx_low_data_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc] - SPI_RX_HIGH - Receive data register high (higher 32 bits)
//------------------------------------------------------------------------------
wire [31:0] csr_spi_rx_high_rdata;


wire csr_spi_rx_high_ren;
assign csr_spi_rx_high_ren = ren && (raddr == 32'hc);
reg csr_spi_rx_high_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_spi_rx_high_ren_ff <= 1'b0;
    end else begin
        csr_spi_rx_high_ren_ff <= csr_spi_rx_high_ren;
    end
end
//---------------------
// Bit field:
// SPI_RX_HIGH[31:0] - DATA - Data received (higher 32 bits)
// access: ro, hardware: i
//---------------------
reg [31:0] csr_spi_rx_high_data_ff;

assign csr_spi_rx_high_rdata[31:0] = csr_spi_rx_high_data_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_spi_rx_high_data_ff <= 32'h0;
    end else  begin
              begin            csr_spi_rx_high_data_ff <= csr_spi_rx_high_data_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x10] - CONTROL_REG - SPI Control register
//------------------------------------------------------------------------------
wire [31:0] csr_control_reg_rdata;
assign csr_control_reg_rdata[10] = 1'b0;
assign csr_control_reg_rdata[15] = 1'b0;
assign csr_control_reg_rdata[31:29] = 3'h0;

wire csr_control_reg_wen;
assign csr_control_reg_wen = wen && (waddr == 32'h10);

wire csr_control_reg_ren;
assign csr_control_reg_ren = ren && (raddr == 32'h10);
reg csr_control_reg_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_ren_ff <= 1'b0;
    end else begin
        csr_control_reg_ren_ff <= csr_control_reg_ren;
    end
end
//---------------------
// Bit field:
// CONTROL_REG[5:0] - BPT - Bits per transfer (0 -> 1 bit, 63 -> 64 bits)
// access: rw, hardware: o
//---------------------
reg [5:0] csr_control_reg_bpt_ff;

assign csr_control_reg_rdata[5:0] = csr_control_reg_bpt_ff;

assign csr_control_reg_bpt_out = csr_control_reg_bpt_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_bpt_ff <= 6'h8;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[0]) begin
                csr_control_reg_bpt_ff[5:0] <= wdata[5:0];
            end
        end else begin
            csr_control_reg_bpt_ff <= csr_control_reg_bpt_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[6] - CPOL - Clock polarity
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_cpol_ff;

assign csr_control_reg_rdata[6] = csr_control_reg_cpol_ff;

assign csr_control_reg_cpol_out = csr_control_reg_cpol_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_cpol_ff <= 1'b0;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[0]) begin
                csr_control_reg_cpol_ff <= wdata[6];
            end
        end else begin
            csr_control_reg_cpol_ff <= csr_control_reg_cpol_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[7] - CPHA - Clock phase
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_cpha_ff;

assign csr_control_reg_rdata[7] = csr_control_reg_cpha_ff;

assign csr_control_reg_cpha_out = csr_control_reg_cpha_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_cpha_ff <= 1'b0;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[0]) begin
                csr_control_reg_cpha_ff <= wdata[7];
            end
        end else begin
            csr_control_reg_cpha_ff <= csr_control_reg_cpha_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[8] - ENSPI - Enable SPI
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_enspi_ff;

assign csr_control_reg_rdata[8] = csr_control_reg_enspi_ff;

assign csr_control_reg_enspi_out = csr_control_reg_enspi_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_enspi_ff <= 1'b0;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[1]) begin
                csr_control_reg_enspi_ff <= wdata[8];
            end
        end else begin
            csr_control_reg_enspi_ff <= csr_control_reg_enspi_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[9] - CSS - Automatic slave select enable
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_css_ff;

assign csr_control_reg_rdata[9] = csr_control_reg_css_ff;

assign csr_control_reg_css_out = csr_control_reg_css_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_css_ff <= 1'b1;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[1]) begin
                csr_control_reg_css_ff <= wdata[9];
            end
        end else begin
            csr_control_reg_css_ff <= csr_control_reg_css_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[14:11] - NUMSS - Slave number selection (0~15)
// access: rw, hardware: o
//---------------------
reg [3:0] csr_control_reg_numss_ff;

assign csr_control_reg_rdata[14:11] = csr_control_reg_numss_ff;

assign csr_control_reg_numss_out = csr_control_reg_numss_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_numss_ff <= 4'h0;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[1]) begin
                csr_control_reg_numss_ff[3:0] <= wdata[14:11];
            end
        end else begin
            csr_control_reg_numss_ff <= csr_control_reg_numss_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[16] - MSB - MSB first enable
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_msb_ff;

assign csr_control_reg_rdata[16] = csr_control_reg_msb_ff;

assign csr_control_reg_msb_out = csr_control_reg_msb_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_msb_ff <= 1'b1;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[2]) begin
                csr_control_reg_msb_ff <= wdata[16];
            end
        end else begin
            csr_control_reg_msb_ff <= csr_control_reg_msb_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[17] - STRX - Start transmit/receive
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_strx_ff;

assign csr_control_reg_rdata[17] = csr_control_reg_strx_ff;

assign csr_control_reg_strx_out = csr_control_reg_strx_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_strx_ff <= 1'b0;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[2]) begin
                csr_control_reg_strx_ff <= wdata[17];
            end
        end else begin
            csr_control_reg_strx_ff <= csr_control_reg_strx_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[27:18] - DIV - Clock divider
// access: rw, hardware: o
//---------------------
reg [9:0] csr_control_reg_div_ff;

assign csr_control_reg_rdata[27:18] = csr_control_reg_div_ff;

assign csr_control_reg_div_out = csr_control_reg_div_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_div_ff <= 10'h1b;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[2]) begin
                csr_control_reg_div_ff[5:0] <= wdata[23:18];
            end
            if (wstrb[3]) begin
                csr_control_reg_div_ff[9:6] <= wdata[27:24];
            end
        end else begin
            csr_control_reg_div_ff <= csr_control_reg_div_ff;
        end
    end
end


//---------------------
// Bit field:
// CONTROL_REG[28] - MODE - 0 - Master mode, 1 - Slave mode
// access: rw, hardware: o
//---------------------
reg  csr_control_reg_mode_ff;

assign csr_control_reg_rdata[28] = csr_control_reg_mode_ff;

assign csr_control_reg_mode_out = csr_control_reg_mode_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_control_reg_mode_ff <= 1'b0;
    end else  begin
     if (csr_control_reg_wen) begin
            if (wstrb[3]) begin
                csr_control_reg_mode_ff <= wdata[28];
            end
        end else begin
            csr_control_reg_mode_ff <= csr_control_reg_mode_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x14] - STATUS_REG - SPI Status register
//------------------------------------------------------------------------------
wire [31:0] csr_status_reg_rdata;
assign csr_status_reg_rdata[3] = 1'b0;
assign csr_status_reg_rdata[31:5] = 27'h0;


wire csr_status_reg_ren;
assign csr_status_reg_ren = ren && (raddr == 32'h14);
reg csr_status_reg_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_status_reg_ren_ff <= 1'b0;
    end else begin
        csr_status_reg_ren_ff <= csr_status_reg_ren;
    end
end
//---------------------
// Bit field:
// STATUS_REG[0] - RXNE - RX not empty (data received)
// access: ro, hardware: i
//---------------------
reg  csr_status_reg_rxne_ff;

assign csr_status_reg_rdata[0] = csr_status_reg_rxne_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_status_reg_rxne_ff <= 1'b0;
    end else  begin
              begin            csr_status_reg_rxne_ff <= csr_status_reg_rxne_in;
        end
    end
end


//---------------------
// Bit field:
// STATUS_REG[1] - TXE - TX empty (ready for transmit)
// access: ro, hardware: i
//---------------------
reg  csr_status_reg_txe_ff;

assign csr_status_reg_rdata[1] = csr_status_reg_txe_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_status_reg_txe_ff <= 1'b0;
    end else  begin
              begin            csr_status_reg_txe_ff <= csr_status_reg_txe_in;
        end
    end
end


//---------------------
// Bit field:
// STATUS_REG[2] - ERR - Error flag (future use)
// access: ro, hardware: i
//---------------------
reg  csr_status_reg_err_ff;

assign csr_status_reg_rdata[2] = csr_status_reg_err_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_status_reg_err_ff <= 1'b0;
    end else  begin
              begin            csr_status_reg_err_ff <= csr_status_reg_err_in;
        end
    end
end


//---------------------
// Bit field:
// STATUS_REG[4] - BUSY - SPI Busy flag
// access: ro, hardware: i
//---------------------
reg  csr_status_reg_busy_ff;

assign csr_status_reg_rdata[4] = csr_status_reg_busy_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_status_reg_busy_ff <= 1'b0;
    end else  begin
              begin            csr_status_reg_busy_ff <= csr_status_reg_busy_in;
        end
    end
end


//------------------------------------------------------------------------------
// Write ready
//------------------------------------------------------------------------------
assign wready = 1'b1;

//------------------------------------------------------------------------------
// Read address decoder
//------------------------------------------------------------------------------
reg [31:0] rdata_ff;
always @(posedge clk) begin
    if (rst) begin
        rdata_ff <= 32'h0;
    end else if (ren) begin
        case (raddr)
            32'h0: rdata_ff <= csr_spi_tx_low_rdata;
            32'h4: rdata_ff <= csr_spi_tx_high_rdata;
            32'h8: rdata_ff <= csr_spi_rx_low_rdata;
            32'hc: rdata_ff <= csr_spi_rx_high_rdata;
            32'h10: rdata_ff <= csr_control_reg_rdata;
            32'h14: rdata_ff <= csr_status_reg_rdata;
            default: rdata_ff <= 32'h0;
        endcase
    end else begin
        rdata_ff <= 32'h0;
    end
end
assign rdata = rdata_ff;

//------------------------------------------------------------------------------
// Read data valid
//------------------------------------------------------------------------------
reg rvalid_ff;
always @(posedge clk) begin
    if (rst) begin
        rvalid_ff <= 1'b0;
    end else if (ren && rvalid) begin
        rvalid_ff <= 1'b0;
    end else if (ren) begin
        rvalid_ff <= 1'b1;
    end
end

assign rvalid = rvalid_ff;

endmodule