// Created with Corsair v1.0.4

module regs_I2C #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // I2C_CONTROL.CLKDIV
    output [9:0] csr_i2c_control_clkdiv_out,
    // I2C_CONTROL.NUMBRX
    output [2:0] csr_i2c_control_numbrx_out,
    // I2C_CONTROL.DUTYSCL
    output [3:0] csr_i2c_control_dutyscl_out,
    // I2C_CONTROL.RSE
    output  csr_i2c_control_rse_out,
    // I2C_CONTROL.LENADD
    output [1:0] csr_i2c_control_lenadd_out,
    // I2C_CONTROL.EN
    output  csr_i2c_control_en_out,
    // I2C_CONTROL.STRX
    output  csr_i2c_control_strx_out,
    // I2C_CONTROL.MODE
    output  csr_i2c_control_mode_out,
    // I2C_CONTROL.NUMBTX
    output [2:0] csr_i2c_control_numbtx_out,
    // I2C_CONTROL.RST
    output  csr_i2c_control_rst_out,
    // I2C_CONTROL.SEML
    output [4:0] csr_i2c_control_seml_out,

    // I2C_STATUS.SLAVE_FOUND
    input  csr_i2c_status_slave_found_in,
    // I2C_STATUS.TXE
    input  csr_i2c_status_txe_in,
    // I2C_STATUS.TXNE
    input  csr_i2c_status_txne_in,
    // I2C_STATUS.RXE
    input  csr_i2c_status_rxe_in,
    // I2C_STATUS.RXNE
    input  csr_i2c_status_rxne_in,
    // I2C_STATUS.ERR
    input  csr_i2c_status_err_in,
    // I2C_STATUS.BUSY
    input  csr_i2c_status_busy_in,
    // I2C_STATUS.TS
    input  csr_i2c_status_ts_in,
    // I2C_STATUS.RS
    input  csr_i2c_status_rs_in,

    // I2C_SLAVE_ADDR.ADDRESS
    output [9:0] csr_i2c_slave_addr_address_out,

    // I2C_REG_ADDR.ADDS
    output [7:0] csr_i2c_reg_addr_adds_out,

    // I2C_TX_DATA_LOW.TX_DATA_LOW
    output [31:0] csr_i2c_tx_data_low_tx_data_low_out,

    // I2C_TX_DATA_HIGH.TX_DATA_HIGH
    output [31:0] csr_i2c_tx_data_high_tx_data_high_out,

    // I2C_RX_DATA_LOW.RX_DATA_LOW
    input [31:0] csr_i2c_rx_data_low_rx_data_low_in,

    // I2C_RX_DATA_HIGH.RX_DATA_HIGH
    input [31:0] csr_i2c_rx_data_high_rx_data_high_in,

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
// [0x0] - I2C_CONTROL - I2C Control Register
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_control_rdata;

wire csr_i2c_control_wen;
assign csr_i2c_control_wen = wen && (waddr == 32'h0);

wire csr_i2c_control_ren;
assign csr_i2c_control_ren = ren && (raddr == 32'h0);
reg csr_i2c_control_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_ren_ff <= 1'b0;
    end else begin
        csr_i2c_control_ren_ff <= csr_i2c_control_ren;
    end
end
//---------------------
// Bit field:
// I2C_CONTROL[9:0] - CLKDIV - Clock divider for SCL
// access: rw, hardware: o
//---------------------
reg [9:0] csr_i2c_control_clkdiv_ff;

assign csr_i2c_control_rdata[9:0] = csr_i2c_control_clkdiv_ff;

assign csr_i2c_control_clkdiv_out = csr_i2c_control_clkdiv_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_clkdiv_ff <= 10'h10e;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[0]) begin
                csr_i2c_control_clkdiv_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_i2c_control_clkdiv_ff[9:8] <= wdata[9:8];
            end
        end else begin
            csr_i2c_control_clkdiv_ff <= csr_i2c_control_clkdiv_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[12:10] - NUMBRX - Number of bytes to receive (0 means 1)
// access: rw, hardware: o
//---------------------
reg [2:0] csr_i2c_control_numbrx_ff;

assign csr_i2c_control_rdata[12:10] = csr_i2c_control_numbrx_ff;

assign csr_i2c_control_numbrx_out = csr_i2c_control_numbrx_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_numbrx_ff <= 3'h1;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[1]) begin
                csr_i2c_control_numbrx_ff[2:0] <= wdata[12:10];
            end
        end else begin
            csr_i2c_control_numbrx_ff <= csr_i2c_control_numbrx_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[16:13] - DUTYSCL - Duty cycle for SCL (reserved for future use)
// access: rw, hardware: o
//---------------------
reg [3:0] csr_i2c_control_dutyscl_ff;

assign csr_i2c_control_rdata[16:13] = csr_i2c_control_dutyscl_ff;

assign csr_i2c_control_dutyscl_out = csr_i2c_control_dutyscl_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_dutyscl_ff <= 4'h5;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[1]) begin
                csr_i2c_control_dutyscl_ff[2:0] <= wdata[15:13];
            end
            if (wstrb[2]) begin
                csr_i2c_control_dutyscl_ff[3] <= wdata[16];
            end
        end else begin
            csr_i2c_control_dutyscl_ff <= csr_i2c_control_dutyscl_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[17] - RSE - Register Slave Enable
// access: rw, hardware: o
//---------------------
reg  csr_i2c_control_rse_ff;

assign csr_i2c_control_rdata[17] = csr_i2c_control_rse_ff;

assign csr_i2c_control_rse_out = csr_i2c_control_rse_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_rse_ff <= 1'b0;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[2]) begin
                csr_i2c_control_rse_ff <= wdata[17];
            end
        end else begin
            csr_i2c_control_rse_ff <= csr_i2c_control_rse_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[19:18] - LENADD - Slave address length
// access: rw, hardware: o
//---------------------
reg [1:0] csr_i2c_control_lenadd_ff;

assign csr_i2c_control_rdata[19:18] = csr_i2c_control_lenadd_ff;

assign csr_i2c_control_lenadd_out = csr_i2c_control_lenadd_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_lenadd_ff <= 2'h0;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[2]) begin
                csr_i2c_control_lenadd_ff[1:0] <= wdata[19:18];
            end
        end else begin
            csr_i2c_control_lenadd_ff <= csr_i2c_control_lenadd_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[20] - EN - Enable I2C
// access: rw, hardware: o
//---------------------
reg  csr_i2c_control_en_ff;

assign csr_i2c_control_rdata[20] = csr_i2c_control_en_ff;

assign csr_i2c_control_en_out = csr_i2c_control_en_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_en_ff <= 1'b0;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[2]) begin
                csr_i2c_control_en_ff <= wdata[20];
            end
        end else begin
            csr_i2c_control_en_ff <= csr_i2c_control_en_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[21] - STRX - Start transmit/receive
// access: rw, hardware: o
//---------------------
reg  csr_i2c_control_strx_ff;

assign csr_i2c_control_rdata[21] = csr_i2c_control_strx_ff;

assign csr_i2c_control_strx_out = csr_i2c_control_strx_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_strx_ff <= 1'b0;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[2]) begin
                csr_i2c_control_strx_ff <= wdata[21];
            end
        end else begin
            csr_i2c_control_strx_ff <= csr_i2c_control_strx_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[22] - MODE - Transfer mode (0 transmit, 1 receive)
// access: rw, hardware: o
//---------------------
reg  csr_i2c_control_mode_ff;

assign csr_i2c_control_rdata[22] = csr_i2c_control_mode_ff;

assign csr_i2c_control_mode_out = csr_i2c_control_mode_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_mode_ff <= 1'b0;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[2]) begin
                csr_i2c_control_mode_ff <= wdata[22];
            end
        end else begin
            csr_i2c_control_mode_ff <= csr_i2c_control_mode_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[25:23] - NUMBTX - Number of bytes to transmit (0 means 1)
// access: rw, hardware: o
//---------------------
reg [2:0] csr_i2c_control_numbtx_ff;

assign csr_i2c_control_rdata[25:23] = csr_i2c_control_numbtx_ff;

assign csr_i2c_control_numbtx_out = csr_i2c_control_numbtx_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_numbtx_ff <= 3'h1;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[2]) begin
                csr_i2c_control_numbtx_ff[0] <= wdata[23];
            end
            if (wstrb[3]) begin
                csr_i2c_control_numbtx_ff[2:1] <= wdata[25:24];
            end
        end else begin
            csr_i2c_control_numbtx_ff <= csr_i2c_control_numbtx_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[26] - RST - Repeat start condition
// access: rw, hardware: o
//---------------------
reg  csr_i2c_control_rst_ff;

assign csr_i2c_control_rdata[26] = csr_i2c_control_rst_ff;

assign csr_i2c_control_rst_out = csr_i2c_control_rst_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_rst_ff <= 1'b0;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[3]) begin
                csr_i2c_control_rst_ff <= wdata[26];
            end
        end else begin
            csr_i2c_control_rst_ff <= csr_i2c_control_rst_ff;
        end
    end
end


//---------------------
// Bit field:
// I2C_CONTROL[31:27] - SEML - Sample location on SCL pulse (max 9)
// access: rw, hardware: o
//---------------------
reg [4:0] csr_i2c_control_seml_ff;

assign csr_i2c_control_rdata[31:27] = csr_i2c_control_seml_ff;

assign csr_i2c_control_seml_out = csr_i2c_control_seml_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_control_seml_ff <= 5'h5;
    end else  begin
     if (csr_i2c_control_wen) begin
            if (wstrb[3]) begin
                csr_i2c_control_seml_ff[4:0] <= wdata[31:27];
            end
        end else begin
            csr_i2c_control_seml_ff <= csr_i2c_control_seml_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - I2C_STATUS - I2C Status Register
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_status_rdata;
assign csr_i2c_status_rdata[5:3] = 3'h0;
assign csr_i2c_status_rdata[10:8] = 3'h0;
assign csr_i2c_status_rdata[31:15] = 17'h0;


wire csr_i2c_status_ren;
assign csr_i2c_status_ren = ren && (raddr == 32'h4);
reg csr_i2c_status_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_ren_ff <= 1'b0;
    end else begin
        csr_i2c_status_ren_ff <= csr_i2c_status_ren;
    end
end
//---------------------
// Bit field:
// I2C_STATUS[0] - SLAVE_FOUND - Slave address detected
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_slave_found_ff;

assign csr_i2c_status_rdata[0] = csr_i2c_status_slave_found_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_slave_found_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_slave_found_ff <= csr_i2c_status_slave_found_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[1] - TXE - Transmit register empty
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_txe_ff;

assign csr_i2c_status_rdata[1] = csr_i2c_status_txe_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_txe_ff <= 1'b1;
    end else  begin
              begin            csr_i2c_status_txe_ff <= csr_i2c_status_txe_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[2] - TXNE - Transmit register not empty
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_txne_ff;

assign csr_i2c_status_rdata[2] = csr_i2c_status_txne_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_txne_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_txne_ff <= csr_i2c_status_txne_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[6] - RXE - Receive register empty
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_rxe_ff;

assign csr_i2c_status_rdata[6] = csr_i2c_status_rxe_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_rxe_ff <= 1'b1;
    end else  begin
              begin            csr_i2c_status_rxe_ff <= csr_i2c_status_rxe_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[7] - RXNE - Receive register not empty
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_rxne_ff;

assign csr_i2c_status_rdata[7] = csr_i2c_status_rxne_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_rxne_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_rxne_ff <= csr_i2c_status_rxne_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[11] - ERR - Error flag
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_err_ff;

assign csr_i2c_status_rdata[11] = csr_i2c_status_err_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_err_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_err_ff <= csr_i2c_status_err_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[12] - BUSY - I2C busy status
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_busy_ff;

assign csr_i2c_status_rdata[12] = csr_i2c_status_busy_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_busy_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_busy_ff <= csr_i2c_status_busy_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[13] - TS - Transmission complete
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_ts_ff;

assign csr_i2c_status_rdata[13] = csr_i2c_status_ts_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_ts_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_ts_ff <= csr_i2c_status_ts_in;
        end
    end
end


//---------------------
// Bit field:
// I2C_STATUS[14] - RS - Reception complete
// access: ro, hardware: i
//---------------------
reg  csr_i2c_status_rs_ff;

assign csr_i2c_status_rdata[14] = csr_i2c_status_rs_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_status_rs_ff <= 1'b0;
    end else  begin
              begin            csr_i2c_status_rs_ff <= csr_i2c_status_rs_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - I2C_SLAVE_ADDR - I2C Slave Register Address
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_slave_addr_rdata;
assign csr_i2c_slave_addr_rdata[31:10] = 22'h0;

wire csr_i2c_slave_addr_wen;
assign csr_i2c_slave_addr_wen = wen && (waddr == 32'h8);

wire csr_i2c_slave_addr_ren;
assign csr_i2c_slave_addr_ren = ren && (raddr == 32'h8);
reg csr_i2c_slave_addr_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_slave_addr_ren_ff <= 1'b0;
    end else begin
        csr_i2c_slave_addr_ren_ff <= csr_i2c_slave_addr_ren;
    end
end
//---------------------
// Bit field:
// I2C_SLAVE_ADDR[9:0] - ADDRESS - 8-bit register address
// access: rw, hardware: o
//---------------------
reg [9:0] csr_i2c_slave_addr_address_ff;

assign csr_i2c_slave_addr_rdata[9:0] = csr_i2c_slave_addr_address_ff;

assign csr_i2c_slave_addr_address_out = csr_i2c_slave_addr_address_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_slave_addr_address_ff <= 10'h0;
    end else  begin
     if (csr_i2c_slave_addr_wen) begin
            if (wstrb[0]) begin
                csr_i2c_slave_addr_address_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_i2c_slave_addr_address_ff[9:8] <= wdata[9:8];
            end
        end else begin
            csr_i2c_slave_addr_address_ff <= csr_i2c_slave_addr_address_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc] - I2C_REG_ADDR - I2C Slave Device Address
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_reg_addr_rdata;
assign csr_i2c_reg_addr_rdata[31:8] = 24'h0;

wire csr_i2c_reg_addr_wen;
assign csr_i2c_reg_addr_wen = wen && (waddr == 32'hc);

wire csr_i2c_reg_addr_ren;
assign csr_i2c_reg_addr_ren = ren && (raddr == 32'hc);
reg csr_i2c_reg_addr_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_reg_addr_ren_ff <= 1'b0;
    end else begin
        csr_i2c_reg_addr_ren_ff <= csr_i2c_reg_addr_ren;
    end
end
//---------------------
// Bit field:
// I2C_REG_ADDR[7:0] - ADDS - 7 to 10-bit slave address
// access: rw, hardware: o
//---------------------
reg [7:0] csr_i2c_reg_addr_adds_ff;

assign csr_i2c_reg_addr_rdata[7:0] = csr_i2c_reg_addr_adds_ff;

assign csr_i2c_reg_addr_adds_out = csr_i2c_reg_addr_adds_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_reg_addr_adds_ff <= 8'h0;
    end else  begin
     if (csr_i2c_reg_addr_wen) begin
            if (wstrb[0]) begin
                csr_i2c_reg_addr_adds_ff[7:0] <= wdata[7:0];
            end
        end else begin
            csr_i2c_reg_addr_adds_ff <= csr_i2c_reg_addr_adds_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x10] - I2C_TX_DATA_LOW - I2C Transmit Data Low 32-bit
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_tx_data_low_rdata;

wire csr_i2c_tx_data_low_wen;
assign csr_i2c_tx_data_low_wen = wen && (waddr == 32'h10);

wire csr_i2c_tx_data_low_ren;
assign csr_i2c_tx_data_low_ren = ren && (raddr == 32'h10);
reg csr_i2c_tx_data_low_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_tx_data_low_ren_ff <= 1'b0;
    end else begin
        csr_i2c_tx_data_low_ren_ff <= csr_i2c_tx_data_low_ren;
    end
end
//---------------------
// Bit field:
// I2C_TX_DATA_LOW[31:0] - TX_DATA_LOW - Lower 32-bit of data to transmit
// access: rw, hardware: o
//---------------------
reg [31:0] csr_i2c_tx_data_low_tx_data_low_ff;

assign csr_i2c_tx_data_low_rdata[31:0] = csr_i2c_tx_data_low_tx_data_low_ff;

assign csr_i2c_tx_data_low_tx_data_low_out = csr_i2c_tx_data_low_tx_data_low_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_tx_data_low_tx_data_low_ff <= 32'h0;
    end else  begin
     if (csr_i2c_tx_data_low_wen) begin
            if (wstrb[0]) begin
                csr_i2c_tx_data_low_tx_data_low_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_i2c_tx_data_low_tx_data_low_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_i2c_tx_data_low_tx_data_low_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_i2c_tx_data_low_tx_data_low_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_i2c_tx_data_low_tx_data_low_ff <= csr_i2c_tx_data_low_tx_data_low_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x14] - I2C_TX_DATA_HIGH - I2C Transmit Data High 32-bit
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_tx_data_high_rdata;

wire csr_i2c_tx_data_high_wen;
assign csr_i2c_tx_data_high_wen = wen && (waddr == 32'h14);

wire csr_i2c_tx_data_high_ren;
assign csr_i2c_tx_data_high_ren = ren && (raddr == 32'h14);
reg csr_i2c_tx_data_high_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_tx_data_high_ren_ff <= 1'b0;
    end else begin
        csr_i2c_tx_data_high_ren_ff <= csr_i2c_tx_data_high_ren;
    end
end
//---------------------
// Bit field:
// I2C_TX_DATA_HIGH[31:0] - TX_DATA_HIGH - Upper 32-bit of data to transmit
// access: rw, hardware: o
//---------------------
reg [31:0] csr_i2c_tx_data_high_tx_data_high_ff;

assign csr_i2c_tx_data_high_rdata[31:0] = csr_i2c_tx_data_high_tx_data_high_ff;

assign csr_i2c_tx_data_high_tx_data_high_out = csr_i2c_tx_data_high_tx_data_high_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_i2c_tx_data_high_tx_data_high_ff <= 32'h0;
    end else  begin
     if (csr_i2c_tx_data_high_wen) begin
            if (wstrb[0]) begin
                csr_i2c_tx_data_high_tx_data_high_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_i2c_tx_data_high_tx_data_high_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_i2c_tx_data_high_tx_data_high_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_i2c_tx_data_high_tx_data_high_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_i2c_tx_data_high_tx_data_high_ff <= csr_i2c_tx_data_high_tx_data_high_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x18] - I2C_RX_DATA_LOW - I2C Received Data Low 32-bit
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_rx_data_low_rdata;


wire csr_i2c_rx_data_low_ren;
assign csr_i2c_rx_data_low_ren = ren && (raddr == 32'h18);
reg csr_i2c_rx_data_low_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_rx_data_low_ren_ff <= 1'b0;
    end else begin
        csr_i2c_rx_data_low_ren_ff <= csr_i2c_rx_data_low_ren;
    end
end
//---------------------
// Bit field:
// I2C_RX_DATA_LOW[31:0] - RX_DATA_LOW - Lower 32-bit of received data
// access: ro, hardware: i
//---------------------
reg [31:0] csr_i2c_rx_data_low_rx_data_low_ff;

assign csr_i2c_rx_data_low_rdata[31:0] = csr_i2c_rx_data_low_rx_data_low_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_rx_data_low_rx_data_low_ff <= 32'h0;
    end else  begin
              begin            csr_i2c_rx_data_low_rx_data_low_ff <= csr_i2c_rx_data_low_rx_data_low_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x1c] - I2C_RX_DATA_HIGH - I2C Received Data High 32-bit
//------------------------------------------------------------------------------
wire [31:0] csr_i2c_rx_data_high_rdata;


wire csr_i2c_rx_data_high_ren;
assign csr_i2c_rx_data_high_ren = ren && (raddr == 32'h1c);
reg csr_i2c_rx_data_high_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_i2c_rx_data_high_ren_ff <= 1'b0;
    end else begin
        csr_i2c_rx_data_high_ren_ff <= csr_i2c_rx_data_high_ren;
    end
end
//---------------------
// Bit field:
// I2C_RX_DATA_HIGH[31:0] - RX_DATA_HIGH - Upper 32-bit of received data
// access: ro, hardware: i
//---------------------
reg [31:0] csr_i2c_rx_data_high_rx_data_high_ff;

assign csr_i2c_rx_data_high_rdata[31:0] = csr_i2c_rx_data_high_rx_data_high_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_i2c_rx_data_high_rx_data_high_ff <= 32'h0;
    end else  begin
              begin            csr_i2c_rx_data_high_rx_data_high_ff <= csr_i2c_rx_data_high_rx_data_high_in;
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
            32'h0: rdata_ff <= csr_i2c_control_rdata;
            32'h4: rdata_ff <= csr_i2c_status_rdata;
            32'h8: rdata_ff <= csr_i2c_slave_addr_rdata;
            32'hc: rdata_ff <= csr_i2c_reg_addr_rdata;
            32'h10: rdata_ff <= csr_i2c_tx_data_low_rdata;
            32'h14: rdata_ff <= csr_i2c_tx_data_high_rdata;
            32'h18: rdata_ff <= csr_i2c_rx_data_low_rdata;
            32'h1c: rdata_ff <= csr_i2c_rx_data_high_rdata;
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