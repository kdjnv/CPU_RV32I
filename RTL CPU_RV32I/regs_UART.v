// Created with Corsair v1.0.4

module regs_UART #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // U_CTRL.EN
    output  csr_u_ctrl_en_out,
    // U_CTRL.STRTX
    output  csr_u_ctrl_strtx_out,
    // U_CTRL.BR
    output [3:0] csr_u_ctrl_br_out,
    // U_CTRL.CLK
    output [7:0] csr_u_ctrl_clk_out,

    // U_STAT.TBUSY
    input  csr_u_stat_tbusy_in,
    // U_STAT.RXNE
    input  csr_u_stat_rxne_in,

    // U_TXDATA.DATA
    output [7:0] csr_u_txdata_data_out,

    // U_RXDATA.DATA
    input [7:0] csr_u_rxdata_data_in,

    // APB
    input               psel,
    input  [ADDR_W-1:0] paddr,
    input               penable,
    input               pwrite,
    input  [DATA_W-1:0] pwdata,
    input  [STRB_W-1:0] pstrb,
    output [DATA_W-1:0] prdata,
    output              pready,
    output              pslverr
);
wire              wready;
wire [ADDR_W-1:0] waddr;
wire [DATA_W-1:0] wdata;
wire              wen;
wire [STRB_W-1:0] wstrb;
wire [DATA_W-1:0] rdata;
wire              rvalid;
wire [ADDR_W-1:0] raddr;
wire              ren;
// APB interface
assign prdata  = rdata;
assign pslverr = 1'b0; // always OKAY
assign pready  = wen             ? wready :
                 (ren & penable) ? rvalid : 1'b1;

// Local Bus interface
assign waddr = paddr;
assign wdata = pwdata;
assign wstrb = pstrb;
assign wen   = psel & penable & pwrite;

assign raddr = paddr;
assign ren   = psel & penable & (~pwrite);

//------------------------------------------------------------------------------
// CSR:
// [0x0] - U_CTRL - UART Configuration Register
//------------------------------------------------------------------------------
wire [31:0] csr_u_ctrl_rdata;
assign csr_u_ctrl_rdata[3:2] = 2'h0;
assign csr_u_ctrl_rdata[31:16] = 16'h0;

wire csr_u_ctrl_wen;
assign csr_u_ctrl_wen = wen && (waddr == 32'h0);

wire csr_u_ctrl_ren;
assign csr_u_ctrl_ren = ren && (raddr == 32'h0);
reg csr_u_ctrl_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_u_ctrl_ren_ff <= 1'b0;
    end else begin
        csr_u_ctrl_ren_ff <= csr_u_ctrl_ren;
    end
end
//---------------------
// Bit field:
// U_CTRL[0] - EN - Enable UART
// access: rw, hardware: o
//---------------------
reg  csr_u_ctrl_en_ff;

assign csr_u_ctrl_rdata[0] = csr_u_ctrl_en_ff;

assign csr_u_ctrl_en_out = csr_u_ctrl_en_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_ctrl_en_ff <= 1'b0;
    end else  begin
     if (csr_u_ctrl_wen) begin
            if (wstrb[0]) begin
                csr_u_ctrl_en_ff <= wdata[0];
            end
        end else begin
            csr_u_ctrl_en_ff <= csr_u_ctrl_en_ff;
        end
    end
end


//---------------------
// Bit field:
// U_CTRL[1] - STRTX - Start Transmission (1 cycle pulse)
// access: rw, hardware: o
//---------------------
reg  csr_u_ctrl_strtx_ff;

assign csr_u_ctrl_rdata[1] = csr_u_ctrl_strtx_ff;

assign csr_u_ctrl_strtx_out = csr_u_ctrl_strtx_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_ctrl_strtx_ff <= 1'b0;
    end else  begin
     if (csr_u_ctrl_wen) begin
            if (wstrb[0]) begin
                csr_u_ctrl_strtx_ff <= wdata[1];
            end
        end else begin
            csr_u_ctrl_strtx_ff <= csr_u_ctrl_strtx_ff;
        end
    end
end


//---------------------
// Bit field:
// U_CTRL[7:4] - BR - Baud Rate Selector
// access: rw, hardware: o
//---------------------
reg [3:0] csr_u_ctrl_br_ff;

assign csr_u_ctrl_rdata[7:4] = csr_u_ctrl_br_ff;

assign csr_u_ctrl_br_out = csr_u_ctrl_br_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_ctrl_br_ff <= 4'hf;
    end else  begin
     if (csr_u_ctrl_wen) begin
            if (wstrb[0]) begin
                csr_u_ctrl_br_ff[3:0] <= wdata[7:4];
            end
        end else begin
            csr_u_ctrl_br_ff <= csr_u_ctrl_br_ff;
        end
    end
end


//---------------------
// Bit field:
// U_CTRL[15:8] - CLK - System Clock Frequency (in MHz or other unit, implementation-defined)
// access: rw, hardware: o
//---------------------
reg [7:0] csr_u_ctrl_clk_ff;

assign csr_u_ctrl_rdata[15:8] = csr_u_ctrl_clk_ff;

assign csr_u_ctrl_clk_out = csr_u_ctrl_clk_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_ctrl_clk_ff <= 8'h0;
    end else  begin
     if (csr_u_ctrl_wen) begin
            if (wstrb[1]) begin
                csr_u_ctrl_clk_ff[7:0] <= wdata[15:8];
            end
        end else begin
            csr_u_ctrl_clk_ff <= csr_u_ctrl_clk_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - U_STAT - UART Status Register
//------------------------------------------------------------------------------
wire [31:0] csr_u_stat_rdata;
assign csr_u_stat_rdata[31:2] = 30'h0;


wire csr_u_stat_ren;
assign csr_u_stat_ren = ren && (raddr == 32'h4);
reg csr_u_stat_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_u_stat_ren_ff <= 1'b0;
    end else begin
        csr_u_stat_ren_ff <= csr_u_stat_ren;
    end
end
//---------------------
// Bit field:
// U_STAT[0] - TBUSY - Transmitter Busy
// access: ro, hardware: i
//---------------------
reg  csr_u_stat_tbusy_ff;

assign csr_u_stat_rdata[0] = csr_u_stat_tbusy_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_u_stat_tbusy_ff <= 1'b0;
    end else  begin
              begin            csr_u_stat_tbusy_ff <= csr_u_stat_tbusy_in;
        end
    end
end


//---------------------
// Bit field:
// U_STAT[1] - RXNE - Receive Buffer Not Empty
// access: ro, hardware: i
//---------------------
reg  csr_u_stat_rxne_ff;

assign csr_u_stat_rdata[1] = csr_u_stat_rxne_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_u_stat_rxne_ff <= 1'b0;
    end else  begin
              begin            csr_u_stat_rxne_ff <= csr_u_stat_rxne_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - U_TXDATA - UART Transmit Data Register
//------------------------------------------------------------------------------
wire [31:0] csr_u_txdata_rdata;
assign csr_u_txdata_rdata[31:8] = 24'h0;

wire csr_u_txdata_wen;
assign csr_u_txdata_wen = wen && (waddr == 32'h8);

wire csr_u_txdata_ren;
assign csr_u_txdata_ren = ren && (raddr == 32'h8);
reg csr_u_txdata_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_u_txdata_ren_ff <= 1'b0;
    end else begin
        csr_u_txdata_ren_ff <= csr_u_txdata_ren;
    end
end
//---------------------
// Bit field:
// U_TXDATA[7:0] - DATA - Data To Transmit
// access: rw, hardware: o
//---------------------
reg [7:0] csr_u_txdata_data_ff;

assign csr_u_txdata_rdata[7:0] = csr_u_txdata_data_ff;

assign csr_u_txdata_data_out = csr_u_txdata_data_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_txdata_data_ff <= 8'h0;
    end else  begin
     if (csr_u_txdata_wen) begin
            if (wstrb[0]) begin
                csr_u_txdata_data_ff[7:0] <= wdata[7:0];
            end
        end else begin
            csr_u_txdata_data_ff <= csr_u_txdata_data_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc] - U_RXDATA - UART Receive Data Register
//------------------------------------------------------------------------------
wire [31:0] csr_u_rxdata_rdata;
assign csr_u_rxdata_rdata[31:8] = 24'h0;


wire csr_u_rxdata_ren;
assign csr_u_rxdata_ren = ren && (raddr == 32'hc);
reg csr_u_rxdata_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_u_rxdata_ren_ff <= 1'b0;
    end else begin
        csr_u_rxdata_ren_ff <= csr_u_rxdata_ren;
    end
end
//---------------------
// Bit field:
// U_RXDATA[7:0] - DATA - Received Data
// access: ro, hardware: i
//---------------------
reg [7:0] csr_u_rxdata_data_ff;

assign csr_u_rxdata_rdata[7:0] = csr_u_rxdata_data_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_u_rxdata_data_ff <= 8'h0;
    end else  begin
              begin            csr_u_rxdata_data_ff <= csr_u_rxdata_data_in;
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
            32'h0: rdata_ff <= csr_u_ctrl_rdata;
            32'h4: rdata_ff <= csr_u_stat_rdata;
            32'h8: rdata_ff <= csr_u_txdata_rdata;
            32'hc: rdata_ff <= csr_u_rxdata_rdata;
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