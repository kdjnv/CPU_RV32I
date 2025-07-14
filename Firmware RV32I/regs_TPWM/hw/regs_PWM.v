// Created with Corsair v1.0.4

module regs_PWM #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // TPWM_config.EN
    output  csr_tpwm_config_en_out,
    // TPWM_config.SEL1
    output [1:0] csr_tpwm_config_sel1_out,
    // TPWM_config.SEL2
    output [1:0] csr_tpwm_config_sel2_out,
    // TPWM_config.RESERVED1
    output [1:0] csr_tpwm_config_reserved1_out,
    // TPWM_config.SEL3
    output [1:0] csr_tpwm_config_sel3_out,
    // TPWM_config.SEL4
    output [1:0] csr_tpwm_config_sel4_out,
    // TPWM_config.NUM
    output [2:0] csr_tpwm_config_num_out,
    // TPWM_config.POL
    output  csr_tpwm_config_pol_out,

    // TPWM_prescaler.DIV
    output [31:0] csr_tpwm_prescaler_div_out,

    // TPWM_period1.PER1
    output [15:0] csr_tpwm_period1_per1_out,
    // TPWM_period1.PER2
    output [15:0] csr_tpwm_period1_per2_out,

    // TPWM_period2.PER3
    output [15:0] csr_tpwm_period2_per3_out,
    // TPWM_period2.PER4
    output [15:0] csr_tpwm_period2_per4_out,

    // TPWM_compare1.CP1
    output [15:0] csr_tpwm_compare1_cp1_out,
    // TPWM_compare1.CP2
    output [15:0] csr_tpwm_compare1_cp2_out,

    // TPWM_compare2.CP3
    output [15:0] csr_tpwm_compare2_cp3_out,
    // TPWM_compare2.CP4
    output [15:0] csr_tpwm_compare2_cp4_out,

    // TPWM_counter1.CNT1
    input [15:0] csr_tpwm_counter1_cnt1_in,
    // TPWM_counter1.CNT2
    input [15:0] csr_tpwm_counter1_cnt2_in,

    // TPWM_counter2.CNT3
    input [15:0] csr_tpwm_counter2_cnt3_in,
    // TPWM_counter2.CNT4
    input [15:0] csr_tpwm_counter2_cnt4_in,

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
// [0x0] - TPWM_config - PWM configuration register
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_config_rdata;
assign csr_tpwm_config_rdata[31:15] = 17'h0;

wire csr_tpwm_config_wen;
assign csr_tpwm_config_wen = wen && (waddr == 32'h0);

wire csr_tpwm_config_ren;
assign csr_tpwm_config_ren = ren && (raddr == 32'h0);
reg csr_tpwm_config_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_config_ren_ff <= csr_tpwm_config_ren;
    end
end
//---------------------
// Bit field:
// TPWM_config[0] - EN - Enable PWM operation
// access: rw, hardware: o
//---------------------
reg  csr_tpwm_config_en_ff;

assign csr_tpwm_config_rdata[0] = csr_tpwm_config_en_ff;

assign csr_tpwm_config_en_out = csr_tpwm_config_en_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_en_ff <= 1'b0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_config_en_ff <= wdata[0];
            end
        end else begin
            csr_tpwm_config_en_ff <= csr_tpwm_config_en_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[2:1] - SEL1 - Output selection for PWM1
// access: rw, hardware: o
//---------------------
reg [1:0] csr_tpwm_config_sel1_ff;

assign csr_tpwm_config_rdata[2:1] = csr_tpwm_config_sel1_ff;

assign csr_tpwm_config_sel1_out = csr_tpwm_config_sel1_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_sel1_ff <= 2'h0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_config_sel1_ff[1:0] <= wdata[2:1];
            end
        end else begin
            csr_tpwm_config_sel1_ff <= csr_tpwm_config_sel1_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[4:3] - SEL2 - Output selection for PWM2
// access: rw, hardware: o
//---------------------
reg [1:0] csr_tpwm_config_sel2_ff;

assign csr_tpwm_config_rdata[4:3] = csr_tpwm_config_sel2_ff;

assign csr_tpwm_config_sel2_out = csr_tpwm_config_sel2_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_sel2_ff <= 2'h0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_config_sel2_ff[1:0] <= wdata[4:3];
            end
        end else begin
            csr_tpwm_config_sel2_ff <= csr_tpwm_config_sel2_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[6:5] - RESERVED1 - Reserved for future use
// access: rw, hardware: o
//---------------------
reg [1:0] csr_tpwm_config_reserved1_ff;

assign csr_tpwm_config_rdata[6:5] = csr_tpwm_config_reserved1_ff;

assign csr_tpwm_config_reserved1_out = csr_tpwm_config_reserved1_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_reserved1_ff <= 2'h0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_config_reserved1_ff[1:0] <= wdata[6:5];
            end
        end else begin
            csr_tpwm_config_reserved1_ff <= csr_tpwm_config_reserved1_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[8:7] - SEL3 - Output selection for PWM3
// access: rw, hardware: o
//---------------------
reg [1:0] csr_tpwm_config_sel3_ff;

assign csr_tpwm_config_rdata[8:7] = csr_tpwm_config_sel3_ff;

assign csr_tpwm_config_sel3_out = csr_tpwm_config_sel3_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_sel3_ff <= 2'h0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_config_sel3_ff[0] <= wdata[7];
            end
            if (wstrb[1]) begin
                csr_tpwm_config_sel3_ff[1] <= wdata[8];
            end
        end else begin
            csr_tpwm_config_sel3_ff <= csr_tpwm_config_sel3_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[10:9] - SEL4 - Output selection for PWM4
// access: rw, hardware: o
//---------------------
reg [1:0] csr_tpwm_config_sel4_ff;

assign csr_tpwm_config_rdata[10:9] = csr_tpwm_config_sel4_ff;

assign csr_tpwm_config_sel4_out = csr_tpwm_config_sel4_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_sel4_ff <= 2'h0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[1]) begin
                csr_tpwm_config_sel4_ff[1:0] <= wdata[10:9];
            end
        end else begin
            csr_tpwm_config_sel4_ff <= csr_tpwm_config_sel4_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[13:11] - NUM - Number of PWM signals selected
// access: rw, hardware: o
//---------------------
reg [2:0] csr_tpwm_config_num_ff;

assign csr_tpwm_config_rdata[13:11] = csr_tpwm_config_num_ff;

assign csr_tpwm_config_num_out = csr_tpwm_config_num_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_num_ff <= 3'h0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[1]) begin
                csr_tpwm_config_num_ff[2:0] <= wdata[13:11];
            end
        end else begin
            csr_tpwm_config_num_ff <= csr_tpwm_config_num_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_config[14] - POL - Polarity of the PWM signal
// access: rw, hardware: o
//---------------------
reg  csr_tpwm_config_pol_ff;

assign csr_tpwm_config_rdata[14] = csr_tpwm_config_pol_ff;

assign csr_tpwm_config_pol_out = csr_tpwm_config_pol_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_config_pol_ff <= 1'b0;
    end else  begin
     if (csr_tpwm_config_wen) begin
            if (wstrb[1]) begin
                csr_tpwm_config_pol_ff <= wdata[14];
            end
        end else begin
            csr_tpwm_config_pol_ff <= csr_tpwm_config_pol_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - TPWM_prescaler - PWM clock divider
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_prescaler_rdata;

wire csr_tpwm_prescaler_wen;
assign csr_tpwm_prescaler_wen = wen && (waddr == 32'h4);

wire csr_tpwm_prescaler_ren;
assign csr_tpwm_prescaler_ren = ren && (raddr == 32'h4);
reg csr_tpwm_prescaler_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_prescaler_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_prescaler_ren_ff <= csr_tpwm_prescaler_ren;
    end
end
//---------------------
// Bit field:
// TPWM_prescaler[31:0] - DIV - Clock divider value for PWM
// access: rw, hardware: o
//---------------------
reg [31:0] csr_tpwm_prescaler_div_ff;

assign csr_tpwm_prescaler_rdata[31:0] = csr_tpwm_prescaler_div_ff;

assign csr_tpwm_prescaler_div_out = csr_tpwm_prescaler_div_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_prescaler_div_ff <= 32'h1b;
    end else  begin
     if (csr_tpwm_prescaler_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_prescaler_div_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tpwm_prescaler_div_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_tpwm_prescaler_div_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tpwm_prescaler_div_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_tpwm_prescaler_div_ff <= csr_tpwm_prescaler_div_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - TPWM_period1 - PWM1 and PWM2 period register
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_period1_rdata;

wire csr_tpwm_period1_wen;
assign csr_tpwm_period1_wen = wen && (waddr == 32'h8);

wire csr_tpwm_period1_ren;
assign csr_tpwm_period1_ren = ren && (raddr == 32'h8);
reg csr_tpwm_period1_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_period1_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_period1_ren_ff <= csr_tpwm_period1_ren;
    end
end
//---------------------
// Bit field:
// TPWM_period1[15:0] - PER1 - Period value for PWM1
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_period1_per1_ff;

assign csr_tpwm_period1_rdata[15:0] = csr_tpwm_period1_per1_ff;

assign csr_tpwm_period1_per1_out = csr_tpwm_period1_per1_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_period1_per1_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_period1_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_period1_per1_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tpwm_period1_per1_ff[15:8] <= wdata[15:8];
            end
        end else begin
            csr_tpwm_period1_per1_ff <= csr_tpwm_period1_per1_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_period1[31:16] - PER2 - Period value for PWM2
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_period1_per2_ff;

assign csr_tpwm_period1_rdata[31:16] = csr_tpwm_period1_per2_ff;

assign csr_tpwm_period1_per2_out = csr_tpwm_period1_per2_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_period1_per2_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_period1_wen) begin
            if (wstrb[2]) begin
                csr_tpwm_period1_per2_ff[7:0] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tpwm_period1_per2_ff[15:8] <= wdata[31:24];
            end
        end else begin
            csr_tpwm_period1_per2_ff <= csr_tpwm_period1_per2_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc] - TPWM_period2 - PWM3 and PWM4 period register
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_period2_rdata;

wire csr_tpwm_period2_wen;
assign csr_tpwm_period2_wen = wen && (waddr == 32'hc);

wire csr_tpwm_period2_ren;
assign csr_tpwm_period2_ren = ren && (raddr == 32'hc);
reg csr_tpwm_period2_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_period2_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_period2_ren_ff <= csr_tpwm_period2_ren;
    end
end
//---------------------
// Bit field:
// TPWM_period2[15:0] - PER3 - Period value for PWM3
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_period2_per3_ff;

assign csr_tpwm_period2_rdata[15:0] = csr_tpwm_period2_per3_ff;

assign csr_tpwm_period2_per3_out = csr_tpwm_period2_per3_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_period2_per3_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_period2_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_period2_per3_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tpwm_period2_per3_ff[15:8] <= wdata[15:8];
            end
        end else begin
            csr_tpwm_period2_per3_ff <= csr_tpwm_period2_per3_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_period2[31:16] - PER4 - Period value for PWM4
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_period2_per4_ff;

assign csr_tpwm_period2_rdata[31:16] = csr_tpwm_period2_per4_ff;

assign csr_tpwm_period2_per4_out = csr_tpwm_period2_per4_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_period2_per4_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_period2_wen) begin
            if (wstrb[2]) begin
                csr_tpwm_period2_per4_ff[7:0] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tpwm_period2_per4_ff[15:8] <= wdata[31:24];
            end
        end else begin
            csr_tpwm_period2_per4_ff <= csr_tpwm_period2_per4_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x10] - TPWM_compare1 - Compare values for PWM1 and PWM2
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_compare1_rdata;

wire csr_tpwm_compare1_wen;
assign csr_tpwm_compare1_wen = wen && (waddr == 32'h10);

wire csr_tpwm_compare1_ren;
assign csr_tpwm_compare1_ren = ren && (raddr == 32'h10);
reg csr_tpwm_compare1_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_compare1_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_compare1_ren_ff <= csr_tpwm_compare1_ren;
    end
end
//---------------------
// Bit field:
// TPWM_compare1[15:0] - CP1 - Compare value for PWM1
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_compare1_cp1_ff;

assign csr_tpwm_compare1_rdata[15:0] = csr_tpwm_compare1_cp1_ff;

assign csr_tpwm_compare1_cp1_out = csr_tpwm_compare1_cp1_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_compare1_cp1_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_compare1_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_compare1_cp1_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tpwm_compare1_cp1_ff[15:8] <= wdata[15:8];
            end
        end else begin
            csr_tpwm_compare1_cp1_ff <= csr_tpwm_compare1_cp1_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_compare1[31:16] - CP2 - Compare value for PWM2
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_compare1_cp2_ff;

assign csr_tpwm_compare1_rdata[31:16] = csr_tpwm_compare1_cp2_ff;

assign csr_tpwm_compare1_cp2_out = csr_tpwm_compare1_cp2_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_compare1_cp2_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_compare1_wen) begin
            if (wstrb[2]) begin
                csr_tpwm_compare1_cp2_ff[7:0] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tpwm_compare1_cp2_ff[15:8] <= wdata[31:24];
            end
        end else begin
            csr_tpwm_compare1_cp2_ff <= csr_tpwm_compare1_cp2_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x14] - TPWM_compare2 - Compare values for PWM3 and PWM4
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_compare2_rdata;

wire csr_tpwm_compare2_wen;
assign csr_tpwm_compare2_wen = wen && (waddr == 32'h14);

wire csr_tpwm_compare2_ren;
assign csr_tpwm_compare2_ren = ren && (raddr == 32'h14);
reg csr_tpwm_compare2_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_compare2_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_compare2_ren_ff <= csr_tpwm_compare2_ren;
    end
end
//---------------------
// Bit field:
// TPWM_compare2[15:0] - CP3 - Compare value for PWM3
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_compare2_cp3_ff;

assign csr_tpwm_compare2_rdata[15:0] = csr_tpwm_compare2_cp3_ff;

assign csr_tpwm_compare2_cp3_out = csr_tpwm_compare2_cp3_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_compare2_cp3_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_compare2_wen) begin
            if (wstrb[0]) begin
                csr_tpwm_compare2_cp3_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tpwm_compare2_cp3_ff[15:8] <= wdata[15:8];
            end
        end else begin
            csr_tpwm_compare2_cp3_ff <= csr_tpwm_compare2_cp3_ff;
        end
    end
end


//---------------------
// Bit field:
// TPWM_compare2[31:16] - CP4 - Compare value for PWM4
// access: rw, hardware: o
//---------------------
reg [15:0] csr_tpwm_compare2_cp4_ff;

assign csr_tpwm_compare2_rdata[31:16] = csr_tpwm_compare2_cp4_ff;

assign csr_tpwm_compare2_cp4_out = csr_tpwm_compare2_cp4_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_compare2_cp4_ff <= 16'hffff;
    end else  begin
     if (csr_tpwm_compare2_wen) begin
            if (wstrb[2]) begin
                csr_tpwm_compare2_cp4_ff[7:0] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tpwm_compare2_cp4_ff[15:8] <= wdata[31:24];
            end
        end else begin
            csr_tpwm_compare2_cp4_ff <= csr_tpwm_compare2_cp4_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x18] - TPWM_counter1 - PWM counters for PWM1 and PWM2
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_counter1_rdata;


wire csr_tpwm_counter1_ren;
assign csr_tpwm_counter1_ren = ren && (raddr == 32'h18);
reg csr_tpwm_counter1_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_counter1_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_counter1_ren_ff <= csr_tpwm_counter1_ren;
    end
end
//---------------------
// Bit field:
// TPWM_counter1[15:0] - CNT1 - Current counter value for PWM1
// access: ro, hardware: i
//---------------------
reg [15:0] csr_tpwm_counter1_cnt1_ff;

assign csr_tpwm_counter1_rdata[15:0] = csr_tpwm_counter1_cnt1_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_counter1_cnt1_ff <= 16'h0;
    end else  begin
              begin            csr_tpwm_counter1_cnt1_ff <= csr_tpwm_counter1_cnt1_in;
        end
    end
end


//---------------------
// Bit field:
// TPWM_counter1[31:16] - CNT2 - Current counter value for PWM2
// access: ro, hardware: i
//---------------------
reg [15:0] csr_tpwm_counter1_cnt2_ff;

assign csr_tpwm_counter1_rdata[31:16] = csr_tpwm_counter1_cnt2_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_counter1_cnt2_ff <= 16'h0;
    end else  begin
              begin            csr_tpwm_counter1_cnt2_ff <= csr_tpwm_counter1_cnt2_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x1c] - TPWM_counter2 - PWM counters for PWM3 and PWM4
//------------------------------------------------------------------------------
wire [31:0] csr_tpwm_counter2_rdata;


wire csr_tpwm_counter2_ren;
assign csr_tpwm_counter2_ren = ren && (raddr == 32'h1c);
reg csr_tpwm_counter2_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_counter2_ren_ff <= 1'b0;
    end else begin
        csr_tpwm_counter2_ren_ff <= csr_tpwm_counter2_ren;
    end
end
//---------------------
// Bit field:
// TPWM_counter2[15:0] - CNT3 - Current counter value for PWM3
// access: ro, hardware: i
//---------------------
reg [15:0] csr_tpwm_counter2_cnt3_ff;

assign csr_tpwm_counter2_rdata[15:0] = csr_tpwm_counter2_cnt3_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_counter2_cnt3_ff <= 16'h0;
    end else  begin
              begin            csr_tpwm_counter2_cnt3_ff <= csr_tpwm_counter2_cnt3_in;
        end
    end
end


//---------------------
// Bit field:
// TPWM_counter2[31:16] - CNT4 - Current counter value for PWM4
// access: ro, hardware: i
//---------------------
reg [15:0] csr_tpwm_counter2_cnt4_ff;

assign csr_tpwm_counter2_rdata[31:16] = csr_tpwm_counter2_cnt4_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tpwm_counter2_cnt4_ff <= 16'h0;
    end else  begin
              begin            csr_tpwm_counter2_cnt4_ff <= csr_tpwm_counter2_cnt4_in;
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
            32'h0: rdata_ff <= csr_tpwm_config_rdata;
            32'h4: rdata_ff <= csr_tpwm_prescaler_rdata;
            32'h8: rdata_ff <= csr_tpwm_period1_rdata;
            32'hc: rdata_ff <= csr_tpwm_period2_rdata;
            32'h10: rdata_ff <= csr_tpwm_compare1_rdata;
            32'h14: rdata_ff <= csr_tpwm_compare2_rdata;
            32'h18: rdata_ff <= csr_tpwm_counter1_rdata;
            32'h1c: rdata_ff <= csr_tpwm_counter2_rdata;
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