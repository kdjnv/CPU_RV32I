// Created with Corsair v1.0.4

module regs_TIME_BASE #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // Tim_config.EN
    output  csr_tim_config_en_out,
    // Tim_config.AR
    output  csr_tim_config_ar_out,
    // Tim_config.DIR
    output  csr_tim_config_dir_out,
    // Tim_config.UD
    output  csr_tim_config_ud_out,

    // Tim_prescaler.DIV
    output [31:0] csr_tim_prescaler_div_out,

    // Tim_period.PER
    output [31:0] csr_tim_period_per_out,

    // Tim_counter.CNT
    input [31:0] csr_tim_counter_cnt_in,

    // Tim_status.OF
    input  csr_tim_status_of_in,
    // Tim_status.OP
    input  csr_tim_status_op_in,
    // Tim_status.ERR
    input  csr_tim_status_err_in,
    // Tim_status.LD
    input  csr_tim_status_ld_in,

    // Tim_load.LOAD
    output [31:0] csr_tim_load_load_out,

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
// [0x0] - Tim_config - Timer configuration register
//------------------------------------------------------------------------------
wire [31:0] csr_tim_config_rdata;
assign csr_tim_config_rdata[3:2] = 2'h0;
assign csr_tim_config_rdata[31:6] = 26'h0;

wire csr_tim_config_wen;
assign csr_tim_config_wen = wen && (waddr == 32'h0);

//---------------------
// Bit field:
// Tim_config[0] - EN - Enable timer
// access: wo, hardware: o
//---------------------
reg  csr_tim_config_en_ff;

assign csr_tim_config_rdata[0] = 1'b0;

assign csr_tim_config_en_out = csr_tim_config_en_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_config_en_ff <= 1'b0;
    end else  begin
     if (csr_tim_config_wen) begin
            if (wstrb[0]) begin
                csr_tim_config_en_ff <= wdata[0];
            end
        end else begin
            csr_tim_config_en_ff <= csr_tim_config_en_ff;
        end
    end
end


//---------------------
// Bit field:
// Tim_config[1] - AR - Auto reload when reaching period
// access: wo, hardware: o
//---------------------
reg  csr_tim_config_ar_ff;

assign csr_tim_config_rdata[1] = 1'b0;

assign csr_tim_config_ar_out = csr_tim_config_ar_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_config_ar_ff <= 1'b1;
    end else  begin
     if (csr_tim_config_wen) begin
            if (wstrb[0]) begin
                csr_tim_config_ar_ff <= wdata[1];
            end
        end else begin
            csr_tim_config_ar_ff <= csr_tim_config_ar_ff;
        end
    end
end


//---------------------
// Bit field:
// Tim_config[4] - DIR - Count direction (0 = up, 1 = down)
// access: wo, hardware: o
//---------------------
reg  csr_tim_config_dir_ff;

assign csr_tim_config_rdata[4] = 1'b0;

assign csr_tim_config_dir_out = csr_tim_config_dir_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_config_dir_ff <= 1'b0;
    end else  begin
     if (csr_tim_config_wen) begin
            if (wstrb[0]) begin
                csr_tim_config_dir_ff <= wdata[4];
            end
        end else begin
            csr_tim_config_dir_ff <= csr_tim_config_dir_ff;
        end
    end
end


//---------------------
// Bit field:
// Tim_config[5] - UD - Update counter with load value
// access: wo, hardware: o
//---------------------
reg  csr_tim_config_ud_ff;

assign csr_tim_config_rdata[5] = 1'b0;

assign csr_tim_config_ud_out = csr_tim_config_ud_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_config_ud_ff <= 1'b0;
    end else  begin
     if (csr_tim_config_wen) begin
            if (wstrb[0]) begin
                csr_tim_config_ud_ff <= wdata[5];
            end
        end else begin
            csr_tim_config_ud_ff <= csr_tim_config_ud_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - Tim_prescaler - Prescaler register (divides input clock)
//------------------------------------------------------------------------------
wire [31:0] csr_tim_prescaler_rdata;

wire csr_tim_prescaler_wen;
assign csr_tim_prescaler_wen = wen && (waddr == 32'h4);

//---------------------
// Bit field:
// Tim_prescaler[31:0] - DIV - Clock divider value
// access: wo, hardware: o
//---------------------
reg [31:0] csr_tim_prescaler_div_ff;

assign csr_tim_prescaler_rdata[31:0] = 32'h0;

assign csr_tim_prescaler_div_out = csr_tim_prescaler_div_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_prescaler_div_ff <= 32'h1b;
    end else  begin
     if (csr_tim_prescaler_wen) begin
            if (wstrb[0]) begin
                csr_tim_prescaler_div_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tim_prescaler_div_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_tim_prescaler_div_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tim_prescaler_div_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_tim_prescaler_div_ff <= csr_tim_prescaler_div_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - Tim_period - Timer period register
//------------------------------------------------------------------------------
wire [31:0] csr_tim_period_rdata;

wire csr_tim_period_wen;
assign csr_tim_period_wen = wen && (waddr == 32'h8);

//---------------------
// Bit field:
// Tim_period[31:0] - PER - Counter max value
// access: wo, hardware: o
//---------------------
reg [31:0] csr_tim_period_per_ff;

assign csr_tim_period_rdata[31:0] = 32'h0;

assign csr_tim_period_per_out = csr_tim_period_per_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_period_per_ff <= 32'hffff;
    end else  begin
     if (csr_tim_period_wen) begin
            if (wstrb[0]) begin
                csr_tim_period_per_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tim_period_per_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_tim_period_per_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tim_period_per_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_tim_period_per_ff <= csr_tim_period_per_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc] - Tim_counter - Current counter value
//------------------------------------------------------------------------------
wire [31:0] csr_tim_counter_rdata;


wire csr_tim_counter_ren;
assign csr_tim_counter_ren = ren && (raddr == 32'hc);
reg csr_tim_counter_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tim_counter_ren_ff <= 1'b0;
    end else begin
        csr_tim_counter_ren_ff <= csr_tim_counter_ren;
    end
end
//---------------------
// Bit field:
// Tim_counter[31:0] - CNT - Counter current count
// access: ro, hardware: i
//---------------------
reg [31:0] csr_tim_counter_cnt_ff;

assign csr_tim_counter_rdata[31:0] = csr_tim_counter_cnt_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tim_counter_cnt_ff <= 32'h0;
    end else  begin
              begin            csr_tim_counter_cnt_ff <= csr_tim_counter_cnt_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x10] - Tim_status - Timer status register
//------------------------------------------------------------------------------
wire [31:0] csr_tim_status_rdata;
assign csr_tim_status_rdata[31:4] = 28'h0;


wire csr_tim_status_ren;
assign csr_tim_status_ren = ren && (raddr == 32'h10);
reg csr_tim_status_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_tim_status_ren_ff <= 1'b0;
    end else begin
        csr_tim_status_ren_ff <= csr_tim_status_ren;
    end
end
//---------------------
// Bit field:
// Tim_status[0] - OF - Overflow flag
// access: ro, hardware: i
//---------------------
reg  csr_tim_status_of_ff;

assign csr_tim_status_rdata[0] = csr_tim_status_of_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tim_status_of_ff <= 1'b0;
    end else  begin
              begin            csr_tim_status_of_ff <= csr_tim_status_of_in;
        end
    end
end


//---------------------
// Bit field:
// Tim_status[1] - OP - Over period flag
// access: ro, hardware: i
//---------------------
reg  csr_tim_status_op_ff;

assign csr_tim_status_rdata[1] = csr_tim_status_op_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tim_status_op_ff <= 1'b0;
    end else  begin
              begin            csr_tim_status_op_ff <= csr_tim_status_op_in;
        end
    end
end


//---------------------
// Bit field:
// Tim_status[2] - ERR - Error flag
// access: ro, hardware: i
//---------------------
reg  csr_tim_status_err_ff;

assign csr_tim_status_rdata[2] = csr_tim_status_err_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tim_status_err_ff <= 1'b0;
    end else  begin
              begin            csr_tim_status_err_ff <= csr_tim_status_err_in;
        end
    end
end


//---------------------
// Bit field:
// Tim_status[3] - LD - Load done flag
// access: ro, hardware: i
//---------------------
reg  csr_tim_status_ld_ff;

assign csr_tim_status_rdata[3] = csr_tim_status_ld_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_tim_status_ld_ff <= 1'b0;
    end else  begin
              begin            csr_tim_status_ld_ff <= csr_tim_status_ld_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x14] - Tim_load - Value to load into counter
//------------------------------------------------------------------------------
wire [31:0] csr_tim_load_rdata;

wire csr_tim_load_wen;
assign csr_tim_load_wen = wen && (waddr == 32'h14);

//---------------------
// Bit field:
// Tim_load[31:0] - LOAD - Counter load value
// access: wo, hardware: o
//---------------------
reg [31:0] csr_tim_load_load_ff;

assign csr_tim_load_rdata[31:0] = 32'h0;

assign csr_tim_load_load_out = csr_tim_load_load_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_tim_load_load_ff <= 32'h0;
    end else  begin
     if (csr_tim_load_wen) begin
            if (wstrb[0]) begin
                csr_tim_load_load_ff[7:0] <= wdata[7:0];
            end
            if (wstrb[1]) begin
                csr_tim_load_load_ff[15:8] <= wdata[15:8];
            end
            if (wstrb[2]) begin
                csr_tim_load_load_ff[23:16] <= wdata[23:16];
            end
            if (wstrb[3]) begin
                csr_tim_load_load_ff[31:24] <= wdata[31:24];
            end
        end else begin
            csr_tim_load_load_ff <= csr_tim_load_load_ff;
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
            32'h0: rdata_ff <= csr_tim_config_rdata;
            32'h4: rdata_ff <= csr_tim_prescaler_rdata;
            32'h8: rdata_ff <= csr_tim_period_rdata;
            32'hc: rdata_ff <= csr_tim_counter_rdata;
            32'h10: rdata_ff <= csr_tim_status_rdata;
            32'h14: rdata_ff <= csr_tim_load_rdata;
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