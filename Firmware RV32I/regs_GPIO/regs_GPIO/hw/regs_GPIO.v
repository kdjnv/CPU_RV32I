// Created with Corsair v1.0.4

module regs_GPIO #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // GPIO_IO.GPIO_0
    input  csr_gpio_io_gpio_0_in,
    output  csr_gpio_io_gpio_0_out,
    // GPIO_IO.GPIO_1
    input  csr_gpio_io_gpio_1_in,
    output  csr_gpio_io_gpio_1_out,
    // GPIO_IO.GPIO_2
    input  csr_gpio_io_gpio_2_in,
    output  csr_gpio_io_gpio_2_out,
    // GPIO_IO.GPIO_3
    input  csr_gpio_io_gpio_3_in,
    output  csr_gpio_io_gpio_3_out,
    // GPIO_IO.GPIO_4
    input  csr_gpio_io_gpio_4_in,
    output  csr_gpio_io_gpio_4_out,
    // GPIO_IO.GPIO_5
    input  csr_gpio_io_gpio_5_in,
    output  csr_gpio_io_gpio_5_out,
    // GPIO_IO.GPIO_6
    input  csr_gpio_io_gpio_6_in,
    output  csr_gpio_io_gpio_6_out,
    // GPIO_IO.GPIO_7
    input  csr_gpio_io_gpio_7_in,
    output  csr_gpio_io_gpio_7_out,
    // GPIO_IO.GPIO_8
    input  csr_gpio_io_gpio_8_in,
    output  csr_gpio_io_gpio_8_out,
    // GPIO_IO.GPIO_9
    input  csr_gpio_io_gpio_9_in,
    output  csr_gpio_io_gpio_9_out,
    // GPIO_IO.GPIO_10
    input  csr_gpio_io_gpio_10_in,
    output  csr_gpio_io_gpio_10_out,
    // GPIO_IO.GPIO_11
    input  csr_gpio_io_gpio_11_in,
    output  csr_gpio_io_gpio_11_out,
    // GPIO_IO.GPIO_12
    input  csr_gpio_io_gpio_12_in,
    output  csr_gpio_io_gpio_12_out,
    // GPIO_IO.GPIO_13
    input  csr_gpio_io_gpio_13_in,
    output  csr_gpio_io_gpio_13_out,
    // GPIO_IO.GPIO_14
    input  csr_gpio_io_gpio_14_in,
    output  csr_gpio_io_gpio_14_out,
    // GPIO_IO.GPIO_15
    input  csr_gpio_io_gpio_15_in,
    output  csr_gpio_io_gpio_15_out,

    // GPIO_CONFIG.GPIO_0_CONFIG
    output  csr_gpio_config_gpio_0_config_out,
    // GPIO_CONFIG.GPIO_1_CONFIG
    output  csr_gpio_config_gpio_1_config_out,
    // GPIO_CONFIG.GPIO_2_CONFIG
    output  csr_gpio_config_gpio_2_config_out,
    // GPIO_CONFIG.GPIO_3_CONFIG
    output  csr_gpio_config_gpio_3_config_out,
    // GPIO_CONFIG.GPIO_4_CONFIG
    output  csr_gpio_config_gpio_4_config_out,
    // GPIO_CONFIG.GPIO_5_CONFIG
    output  csr_gpio_config_gpio_5_config_out,
    // GPIO_CONFIG.GPIO_6_CONFIG
    output  csr_gpio_config_gpio_6_config_out,
    // GPIO_CONFIG.GPIO_7_CONFIG
    output  csr_gpio_config_gpio_7_config_out,
    // GPIO_CONFIG.GPIO_8_CONFIG
    output  csr_gpio_config_gpio_8_config_out,
    // GPIO_CONFIG.GPIO_9_CONFIG
    output  csr_gpio_config_gpio_9_config_out,
    // GPIO_CONFIG.GPIO_10_CONFIG
    output  csr_gpio_config_gpio_10_config_out,
    // GPIO_CONFIG.GPIO_11_CONFIG
    output  csr_gpio_config_gpio_11_config_out,
    // GPIO_CONFIG.GPIO_12_CONFIG
    output  csr_gpio_config_gpio_12_config_out,
    // GPIO_CONFIG.GPIO_13_CONFIG
    output  csr_gpio_config_gpio_13_config_out,
    // GPIO_CONFIG.GPIO_14_CONFIG
    output  csr_gpio_config_gpio_14_config_out,
    // GPIO_CONFIG.GPIO_15_CONFIG
    output  csr_gpio_config_gpio_15_config_out,

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
// [0x0] - GPIO_IO - GPIO Read/Write Register
//------------------------------------------------------------------------------
wire [31:0] csr_gpio_io_rdata;
assign csr_gpio_io_rdata[31:16] = 16'h0;

wire csr_gpio_io_wen;
assign csr_gpio_io_wen = wen && (waddr == 32'h0);

wire csr_gpio_io_ren;
assign csr_gpio_io_ren = ren && (raddr == 32'h0);
reg csr_gpio_io_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_ren_ff <= 1'b0;
    end else begin
        csr_gpio_io_ren_ff <= csr_gpio_io_ren;
    end
end
//---------------------
// Bit field:
// GPIO_IO[0] - GPIO_0 - GPIO0
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_0_ff;

assign csr_gpio_io_rdata[0] = csr_gpio_io_gpio_0_ff;

assign csr_gpio_io_gpio_0_out = csr_gpio_io_gpio_0_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_0_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_0_ff <= wdata[0];
            end
        end else         begin            csr_gpio_io_gpio_0_ff <= csr_gpio_io_gpio_0_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[1] - GPIO_1 - GPIO1
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_1_ff;

assign csr_gpio_io_rdata[1] = csr_gpio_io_gpio_1_ff;

assign csr_gpio_io_gpio_1_out = csr_gpio_io_gpio_1_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_1_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_1_ff <= wdata[1];
            end
        end else         begin            csr_gpio_io_gpio_1_ff <= csr_gpio_io_gpio_1_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[2] - GPIO_2 - GPIO2
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_2_ff;

assign csr_gpio_io_rdata[2] = csr_gpio_io_gpio_2_ff;

assign csr_gpio_io_gpio_2_out = csr_gpio_io_gpio_2_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_2_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_2_ff <= wdata[2];
            end
        end else         begin            csr_gpio_io_gpio_2_ff <= csr_gpio_io_gpio_2_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[3] - GPIO_3 - GPIO3
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_3_ff;

assign csr_gpio_io_rdata[3] = csr_gpio_io_gpio_3_ff;

assign csr_gpio_io_gpio_3_out = csr_gpio_io_gpio_3_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_3_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_3_ff <= wdata[3];
            end
        end else         begin            csr_gpio_io_gpio_3_ff <= csr_gpio_io_gpio_3_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[4] - GPIO_4 - GPIO4
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_4_ff;

assign csr_gpio_io_rdata[4] = csr_gpio_io_gpio_4_ff;

assign csr_gpio_io_gpio_4_out = csr_gpio_io_gpio_4_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_4_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_4_ff <= wdata[4];
            end
        end else         begin            csr_gpio_io_gpio_4_ff <= csr_gpio_io_gpio_4_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[5] - GPIO_5 - GPIO5
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_5_ff;

assign csr_gpio_io_rdata[5] = csr_gpio_io_gpio_5_ff;

assign csr_gpio_io_gpio_5_out = csr_gpio_io_gpio_5_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_5_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_5_ff <= wdata[5];
            end
        end else         begin            csr_gpio_io_gpio_5_ff <= csr_gpio_io_gpio_5_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[6] - GPIO_6 - GPIO6
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_6_ff;

assign csr_gpio_io_rdata[6] = csr_gpio_io_gpio_6_ff;

assign csr_gpio_io_gpio_6_out = csr_gpio_io_gpio_6_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_6_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_6_ff <= wdata[6];
            end
        end else         begin            csr_gpio_io_gpio_6_ff <= csr_gpio_io_gpio_6_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[7] - GPIO_7 - GPIO7
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_7_ff;

assign csr_gpio_io_rdata[7] = csr_gpio_io_gpio_7_ff;

assign csr_gpio_io_gpio_7_out = csr_gpio_io_gpio_7_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_7_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[0]) begin
                csr_gpio_io_gpio_7_ff <= wdata[7];
            end
        end else         begin            csr_gpio_io_gpio_7_ff <= csr_gpio_io_gpio_7_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[8] - GPIO_8 - GPIO8
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_8_ff;

assign csr_gpio_io_rdata[8] = csr_gpio_io_gpio_8_ff;

assign csr_gpio_io_gpio_8_out = csr_gpio_io_gpio_8_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_8_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_8_ff <= wdata[8];
            end
        end else         begin            csr_gpio_io_gpio_8_ff <= csr_gpio_io_gpio_8_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[9] - GPIO_9 - GPIO9
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_9_ff;

assign csr_gpio_io_rdata[9] = csr_gpio_io_gpio_9_ff;

assign csr_gpio_io_gpio_9_out = csr_gpio_io_gpio_9_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_9_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_9_ff <= wdata[9];
            end
        end else         begin            csr_gpio_io_gpio_9_ff <= csr_gpio_io_gpio_9_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[10] - GPIO_10 - GPIO10
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_10_ff;

assign csr_gpio_io_rdata[10] = csr_gpio_io_gpio_10_ff;

assign csr_gpio_io_gpio_10_out = csr_gpio_io_gpio_10_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_10_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_10_ff <= wdata[10];
            end
        end else         begin            csr_gpio_io_gpio_10_ff <= csr_gpio_io_gpio_10_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[11] - GPIO_11 - GPIO11
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_11_ff;

assign csr_gpio_io_rdata[11] = csr_gpio_io_gpio_11_ff;

assign csr_gpio_io_gpio_11_out = csr_gpio_io_gpio_11_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_11_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_11_ff <= wdata[11];
            end
        end else         begin            csr_gpio_io_gpio_11_ff <= csr_gpio_io_gpio_11_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[12] - GPIO_12 - GPIO12
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_12_ff;

assign csr_gpio_io_rdata[12] = csr_gpio_io_gpio_12_ff;

assign csr_gpio_io_gpio_12_out = csr_gpio_io_gpio_12_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_12_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_12_ff <= wdata[12];
            end
        end else         begin            csr_gpio_io_gpio_12_ff <= csr_gpio_io_gpio_12_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[13] - GPIO_13 - GPIO13
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_13_ff;

assign csr_gpio_io_rdata[13] = csr_gpio_io_gpio_13_ff;

assign csr_gpio_io_gpio_13_out = csr_gpio_io_gpio_13_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_13_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_13_ff <= wdata[13];
            end
        end else         begin            csr_gpio_io_gpio_13_ff <= csr_gpio_io_gpio_13_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[14] - GPIO_14 - GPIO14
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_14_ff;

assign csr_gpio_io_rdata[14] = csr_gpio_io_gpio_14_ff;

assign csr_gpio_io_gpio_14_out = csr_gpio_io_gpio_14_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_14_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_14_ff <= wdata[14];
            end
        end else         begin            csr_gpio_io_gpio_14_ff <= csr_gpio_io_gpio_14_in;
        end
    end
end


//---------------------
// Bit field:
// GPIO_IO[15] - GPIO_15 - GPIO15
// access: rw, hardware: io
//---------------------
reg  csr_gpio_io_gpio_15_ff;

assign csr_gpio_io_rdata[15] = csr_gpio_io_gpio_15_ff;

assign csr_gpio_io_gpio_15_out = csr_gpio_io_gpio_15_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_io_gpio_15_ff <= 1'b0;
    end else  begin
     if (csr_gpio_io_wen) begin
            if (wstrb[1]) begin
                csr_gpio_io_gpio_15_ff <= wdata[15];
            end
        end else         begin            csr_gpio_io_gpio_15_ff <= csr_gpio_io_gpio_15_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - GPIO_CONFIG - GPIO_CONFIG
//------------------------------------------------------------------------------
wire [31:0] csr_gpio_config_rdata;
assign csr_gpio_config_rdata[31:16] = 16'h0;

wire csr_gpio_config_wen;
assign csr_gpio_config_wen = wen && (waddr == 32'h4);

wire csr_gpio_config_ren;
assign csr_gpio_config_ren = ren && (raddr == 32'h4);
reg csr_gpio_config_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_ren_ff <= 1'b0;
    end else begin
        csr_gpio_config_ren_ff <= csr_gpio_config_ren;
    end
end
//---------------------
// Bit field:
// GPIO_CONFIG[0] - GPIO_0_CONFIG - GPIO0_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_0_config_ff;

assign csr_gpio_config_rdata[0] = csr_gpio_config_gpio_0_config_ff;

assign csr_gpio_config_gpio_0_config_out = csr_gpio_config_gpio_0_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_0_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_0_config_ff <= wdata[0];
            end
        end else begin
            csr_gpio_config_gpio_0_config_ff <= csr_gpio_config_gpio_0_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[1] - GPIO_1_CONFIG - GPIO1_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_1_config_ff;

assign csr_gpio_config_rdata[1] = csr_gpio_config_gpio_1_config_ff;

assign csr_gpio_config_gpio_1_config_out = csr_gpio_config_gpio_1_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_1_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_1_config_ff <= wdata[1];
            end
        end else begin
            csr_gpio_config_gpio_1_config_ff <= csr_gpio_config_gpio_1_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[2] - GPIO_2_CONFIG - GPIO2_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_2_config_ff;

assign csr_gpio_config_rdata[2] = csr_gpio_config_gpio_2_config_ff;

assign csr_gpio_config_gpio_2_config_out = csr_gpio_config_gpio_2_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_2_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_2_config_ff <= wdata[2];
            end
        end else begin
            csr_gpio_config_gpio_2_config_ff <= csr_gpio_config_gpio_2_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[3] - GPIO_3_CONFIG - GPIO3_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_3_config_ff;

assign csr_gpio_config_rdata[3] = csr_gpio_config_gpio_3_config_ff;

assign csr_gpio_config_gpio_3_config_out = csr_gpio_config_gpio_3_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_3_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_3_config_ff <= wdata[3];
            end
        end else begin
            csr_gpio_config_gpio_3_config_ff <= csr_gpio_config_gpio_3_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[4] - GPIO_4_CONFIG - GPIO4_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_4_config_ff;

assign csr_gpio_config_rdata[4] = csr_gpio_config_gpio_4_config_ff;

assign csr_gpio_config_gpio_4_config_out = csr_gpio_config_gpio_4_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_4_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_4_config_ff <= wdata[4];
            end
        end else begin
            csr_gpio_config_gpio_4_config_ff <= csr_gpio_config_gpio_4_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[5] - GPIO_5_CONFIG - GPIO5_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_5_config_ff;

assign csr_gpio_config_rdata[5] = csr_gpio_config_gpio_5_config_ff;

assign csr_gpio_config_gpio_5_config_out = csr_gpio_config_gpio_5_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_5_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_5_config_ff <= wdata[5];
            end
        end else begin
            csr_gpio_config_gpio_5_config_ff <= csr_gpio_config_gpio_5_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[6] - GPIO_6_CONFIG - GPIO6_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_6_config_ff;

assign csr_gpio_config_rdata[6] = csr_gpio_config_gpio_6_config_ff;

assign csr_gpio_config_gpio_6_config_out = csr_gpio_config_gpio_6_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_6_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_6_config_ff <= wdata[6];
            end
        end else begin
            csr_gpio_config_gpio_6_config_ff <= csr_gpio_config_gpio_6_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[7] - GPIO_7_CONFIG - GPIO7_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_7_config_ff;

assign csr_gpio_config_rdata[7] = csr_gpio_config_gpio_7_config_ff;

assign csr_gpio_config_gpio_7_config_out = csr_gpio_config_gpio_7_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_7_config_ff <= 1'b0;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[0]) begin
                csr_gpio_config_gpio_7_config_ff <= wdata[7];
            end
        end else begin
            csr_gpio_config_gpio_7_config_ff <= csr_gpio_config_gpio_7_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[8] - GPIO_8_CONFIG - GPIO8_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_8_config_ff;

assign csr_gpio_config_rdata[8] = csr_gpio_config_gpio_8_config_ff;

assign csr_gpio_config_gpio_8_config_out = csr_gpio_config_gpio_8_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_8_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_8_config_ff <= wdata[8];
            end
        end else begin
            csr_gpio_config_gpio_8_config_ff <= csr_gpio_config_gpio_8_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[9] - GPIO_9_CONFIG - GPIO9_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_9_config_ff;

assign csr_gpio_config_rdata[9] = csr_gpio_config_gpio_9_config_ff;

assign csr_gpio_config_gpio_9_config_out = csr_gpio_config_gpio_9_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_9_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_9_config_ff <= wdata[9];
            end
        end else begin
            csr_gpio_config_gpio_9_config_ff <= csr_gpio_config_gpio_9_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[10] - GPIO_10_CONFIG - GPIO10_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_10_config_ff;

assign csr_gpio_config_rdata[10] = csr_gpio_config_gpio_10_config_ff;

assign csr_gpio_config_gpio_10_config_out = csr_gpio_config_gpio_10_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_10_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_10_config_ff <= wdata[10];
            end
        end else begin
            csr_gpio_config_gpio_10_config_ff <= csr_gpio_config_gpio_10_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[11] - GPIO_11_CONFIG - GPIO11_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_11_config_ff;

assign csr_gpio_config_rdata[11] = csr_gpio_config_gpio_11_config_ff;

assign csr_gpio_config_gpio_11_config_out = csr_gpio_config_gpio_11_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_11_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_11_config_ff <= wdata[11];
            end
        end else begin
            csr_gpio_config_gpio_11_config_ff <= csr_gpio_config_gpio_11_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[12] - GPIO_12_CONFIG - GPIO12_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_12_config_ff;

assign csr_gpio_config_rdata[12] = csr_gpio_config_gpio_12_config_ff;

assign csr_gpio_config_gpio_12_config_out = csr_gpio_config_gpio_12_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_12_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_12_config_ff <= wdata[12];
            end
        end else begin
            csr_gpio_config_gpio_12_config_ff <= csr_gpio_config_gpio_12_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[13] - GPIO_13_CONFIG - GPIO13_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_13_config_ff;

assign csr_gpio_config_rdata[13] = csr_gpio_config_gpio_13_config_ff;

assign csr_gpio_config_gpio_13_config_out = csr_gpio_config_gpio_13_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_13_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_13_config_ff <= wdata[13];
            end
        end else begin
            csr_gpio_config_gpio_13_config_ff <= csr_gpio_config_gpio_13_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[14] - GPIO_14_CONFIG - GPIO14_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_14_config_ff;

assign csr_gpio_config_rdata[14] = csr_gpio_config_gpio_14_config_ff;

assign csr_gpio_config_gpio_14_config_out = csr_gpio_config_gpio_14_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_14_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_14_config_ff <= wdata[14];
            end
        end else begin
            csr_gpio_config_gpio_14_config_ff <= csr_gpio_config_gpio_14_config_ff;
        end
    end
end


//---------------------
// Bit field:
// GPIO_CONFIG[15] - GPIO_15_CONFIG - GPIO15_CONFIG
// access: rw, hardware: o
//---------------------
reg  csr_gpio_config_gpio_15_config_ff;

assign csr_gpio_config_rdata[15] = csr_gpio_config_gpio_15_config_ff;

assign csr_gpio_config_gpio_15_config_out = csr_gpio_config_gpio_15_config_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_gpio_config_gpio_15_config_ff <= 1'b1;
    end else  begin
     if (csr_gpio_config_wen) begin
            if (wstrb[1]) begin
                csr_gpio_config_gpio_15_config_ff <= wdata[15];
            end
        end else begin
            csr_gpio_config_gpio_15_config_ff <= csr_gpio_config_gpio_15_config_ff;
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
            32'h0: rdata_ff <= csr_gpio_io_rdata;
            32'h4: rdata_ff <= csr_gpio_config_rdata;
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