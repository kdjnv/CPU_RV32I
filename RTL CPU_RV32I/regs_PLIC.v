// Created with Corsair v1.0.4

module regs_PLIC #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst_n,
    // P_PRIORITY1.PRIORITY1
    output [2:0] csr_p_priority1_priority1_out,

    // P_PRIORITY2.PRIORITY2
    output [2:0] csr_p_priority2_priority2_out,

    // P_PRIORITY3.PRIORITY3
    output [2:0] csr_p_priority3_priority3_out,

    // P_PRIORITY4.PRIORITY4
    output [2:0] csr_p_priority4_priority4_out,

    // P_PRIORITY5.PRIORITY5
    output [2:0] csr_p_priority5_priority5_out,

    // P_PRIORITY6.PRIORITY6
    output [2:0] csr_p_priority6_priority6_out,

    // P_PRIORITY7.PRIORITY7
    output [2:0] csr_p_priority7_priority7_out,

    // P_PRIORITY8.PRIORITY8
    output [2:0] csr_p_priority8_priority8_out,

    // P_PRIORITY9.PRIORITY9
    output [2:0] csr_p_priority9_priority9_out,

    // P_PRIORITY10.PRIORITY10
    output [2:0] csr_p_priority10_priority10_out,

    // P_PRIORITY11.PRIORITY11
    output [2:0] csr_p_priority11_priority11_out,

    // P_PRIORITY12.PRIORITY12
    output [2:0] csr_p_priority12_priority12_out,

    // P_PRIORITY13.PRIORITY13
    output [2:0] csr_p_priority13_priority13_out,

    // P_PRIORITY14.PRIORITY14
    output [2:0] csr_p_priority14_priority14_out,

    // P_PRIORITY15.PRIORITY15
    output [2:0] csr_p_priority15_priority15_out,

    // P_PRIORITY16.PRIORITY16
    output [2:0] csr_p_priority16_priority16_out,

    // P_PRIORITY17.PRIORITY17
    output [2:0] csr_p_priority17_priority17_out,

    // P_PRIORITY18.PRIORITY18
    output [2:0] csr_p_priority18_priority18_out,

    // P_PRIORITY19.PRIORITY19
    output [2:0] csr_p_priority19_priority19_out,

    // P_PRIORITY20.PRIORITY20
    output [2:0] csr_p_priority20_priority20_out,

    // P_PRIORITY21.PRIORITY21
    output [2:0] csr_p_priority21_priority21_out,

    // P_PRIORITY22.PRIORITY22
    output [2:0] csr_p_priority22_priority22_out,

    // P_PRIORITY23.PRIORITY23
    output [2:0] csr_p_priority23_priority23_out,

    // P_PRIORITY24.PRIORITY24
    output [2:0] csr_p_priority24_priority24_out,

    // P_PRIORITY25.PRIORITY25
    output [2:0] csr_p_priority25_priority25_out,

    // P_PRIORITY26.PRIORITY26
    output [2:0] csr_p_priority26_priority26_out,

    // P_PRIORITY27.PRIORITY27
    output [2:0] csr_p_priority27_priority27_out,

    // P_PRIORITY28.PRIORITY28
    output [2:0] csr_p_priority28_priority28_out,

    // P_PRIORITY29.PRIORITY29
    output [2:0] csr_p_priority29_priority29_out,

    // P_PRIORITY30.PRIORITY30
    output [2:0] csr_p_priority30_priority30_out,

    // P_PRIORITY31.PRIORITY31
    output [2:0] csr_p_priority31_priority31_out,

    // P_PRIORITY32.PRIORITY32
    output [2:0] csr_p_priority32_priority32_out,

    // P_PRIORITY33.PRIORITY33
    output [2:0] csr_p_priority33_priority33_out,

    // P_PRIORITY34.PRIORITY34
    output [2:0] csr_p_priority34_priority34_out,

    // P_PRIORITY35.PRIORITY35
    output [2:0] csr_p_priority35_priority35_out,

    // P_PRIORITY36.PRIORITY36
    output [2:0] csr_p_priority36_priority36_out,

    // P_PRIORITY37.PRIORITY37
    output [2:0] csr_p_priority37_priority37_out,

    // P_PRIORITY38.PRIORITY38
    output [2:0] csr_p_priority38_priority38_out,

    // P_PRIORITY39.PRIORITY39
    output [2:0] csr_p_priority39_priority39_out,

    // P_PRIORITY40.PRIORITY40
    output [2:0] csr_p_priority40_priority40_out,

    // P_PRIORITY41.PRIORITY41
    output [2:0] csr_p_priority41_priority41_out,

    // P_PRIORITY42.PRIORITY42
    output [2:0] csr_p_priority42_priority42_out,

    // P_PRIORITY43.PRIORITY43
    output [2:0] csr_p_priority43_priority43_out,

    // P_PRIORITY44.PRIORITY44
    output [2:0] csr_p_priority44_priority44_out,

    // P_PRIORITY45.PRIORITY45
    output [2:0] csr_p_priority45_priority45_out,

    // P_PRIORITY46.PRIORITY46
    output [2:0] csr_p_priority46_priority46_out,

    // P_PRIORITY47.PRIORITY47
    output [2:0] csr_p_priority47_priority47_out,

    // P_PRIORITY48.PRIORITY48
    output [2:0] csr_p_priority48_priority48_out,

    // P_PRIORITY49.PRIORITY49
    output [2:0] csr_p_priority49_priority49_out,

    // P_PRIORITY50.PRIORITY50
    output [2:0] csr_p_priority50_priority50_out,

    // P_PRIORITY51.PRIORITY51
    output [2:0] csr_p_priority51_priority51_out,

    // P_PRIORITY52.PRIORITY52
    output [2:0] csr_p_priority52_priority52_out,

    // P_PRIORITY53.PRIORITY53
    output [2:0] csr_p_priority53_priority53_out,

    // P_PRIORITY54.PRIORITY54
    output [2:0] csr_p_priority54_priority54_out,

    // P_PRIORITY55.PRIORITY55
    output [2:0] csr_p_priority55_priority55_out,

    // P_PRIORITY56.PRIORITY56
    output [2:0] csr_p_priority56_priority56_out,

    // P_PRIORITY57.PRIORITY57
    output [2:0] csr_p_priority57_priority57_out,

    // P_PRIORITY58.PRIORITY58
    output [2:0] csr_p_priority58_priority58_out,

    // P_PRIORITY59.PRIORITY59
    output [2:0] csr_p_priority59_priority59_out,

    // P_PRIORITY60.PRIORITY60
    output [2:0] csr_p_priority60_priority60_out,

    // P_PRIORITY61.PRIORITY61
    output [2:0] csr_p_priority61_priority61_out,

    // P_PRIORITY62.PRIORITY62
    output [2:0] csr_p_priority62_priority62_out,

    // P_PRIORITY63.PRIORITY63
    output [2:0] csr_p_priority63_priority63_out,

    // P_PRIORITY64.PRIORITY64
    output [2:0] csr_p_priority64_priority64_out,

    // P_PENDING0.PENDING_IRQ1
    input  csr_p_pending0_pending_irq1_in,
    // P_PENDING0.PENDING_IRQ2
    input  csr_p_pending0_pending_irq2_in,
    // P_PENDING0.PENDING_IRQ3
    input  csr_p_pending0_pending_irq3_in,
    // P_PENDING0.PENDING_IRQ4
    input  csr_p_pending0_pending_irq4_in,
    // P_PENDING0.PENDING_IRQ5
    input  csr_p_pending0_pending_irq5_in,
    // P_PENDING0.PENDING_IRQ6
    input  csr_p_pending0_pending_irq6_in,
    // P_PENDING0.PENDING_IRQ7
    input  csr_p_pending0_pending_irq7_in,
    // P_PENDING0.PENDING_IRQ8
    input  csr_p_pending0_pending_irq8_in,
    // P_PENDING0.PENDING_IRQ9
    input  csr_p_pending0_pending_irq9_in,
    // P_PENDING0.PENDING_IRQ10
    input  csr_p_pending0_pending_irq10_in,
    // P_PENDING0.PENDING_IRQ11
    input  csr_p_pending0_pending_irq11_in,
    // P_PENDING0.PENDING_IRQ12
    input  csr_p_pending0_pending_irq12_in,
    // P_PENDING0.PENDING_IRQ13
    input  csr_p_pending0_pending_irq13_in,
    // P_PENDING0.PENDING_IRQ14
    input  csr_p_pending0_pending_irq14_in,
    // P_PENDING0.PENDING_IRQ15
    input  csr_p_pending0_pending_irq15_in,
    // P_PENDING0.PENDING_IRQ16
    input  csr_p_pending0_pending_irq16_in,
    // P_PENDING0.PENDING_IRQ17
    input  csr_p_pending0_pending_irq17_in,
    // P_PENDING0.PENDING_IRQ18
    input  csr_p_pending0_pending_irq18_in,
    // P_PENDING0.PENDING_IRQ19
    input  csr_p_pending0_pending_irq19_in,
    // P_PENDING0.PENDING_IRQ20
    input  csr_p_pending0_pending_irq20_in,
    // P_PENDING0.PENDING_IRQ21
    input  csr_p_pending0_pending_irq21_in,
    // P_PENDING0.PENDING_IRQ22
    input  csr_p_pending0_pending_irq22_in,
    // P_PENDING0.PENDING_IRQ23
    input  csr_p_pending0_pending_irq23_in,
    // P_PENDING0.PENDING_IRQ24
    input  csr_p_pending0_pending_irq24_in,
    // P_PENDING0.PENDING_IRQ25
    input  csr_p_pending0_pending_irq25_in,
    // P_PENDING0.PENDING_IRQ26
    input  csr_p_pending0_pending_irq26_in,
    // P_PENDING0.PENDING_IRQ27
    input  csr_p_pending0_pending_irq27_in,
    // P_PENDING0.PENDING_IRQ28
    input  csr_p_pending0_pending_irq28_in,
    // P_PENDING0.PENDING_IRQ29
    input  csr_p_pending0_pending_irq29_in,
    // P_PENDING0.PENDING_IRQ30
    input  csr_p_pending0_pending_irq30_in,
    // P_PENDING0.PENDING_IRQ31
    input  csr_p_pending0_pending_irq31_in,

    // P_PENDING1.PENDING_IRQ32
    input  csr_p_pending1_pending_irq32_in,
    // P_PENDING1.PENDING_IRQ33
    input  csr_p_pending1_pending_irq33_in,
    // P_PENDING1.PENDING_IRQ34
    input  csr_p_pending1_pending_irq34_in,
    // P_PENDING1.PENDING_IRQ35
    input  csr_p_pending1_pending_irq35_in,
    // P_PENDING1.PENDING_IRQ36
    input  csr_p_pending1_pending_irq36_in,
    // P_PENDING1.PENDING_IRQ37
    input  csr_p_pending1_pending_irq37_in,
    // P_PENDING1.PENDING_IRQ38
    input  csr_p_pending1_pending_irq38_in,
    // P_PENDING1.PENDING_IRQ39
    input  csr_p_pending1_pending_irq39_in,
    // P_PENDING1.PENDING_IRQ40
    input  csr_p_pending1_pending_irq40_in,
    // P_PENDING1.PENDING_IRQ41
    input  csr_p_pending1_pending_irq41_in,
    // P_PENDING1.PENDING_IRQ42
    input  csr_p_pending1_pending_irq42_in,
    // P_PENDING1.PENDING_IRQ43
    input  csr_p_pending1_pending_irq43_in,
    // P_PENDING1.PENDING_IRQ44
    input  csr_p_pending1_pending_irq44_in,
    // P_PENDING1.PENDING_IRQ45
    input  csr_p_pending1_pending_irq45_in,
    // P_PENDING1.PENDING_IRQ46
    input  csr_p_pending1_pending_irq46_in,
    // P_PENDING1.PENDING_IRQ47
    input  csr_p_pending1_pending_irq47_in,
    // P_PENDING1.PENDING_IRQ48
    input  csr_p_pending1_pending_irq48_in,
    // P_PENDING1.PENDING_IRQ49
    input  csr_p_pending1_pending_irq49_in,
    // P_PENDING1.PENDING_IRQ50
    input  csr_p_pending1_pending_irq50_in,
    // P_PENDING1.PENDING_IRQ51
    input  csr_p_pending1_pending_irq51_in,
    // P_PENDING1.PENDING_IRQ52
    input  csr_p_pending1_pending_irq52_in,
    // P_PENDING1.PENDING_IRQ53
    input  csr_p_pending1_pending_irq53_in,
    // P_PENDING1.PENDING_IRQ54
    input  csr_p_pending1_pending_irq54_in,
    // P_PENDING1.PENDING_IRQ55
    input  csr_p_pending1_pending_irq55_in,
    // P_PENDING1.PENDING_IRQ56
    input  csr_p_pending1_pending_irq56_in,
    // P_PENDING1.PENDING_IRQ57
    input  csr_p_pending1_pending_irq57_in,
    // P_PENDING1.PENDING_IRQ58
    input  csr_p_pending1_pending_irq58_in,
    // P_PENDING1.PENDING_IRQ59
    input  csr_p_pending1_pending_irq59_in,
    // P_PENDING1.PENDING_IRQ60
    input  csr_p_pending1_pending_irq60_in,
    // P_PENDING1.PENDING_IRQ61
    input  csr_p_pending1_pending_irq61_in,
    // P_PENDING1.PENDING_IRQ62
    input  csr_p_pending1_pending_irq62_in,

    // P_ENABLE0.ENABLE_IRQ1
    output  csr_p_enable0_enable_irq1_out,
    // P_ENABLE0.ENABLE_IRQ2
    output  csr_p_enable0_enable_irq2_out,
    // P_ENABLE0.ENABLE_IRQ3
    output  csr_p_enable0_enable_irq3_out,
    // P_ENABLE0.ENABLE_IRQ4
    output  csr_p_enable0_enable_irq4_out,
    // P_ENABLE0.ENABLE_IRQ5
    output  csr_p_enable0_enable_irq5_out,
    // P_ENABLE0.ENABLE_IRQ6
    output  csr_p_enable0_enable_irq6_out,
    // P_ENABLE0.ENABLE_IRQ7
    output  csr_p_enable0_enable_irq7_out,
    // P_ENABLE0.ENABLE_IRQ8
    output  csr_p_enable0_enable_irq8_out,
    // P_ENABLE0.ENABLE_IRQ9
    output  csr_p_enable0_enable_irq9_out,
    // P_ENABLE0.ENABLE_IRQ10
    output  csr_p_enable0_enable_irq10_out,
    // P_ENABLE0.ENABLE_IRQ11
    output  csr_p_enable0_enable_irq11_out,
    // P_ENABLE0.ENABLE_IRQ12
    output  csr_p_enable0_enable_irq12_out,
    // P_ENABLE0.ENABLE_IRQ13
    output  csr_p_enable0_enable_irq13_out,
    // P_ENABLE0.ENABLE_IRQ14
    output  csr_p_enable0_enable_irq14_out,
    // P_ENABLE0.ENABLE_IRQ15
    output  csr_p_enable0_enable_irq15_out,
    // P_ENABLE0.ENABLE_IRQ16
    output  csr_p_enable0_enable_irq16_out,
    // P_ENABLE0.ENABLE_IRQ17
    output  csr_p_enable0_enable_irq17_out,
    // P_ENABLE0.ENABLE_IRQ18
    output  csr_p_enable0_enable_irq18_out,
    // P_ENABLE0.ENABLE_IRQ19
    output  csr_p_enable0_enable_irq19_out,
    // P_ENABLE0.ENABLE_IRQ20
    output  csr_p_enable0_enable_irq20_out,
    // P_ENABLE0.ENABLE_IRQ21
    output  csr_p_enable0_enable_irq21_out,
    // P_ENABLE0.ENABLE_IRQ22
    output  csr_p_enable0_enable_irq22_out,
    // P_ENABLE0.ENABLE_IRQ23
    output  csr_p_enable0_enable_irq23_out,
    // P_ENABLE0.ENABLE_IRQ24
    output  csr_p_enable0_enable_irq24_out,
    // P_ENABLE0.ENABLE_IRQ25
    output  csr_p_enable0_enable_irq25_out,
    // P_ENABLE0.ENABLE_IRQ26
    output  csr_p_enable0_enable_irq26_out,
    // P_ENABLE0.ENABLE_IRQ27
    output  csr_p_enable0_enable_irq27_out,
    // P_ENABLE0.ENABLE_IRQ28
    output  csr_p_enable0_enable_irq28_out,
    // P_ENABLE0.ENABLE_IRQ29
    output  csr_p_enable0_enable_irq29_out,
    // P_ENABLE0.ENABLE_IRQ30
    output  csr_p_enable0_enable_irq30_out,
    // P_ENABLE0.ENABLE_IRQ31
    output  csr_p_enable0_enable_irq31_out,

    // P_ENABLE1.ENABLE_IRQ32
    output  csr_p_enable1_enable_irq32_out,
    // P_ENABLE1.ENABLE_IRQ33
    output  csr_p_enable1_enable_irq33_out,
    // P_ENABLE1.ENABLE_IRQ34
    output  csr_p_enable1_enable_irq34_out,
    // P_ENABLE1.ENABLE_IRQ35
    output  csr_p_enable1_enable_irq35_out,
    // P_ENABLE1.ENABLE_IRQ36
    output  csr_p_enable1_enable_irq36_out,
    // P_ENABLE1.ENABLE_IRQ37
    output  csr_p_enable1_enable_irq37_out,
    // P_ENABLE1.ENABLE_IRQ38
    output  csr_p_enable1_enable_irq38_out,
    // P_ENABLE1.ENABLE_IRQ39
    output  csr_p_enable1_enable_irq39_out,
    // P_ENABLE1.ENABLE_IRQ40
    output  csr_p_enable1_enable_irq40_out,
    // P_ENABLE1.ENABLE_IRQ41
    output  csr_p_enable1_enable_irq41_out,
    // P_ENABLE1.ENABLE_IRQ42
    output  csr_p_enable1_enable_irq42_out,
    // P_ENABLE1.ENABLE_IRQ43
    output  csr_p_enable1_enable_irq43_out,
    // P_ENABLE1.ENABLE_IRQ44
    output  csr_p_enable1_enable_irq44_out,
    // P_ENABLE1.ENABLE_IRQ45
    output  csr_p_enable1_enable_irq45_out,
    // P_ENABLE1.ENABLE_IRQ46
    output  csr_p_enable1_enable_irq46_out,
    // P_ENABLE1.ENABLE_IRQ47
    output  csr_p_enable1_enable_irq47_out,
    // P_ENABLE1.ENABLE_IRQ48
    output  csr_p_enable1_enable_irq48_out,
    // P_ENABLE1.ENABLE_IRQ49
    output  csr_p_enable1_enable_irq49_out,
    // P_ENABLE1.ENABLE_IRQ50
    output  csr_p_enable1_enable_irq50_out,
    // P_ENABLE1.ENABLE_IRQ51
    output  csr_p_enable1_enable_irq51_out,
    // P_ENABLE1.ENABLE_IRQ52
    output  csr_p_enable1_enable_irq52_out,
    // P_ENABLE1.ENABLE_IRQ53
    output  csr_p_enable1_enable_irq53_out,
    // P_ENABLE1.ENABLE_IRQ54
    output  csr_p_enable1_enable_irq54_out,
    // P_ENABLE1.ENABLE_IRQ55
    output  csr_p_enable1_enable_irq55_out,
    // P_ENABLE1.ENABLE_IRQ56
    output  csr_p_enable1_enable_irq56_out,
    // P_ENABLE1.ENABLE_IRQ57
    output  csr_p_enable1_enable_irq57_out,
    // P_ENABLE1.ENABLE_IRQ58
    output  csr_p_enable1_enable_irq58_out,
    // P_ENABLE1.ENABLE_IRQ59
    output  csr_p_enable1_enable_irq59_out,
    // P_ENABLE1.ENABLE_IRQ60
    output  csr_p_enable1_enable_irq60_out,
    // P_ENABLE1.ENABLE_IRQ61
    output  csr_p_enable1_enable_irq61_out,
    // P_ENABLE1.ENABLE_IRQ62
    output  csr_p_enable1_enable_irq62_out,

    // P_THRESHOLD.THRESHOLD
    output [2:0] csr_p_threshold_threshold_out,

    // P_CLAIMCOMPLETE.CP
    input [5:0] csr_p_claimcomplete_cp_in,
    output [5:0] csr_p_claimcomplete_cp_out,

    output  reg         complete_flag,
    output  reg [5:0]   complete_id,
    output  reg         claim_flag,

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
// [0x4] - P_PRIORITY1 - Priority Register IRQ1 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority1_rdata;
assign csr_p_priority1_rdata[31:3] = 29'h0;

wire csr_p_priority1_wen;
assign csr_p_priority1_wen = wen && (waddr == 32'h4);

wire csr_p_priority1_ren;
assign csr_p_priority1_ren = ren && (raddr == 32'h4);
reg csr_p_priority1_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority1_ren_ff <= 1'b0;
    end else begin
        csr_p_priority1_ren_ff <= csr_p_priority1_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY1[2:0] - PRIORITY1 - Priority level of IRQ1. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority1_priority1_ff;

assign csr_p_priority1_rdata[2:0] = csr_p_priority1_priority1_ff;

assign csr_p_priority1_priority1_out = csr_p_priority1_priority1_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority1_priority1_ff <= 3'h0;
    end else  begin
     if (csr_p_priority1_wen) begin
            if (wstrb[0]) begin
                csr_p_priority1_priority1_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority1_priority1_ff <= csr_p_priority1_priority1_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - P_PRIORITY2 - Priority Register IRQ2 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority2_rdata;
assign csr_p_priority2_rdata[31:3] = 29'h0;

wire csr_p_priority2_wen;
assign csr_p_priority2_wen = wen && (waddr == 32'h8);

wire csr_p_priority2_ren;
assign csr_p_priority2_ren = ren && (raddr == 32'h8);
reg csr_p_priority2_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority2_ren_ff <= 1'b0;
    end else begin
        csr_p_priority2_ren_ff <= csr_p_priority2_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY2[2:0] - PRIORITY2 - Priority level of IRQ2. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority2_priority2_ff;

assign csr_p_priority2_rdata[2:0] = csr_p_priority2_priority2_ff;

assign csr_p_priority2_priority2_out = csr_p_priority2_priority2_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority2_priority2_ff <= 3'h0;
    end else  begin
     if (csr_p_priority2_wen) begin
            if (wstrb[0]) begin
                csr_p_priority2_priority2_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority2_priority2_ff <= csr_p_priority2_priority2_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc] - P_PRIORITY3 - Priority Register IRQ3 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority3_rdata;
assign csr_p_priority3_rdata[31:3] = 29'h0;

wire csr_p_priority3_wen;
assign csr_p_priority3_wen = wen && (waddr == 32'hc);

wire csr_p_priority3_ren;
assign csr_p_priority3_ren = ren && (raddr == 32'hc);
reg csr_p_priority3_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority3_ren_ff <= 1'b0;
    end else begin
        csr_p_priority3_ren_ff <= csr_p_priority3_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY3[2:0] - PRIORITY3 - Priority level of IRQ3. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority3_priority3_ff;

assign csr_p_priority3_rdata[2:0] = csr_p_priority3_priority3_ff;

assign csr_p_priority3_priority3_out = csr_p_priority3_priority3_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority3_priority3_ff <= 3'h0;
    end else  begin
     if (csr_p_priority3_wen) begin
            if (wstrb[0]) begin
                csr_p_priority3_priority3_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority3_priority3_ff <= csr_p_priority3_priority3_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x10] - P_PRIORITY4 - Priority Register IRQ4 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority4_rdata;
assign csr_p_priority4_rdata[31:3] = 29'h0;

wire csr_p_priority4_wen;
assign csr_p_priority4_wen = wen && (waddr == 32'h10);

wire csr_p_priority4_ren;
assign csr_p_priority4_ren = ren && (raddr == 32'h10);
reg csr_p_priority4_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority4_ren_ff <= 1'b0;
    end else begin
        csr_p_priority4_ren_ff <= csr_p_priority4_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY4[2:0] - PRIORITY4 - Priority level of IRQ4. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority4_priority4_ff;

assign csr_p_priority4_rdata[2:0] = csr_p_priority4_priority4_ff;

assign csr_p_priority4_priority4_out = csr_p_priority4_priority4_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority4_priority4_ff <= 3'h0;
    end else  begin
     if (csr_p_priority4_wen) begin
            if (wstrb[0]) begin
                csr_p_priority4_priority4_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority4_priority4_ff <= csr_p_priority4_priority4_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x14] - P_PRIORITY5 - Priority Register IRQ5 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority5_rdata;
assign csr_p_priority5_rdata[31:3] = 29'h0;

wire csr_p_priority5_wen;
assign csr_p_priority5_wen = wen && (waddr == 32'h14);

wire csr_p_priority5_ren;
assign csr_p_priority5_ren = ren && (raddr == 32'h14);
reg csr_p_priority5_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority5_ren_ff <= 1'b0;
    end else begin
        csr_p_priority5_ren_ff <= csr_p_priority5_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY5[2:0] - PRIORITY5 - Priority level of IRQ5. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority5_priority5_ff;

assign csr_p_priority5_rdata[2:0] = csr_p_priority5_priority5_ff;

assign csr_p_priority5_priority5_out = csr_p_priority5_priority5_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority5_priority5_ff <= 3'h0;
    end else  begin
     if (csr_p_priority5_wen) begin
            if (wstrb[0]) begin
                csr_p_priority5_priority5_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority5_priority5_ff <= csr_p_priority5_priority5_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x18] - P_PRIORITY6 - Priority Register IRQ6 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority6_rdata;
assign csr_p_priority6_rdata[31:3] = 29'h0;

wire csr_p_priority6_wen;
assign csr_p_priority6_wen = wen && (waddr == 32'h18);

wire csr_p_priority6_ren;
assign csr_p_priority6_ren = ren && (raddr == 32'h18);
reg csr_p_priority6_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority6_ren_ff <= 1'b0;
    end else begin
        csr_p_priority6_ren_ff <= csr_p_priority6_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY6[2:0] - PRIORITY6 - Priority level of IRQ6. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority6_priority6_ff;

assign csr_p_priority6_rdata[2:0] = csr_p_priority6_priority6_ff;

assign csr_p_priority6_priority6_out = csr_p_priority6_priority6_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority6_priority6_ff <= 3'h0;
    end else  begin
     if (csr_p_priority6_wen) begin
            if (wstrb[0]) begin
                csr_p_priority6_priority6_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority6_priority6_ff <= csr_p_priority6_priority6_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x1c] - P_PRIORITY7 - Priority Register IRQ7 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority7_rdata;
assign csr_p_priority7_rdata[31:3] = 29'h0;

wire csr_p_priority7_wen;
assign csr_p_priority7_wen = wen && (waddr == 32'h1c);

wire csr_p_priority7_ren;
assign csr_p_priority7_ren = ren && (raddr == 32'h1c);
reg csr_p_priority7_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority7_ren_ff <= 1'b0;
    end else begin
        csr_p_priority7_ren_ff <= csr_p_priority7_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY7[2:0] - PRIORITY7 - Priority level of IRQ7. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority7_priority7_ff;

assign csr_p_priority7_rdata[2:0] = csr_p_priority7_priority7_ff;

assign csr_p_priority7_priority7_out = csr_p_priority7_priority7_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority7_priority7_ff <= 3'h0;
    end else  begin
     if (csr_p_priority7_wen) begin
            if (wstrb[0]) begin
                csr_p_priority7_priority7_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority7_priority7_ff <= csr_p_priority7_priority7_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x20] - P_PRIORITY8 - Priority Register IRQ8 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority8_rdata;
assign csr_p_priority8_rdata[31:3] = 29'h0;

wire csr_p_priority8_wen;
assign csr_p_priority8_wen = wen && (waddr == 32'h20);

wire csr_p_priority8_ren;
assign csr_p_priority8_ren = ren && (raddr == 32'h20);
reg csr_p_priority8_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority8_ren_ff <= 1'b0;
    end else begin
        csr_p_priority8_ren_ff <= csr_p_priority8_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY8[2:0] - PRIORITY8 - Priority level of IRQ8. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority8_priority8_ff;

assign csr_p_priority8_rdata[2:0] = csr_p_priority8_priority8_ff;

assign csr_p_priority8_priority8_out = csr_p_priority8_priority8_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority8_priority8_ff <= 3'h0;
    end else  begin
     if (csr_p_priority8_wen) begin
            if (wstrb[0]) begin
                csr_p_priority8_priority8_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority8_priority8_ff <= csr_p_priority8_priority8_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x24] - P_PRIORITY9 - Priority Register IRQ9 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority9_rdata;
assign csr_p_priority9_rdata[31:3] = 29'h0;

wire csr_p_priority9_wen;
assign csr_p_priority9_wen = wen && (waddr == 32'h24);

wire csr_p_priority9_ren;
assign csr_p_priority9_ren = ren && (raddr == 32'h24);
reg csr_p_priority9_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority9_ren_ff <= 1'b0;
    end else begin
        csr_p_priority9_ren_ff <= csr_p_priority9_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY9[2:0] - PRIORITY9 - Priority level of IRQ9. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority9_priority9_ff;

assign csr_p_priority9_rdata[2:0] = csr_p_priority9_priority9_ff;

assign csr_p_priority9_priority9_out = csr_p_priority9_priority9_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority9_priority9_ff <= 3'h0;
    end else  begin
     if (csr_p_priority9_wen) begin
            if (wstrb[0]) begin
                csr_p_priority9_priority9_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority9_priority9_ff <= csr_p_priority9_priority9_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x28] - P_PRIORITY10 - Priority Register IRQ10 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority10_rdata;
assign csr_p_priority10_rdata[31:3] = 29'h0;

wire csr_p_priority10_wen;
assign csr_p_priority10_wen = wen && (waddr == 32'h28);

wire csr_p_priority10_ren;
assign csr_p_priority10_ren = ren && (raddr == 32'h28);
reg csr_p_priority10_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority10_ren_ff <= 1'b0;
    end else begin
        csr_p_priority10_ren_ff <= csr_p_priority10_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY10[2:0] - PRIORITY10 - Priority level of IRQ10. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority10_priority10_ff;

assign csr_p_priority10_rdata[2:0] = csr_p_priority10_priority10_ff;

assign csr_p_priority10_priority10_out = csr_p_priority10_priority10_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority10_priority10_ff <= 3'h0;
    end else  begin
     if (csr_p_priority10_wen) begin
            if (wstrb[0]) begin
                csr_p_priority10_priority10_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority10_priority10_ff <= csr_p_priority10_priority10_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x2c] - P_PRIORITY11 - Priority Register IRQ11 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority11_rdata;
assign csr_p_priority11_rdata[31:3] = 29'h0;

wire csr_p_priority11_wen;
assign csr_p_priority11_wen = wen && (waddr == 32'h2c);

wire csr_p_priority11_ren;
assign csr_p_priority11_ren = ren && (raddr == 32'h2c);
reg csr_p_priority11_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority11_ren_ff <= 1'b0;
    end else begin
        csr_p_priority11_ren_ff <= csr_p_priority11_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY11[2:0] - PRIORITY11 - Priority level of IRQ11. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority11_priority11_ff;

assign csr_p_priority11_rdata[2:0] = csr_p_priority11_priority11_ff;

assign csr_p_priority11_priority11_out = csr_p_priority11_priority11_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority11_priority11_ff <= 3'h0;
    end else  begin
     if (csr_p_priority11_wen) begin
            if (wstrb[0]) begin
                csr_p_priority11_priority11_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority11_priority11_ff <= csr_p_priority11_priority11_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x30] - P_PRIORITY12 - Priority Register IRQ12 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority12_rdata;
assign csr_p_priority12_rdata[31:3] = 29'h0;

wire csr_p_priority12_wen;
assign csr_p_priority12_wen = wen && (waddr == 32'h30);

wire csr_p_priority12_ren;
assign csr_p_priority12_ren = ren && (raddr == 32'h30);
reg csr_p_priority12_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority12_ren_ff <= 1'b0;
    end else begin
        csr_p_priority12_ren_ff <= csr_p_priority12_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY12[2:0] - PRIORITY12 - Priority level of IRQ12. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority12_priority12_ff;

assign csr_p_priority12_rdata[2:0] = csr_p_priority12_priority12_ff;

assign csr_p_priority12_priority12_out = csr_p_priority12_priority12_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority12_priority12_ff <= 3'h0;
    end else  begin
     if (csr_p_priority12_wen) begin
            if (wstrb[0]) begin
                csr_p_priority12_priority12_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority12_priority12_ff <= csr_p_priority12_priority12_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x34] - P_PRIORITY13 - Priority Register IRQ13 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority13_rdata;
assign csr_p_priority13_rdata[31:3] = 29'h0;

wire csr_p_priority13_wen;
assign csr_p_priority13_wen = wen && (waddr == 32'h34);

wire csr_p_priority13_ren;
assign csr_p_priority13_ren = ren && (raddr == 32'h34);
reg csr_p_priority13_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority13_ren_ff <= 1'b0;
    end else begin
        csr_p_priority13_ren_ff <= csr_p_priority13_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY13[2:0] - PRIORITY13 - Priority level of IRQ13. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority13_priority13_ff;

assign csr_p_priority13_rdata[2:0] = csr_p_priority13_priority13_ff;

assign csr_p_priority13_priority13_out = csr_p_priority13_priority13_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority13_priority13_ff <= 3'h0;
    end else  begin
     if (csr_p_priority13_wen) begin
            if (wstrb[0]) begin
                csr_p_priority13_priority13_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority13_priority13_ff <= csr_p_priority13_priority13_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x38] - P_PRIORITY14 - Priority Register IRQ14 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority14_rdata;
assign csr_p_priority14_rdata[31:3] = 29'h0;

wire csr_p_priority14_wen;
assign csr_p_priority14_wen = wen && (waddr == 32'h38);

wire csr_p_priority14_ren;
assign csr_p_priority14_ren = ren && (raddr == 32'h38);
reg csr_p_priority14_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority14_ren_ff <= 1'b0;
    end else begin
        csr_p_priority14_ren_ff <= csr_p_priority14_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY14[2:0] - PRIORITY14 - Priority level of IRQ14. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority14_priority14_ff;

assign csr_p_priority14_rdata[2:0] = csr_p_priority14_priority14_ff;

assign csr_p_priority14_priority14_out = csr_p_priority14_priority14_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority14_priority14_ff <= 3'h0;
    end else  begin
     if (csr_p_priority14_wen) begin
            if (wstrb[0]) begin
                csr_p_priority14_priority14_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority14_priority14_ff <= csr_p_priority14_priority14_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x3c] - P_PRIORITY15 - Priority Register IRQ15 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority15_rdata;
assign csr_p_priority15_rdata[31:3] = 29'h0;

wire csr_p_priority15_wen;
assign csr_p_priority15_wen = wen && (waddr == 32'h3c);

wire csr_p_priority15_ren;
assign csr_p_priority15_ren = ren && (raddr == 32'h3c);
reg csr_p_priority15_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority15_ren_ff <= 1'b0;
    end else begin
        csr_p_priority15_ren_ff <= csr_p_priority15_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY15[2:0] - PRIORITY15 - Priority level of IRQ15. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority15_priority15_ff;

assign csr_p_priority15_rdata[2:0] = csr_p_priority15_priority15_ff;

assign csr_p_priority15_priority15_out = csr_p_priority15_priority15_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority15_priority15_ff <= 3'h0;
    end else  begin
     if (csr_p_priority15_wen) begin
            if (wstrb[0]) begin
                csr_p_priority15_priority15_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority15_priority15_ff <= csr_p_priority15_priority15_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x40] - P_PRIORITY16 - Priority Register IRQ16 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority16_rdata;
assign csr_p_priority16_rdata[31:3] = 29'h0;

wire csr_p_priority16_wen;
assign csr_p_priority16_wen = wen && (waddr == 32'h40);

wire csr_p_priority16_ren;
assign csr_p_priority16_ren = ren && (raddr == 32'h40);
reg csr_p_priority16_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority16_ren_ff <= 1'b0;
    end else begin
        csr_p_priority16_ren_ff <= csr_p_priority16_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY16[2:0] - PRIORITY16 - Priority level of IRQ16. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority16_priority16_ff;

assign csr_p_priority16_rdata[2:0] = csr_p_priority16_priority16_ff;

assign csr_p_priority16_priority16_out = csr_p_priority16_priority16_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority16_priority16_ff <= 3'h0;
    end else  begin
     if (csr_p_priority16_wen) begin
            if (wstrb[0]) begin
                csr_p_priority16_priority16_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority16_priority16_ff <= csr_p_priority16_priority16_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x44] - P_PRIORITY17 - Priority Register IRQ17 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority17_rdata;
assign csr_p_priority17_rdata[31:3] = 29'h0;

wire csr_p_priority17_wen;
assign csr_p_priority17_wen = wen && (waddr == 32'h44);

wire csr_p_priority17_ren;
assign csr_p_priority17_ren = ren && (raddr == 32'h44);
reg csr_p_priority17_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority17_ren_ff <= 1'b0;
    end else begin
        csr_p_priority17_ren_ff <= csr_p_priority17_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY17[2:0] - PRIORITY17 - Priority level of IRQ17. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority17_priority17_ff;

assign csr_p_priority17_rdata[2:0] = csr_p_priority17_priority17_ff;

assign csr_p_priority17_priority17_out = csr_p_priority17_priority17_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority17_priority17_ff <= 3'h0;
    end else  begin
     if (csr_p_priority17_wen) begin
            if (wstrb[0]) begin
                csr_p_priority17_priority17_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority17_priority17_ff <= csr_p_priority17_priority17_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x48] - P_PRIORITY18 - Priority Register IRQ18 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority18_rdata;
assign csr_p_priority18_rdata[31:3] = 29'h0;

wire csr_p_priority18_wen;
assign csr_p_priority18_wen = wen && (waddr == 32'h48);

wire csr_p_priority18_ren;
assign csr_p_priority18_ren = ren && (raddr == 32'h48);
reg csr_p_priority18_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority18_ren_ff <= 1'b0;
    end else begin
        csr_p_priority18_ren_ff <= csr_p_priority18_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY18[2:0] - PRIORITY18 - Priority level of IRQ18. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority18_priority18_ff;

assign csr_p_priority18_rdata[2:0] = csr_p_priority18_priority18_ff;

assign csr_p_priority18_priority18_out = csr_p_priority18_priority18_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority18_priority18_ff <= 3'h0;
    end else  begin
     if (csr_p_priority18_wen) begin
            if (wstrb[0]) begin
                csr_p_priority18_priority18_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority18_priority18_ff <= csr_p_priority18_priority18_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4c] - P_PRIORITY19 - Priority Register IRQ19 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority19_rdata;
assign csr_p_priority19_rdata[31:3] = 29'h0;

wire csr_p_priority19_wen;
assign csr_p_priority19_wen = wen && (waddr == 32'h4c);

wire csr_p_priority19_ren;
assign csr_p_priority19_ren = ren && (raddr == 32'h4c);
reg csr_p_priority19_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority19_ren_ff <= 1'b0;
    end else begin
        csr_p_priority19_ren_ff <= csr_p_priority19_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY19[2:0] - PRIORITY19 - Priority level of IRQ19. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority19_priority19_ff;

assign csr_p_priority19_rdata[2:0] = csr_p_priority19_priority19_ff;

assign csr_p_priority19_priority19_out = csr_p_priority19_priority19_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority19_priority19_ff <= 3'h0;
    end else  begin
     if (csr_p_priority19_wen) begin
            if (wstrb[0]) begin
                csr_p_priority19_priority19_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority19_priority19_ff <= csr_p_priority19_priority19_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x50] - P_PRIORITY20 - Priority Register IRQ20 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority20_rdata;
assign csr_p_priority20_rdata[31:3] = 29'h0;

wire csr_p_priority20_wen;
assign csr_p_priority20_wen = wen && (waddr == 32'h50);

wire csr_p_priority20_ren;
assign csr_p_priority20_ren = ren && (raddr == 32'h50);
reg csr_p_priority20_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority20_ren_ff <= 1'b0;
    end else begin
        csr_p_priority20_ren_ff <= csr_p_priority20_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY20[2:0] - PRIORITY20 - Priority level of IRQ20. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority20_priority20_ff;

assign csr_p_priority20_rdata[2:0] = csr_p_priority20_priority20_ff;

assign csr_p_priority20_priority20_out = csr_p_priority20_priority20_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority20_priority20_ff <= 3'h0;
    end else  begin
     if (csr_p_priority20_wen) begin
            if (wstrb[0]) begin
                csr_p_priority20_priority20_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority20_priority20_ff <= csr_p_priority20_priority20_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x54] - P_PRIORITY21 - Priority Register IRQ21 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority21_rdata;
assign csr_p_priority21_rdata[31:3] = 29'h0;

wire csr_p_priority21_wen;
assign csr_p_priority21_wen = wen && (waddr == 32'h54);

wire csr_p_priority21_ren;
assign csr_p_priority21_ren = ren && (raddr == 32'h54);
reg csr_p_priority21_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority21_ren_ff <= 1'b0;
    end else begin
        csr_p_priority21_ren_ff <= csr_p_priority21_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY21[2:0] - PRIORITY21 - Priority level of IRQ21. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority21_priority21_ff;

assign csr_p_priority21_rdata[2:0] = csr_p_priority21_priority21_ff;

assign csr_p_priority21_priority21_out = csr_p_priority21_priority21_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority21_priority21_ff <= 3'h0;
    end else  begin
     if (csr_p_priority21_wen) begin
            if (wstrb[0]) begin
                csr_p_priority21_priority21_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority21_priority21_ff <= csr_p_priority21_priority21_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x58] - P_PRIORITY22 - Priority Register IRQ22 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority22_rdata;
assign csr_p_priority22_rdata[31:3] = 29'h0;

wire csr_p_priority22_wen;
assign csr_p_priority22_wen = wen && (waddr == 32'h58);

wire csr_p_priority22_ren;
assign csr_p_priority22_ren = ren && (raddr == 32'h58);
reg csr_p_priority22_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority22_ren_ff <= 1'b0;
    end else begin
        csr_p_priority22_ren_ff <= csr_p_priority22_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY22[2:0] - PRIORITY22 - Priority level of IRQ22. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority22_priority22_ff;

assign csr_p_priority22_rdata[2:0] = csr_p_priority22_priority22_ff;

assign csr_p_priority22_priority22_out = csr_p_priority22_priority22_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority22_priority22_ff <= 3'h0;
    end else  begin
     if (csr_p_priority22_wen) begin
            if (wstrb[0]) begin
                csr_p_priority22_priority22_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority22_priority22_ff <= csr_p_priority22_priority22_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x5c] - P_PRIORITY23 - Priority Register IRQ23 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority23_rdata;
assign csr_p_priority23_rdata[31:3] = 29'h0;

wire csr_p_priority23_wen;
assign csr_p_priority23_wen = wen && (waddr == 32'h5c);

wire csr_p_priority23_ren;
assign csr_p_priority23_ren = ren && (raddr == 32'h5c);
reg csr_p_priority23_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority23_ren_ff <= 1'b0;
    end else begin
        csr_p_priority23_ren_ff <= csr_p_priority23_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY23[2:0] - PRIORITY23 - Priority level of IRQ23. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority23_priority23_ff;

assign csr_p_priority23_rdata[2:0] = csr_p_priority23_priority23_ff;

assign csr_p_priority23_priority23_out = csr_p_priority23_priority23_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority23_priority23_ff <= 3'h0;
    end else  begin
     if (csr_p_priority23_wen) begin
            if (wstrb[0]) begin
                csr_p_priority23_priority23_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority23_priority23_ff <= csr_p_priority23_priority23_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x60] - P_PRIORITY24 - Priority Register IRQ24 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority24_rdata;
assign csr_p_priority24_rdata[31:3] = 29'h0;

wire csr_p_priority24_wen;
assign csr_p_priority24_wen = wen && (waddr == 32'h60);

wire csr_p_priority24_ren;
assign csr_p_priority24_ren = ren && (raddr == 32'h60);
reg csr_p_priority24_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority24_ren_ff <= 1'b0;
    end else begin
        csr_p_priority24_ren_ff <= csr_p_priority24_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY24[2:0] - PRIORITY24 - Priority level of IRQ24. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority24_priority24_ff;

assign csr_p_priority24_rdata[2:0] = csr_p_priority24_priority24_ff;

assign csr_p_priority24_priority24_out = csr_p_priority24_priority24_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority24_priority24_ff <= 3'h0;
    end else  begin
     if (csr_p_priority24_wen) begin
            if (wstrb[0]) begin
                csr_p_priority24_priority24_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority24_priority24_ff <= csr_p_priority24_priority24_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x64] - P_PRIORITY25 - Priority Register IRQ25 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority25_rdata;
assign csr_p_priority25_rdata[31:3] = 29'h0;

wire csr_p_priority25_wen;
assign csr_p_priority25_wen = wen && (waddr == 32'h64);

wire csr_p_priority25_ren;
assign csr_p_priority25_ren = ren && (raddr == 32'h64);
reg csr_p_priority25_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority25_ren_ff <= 1'b0;
    end else begin
        csr_p_priority25_ren_ff <= csr_p_priority25_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY25[2:0] - PRIORITY25 - Priority level of IRQ25. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority25_priority25_ff;

assign csr_p_priority25_rdata[2:0] = csr_p_priority25_priority25_ff;

assign csr_p_priority25_priority25_out = csr_p_priority25_priority25_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority25_priority25_ff <= 3'h0;
    end else  begin
     if (csr_p_priority25_wen) begin
            if (wstrb[0]) begin
                csr_p_priority25_priority25_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority25_priority25_ff <= csr_p_priority25_priority25_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x68] - P_PRIORITY26 - Priority Register IRQ26 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority26_rdata;
assign csr_p_priority26_rdata[31:3] = 29'h0;

wire csr_p_priority26_wen;
assign csr_p_priority26_wen = wen && (waddr == 32'h68);

wire csr_p_priority26_ren;
assign csr_p_priority26_ren = ren && (raddr == 32'h68);
reg csr_p_priority26_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority26_ren_ff <= 1'b0;
    end else begin
        csr_p_priority26_ren_ff <= csr_p_priority26_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY26[2:0] - PRIORITY26 - Priority level of IRQ26. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority26_priority26_ff;

assign csr_p_priority26_rdata[2:0] = csr_p_priority26_priority26_ff;

assign csr_p_priority26_priority26_out = csr_p_priority26_priority26_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority26_priority26_ff <= 3'h0;
    end else  begin
     if (csr_p_priority26_wen) begin
            if (wstrb[0]) begin
                csr_p_priority26_priority26_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority26_priority26_ff <= csr_p_priority26_priority26_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x6c] - P_PRIORITY27 - Priority Register IRQ27 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority27_rdata;
assign csr_p_priority27_rdata[31:3] = 29'h0;

wire csr_p_priority27_wen;
assign csr_p_priority27_wen = wen && (waddr == 32'h6c);

wire csr_p_priority27_ren;
assign csr_p_priority27_ren = ren && (raddr == 32'h6c);
reg csr_p_priority27_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority27_ren_ff <= 1'b0;
    end else begin
        csr_p_priority27_ren_ff <= csr_p_priority27_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY27[2:0] - PRIORITY27 - Priority level of IRQ27. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority27_priority27_ff;

assign csr_p_priority27_rdata[2:0] = csr_p_priority27_priority27_ff;

assign csr_p_priority27_priority27_out = csr_p_priority27_priority27_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority27_priority27_ff <= 3'h0;
    end else  begin
     if (csr_p_priority27_wen) begin
            if (wstrb[0]) begin
                csr_p_priority27_priority27_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority27_priority27_ff <= csr_p_priority27_priority27_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x70] - P_PRIORITY28 - Priority Register IRQ28 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority28_rdata;
assign csr_p_priority28_rdata[31:3] = 29'h0;

wire csr_p_priority28_wen;
assign csr_p_priority28_wen = wen && (waddr == 32'h70);

wire csr_p_priority28_ren;
assign csr_p_priority28_ren = ren && (raddr == 32'h70);
reg csr_p_priority28_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority28_ren_ff <= 1'b0;
    end else begin
        csr_p_priority28_ren_ff <= csr_p_priority28_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY28[2:0] - PRIORITY28 - Priority level of IRQ28. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority28_priority28_ff;

assign csr_p_priority28_rdata[2:0] = csr_p_priority28_priority28_ff;

assign csr_p_priority28_priority28_out = csr_p_priority28_priority28_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority28_priority28_ff <= 3'h0;
    end else  begin
     if (csr_p_priority28_wen) begin
            if (wstrb[0]) begin
                csr_p_priority28_priority28_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority28_priority28_ff <= csr_p_priority28_priority28_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x74] - P_PRIORITY29 - Priority Register IRQ29 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority29_rdata;
assign csr_p_priority29_rdata[31:3] = 29'h0;

wire csr_p_priority29_wen;
assign csr_p_priority29_wen = wen && (waddr == 32'h74);

wire csr_p_priority29_ren;
assign csr_p_priority29_ren = ren && (raddr == 32'h74);
reg csr_p_priority29_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority29_ren_ff <= 1'b0;
    end else begin
        csr_p_priority29_ren_ff <= csr_p_priority29_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY29[2:0] - PRIORITY29 - Priority level of IRQ29. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority29_priority29_ff;

assign csr_p_priority29_rdata[2:0] = csr_p_priority29_priority29_ff;

assign csr_p_priority29_priority29_out = csr_p_priority29_priority29_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority29_priority29_ff <= 3'h0;
    end else  begin
     if (csr_p_priority29_wen) begin
            if (wstrb[0]) begin
                csr_p_priority29_priority29_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority29_priority29_ff <= csr_p_priority29_priority29_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x78] - P_PRIORITY30 - Priority Register IRQ30 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority30_rdata;
assign csr_p_priority30_rdata[31:3] = 29'h0;

wire csr_p_priority30_wen;
assign csr_p_priority30_wen = wen && (waddr == 32'h78);

wire csr_p_priority30_ren;
assign csr_p_priority30_ren = ren && (raddr == 32'h78);
reg csr_p_priority30_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority30_ren_ff <= 1'b0;
    end else begin
        csr_p_priority30_ren_ff <= csr_p_priority30_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY30[2:0] - PRIORITY30 - Priority level of IRQ30. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority30_priority30_ff;

assign csr_p_priority30_rdata[2:0] = csr_p_priority30_priority30_ff;

assign csr_p_priority30_priority30_out = csr_p_priority30_priority30_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority30_priority30_ff <= 3'h0;
    end else  begin
     if (csr_p_priority30_wen) begin
            if (wstrb[0]) begin
                csr_p_priority30_priority30_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority30_priority30_ff <= csr_p_priority30_priority30_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x7c] - P_PRIORITY31 - Priority Register IRQ31 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority31_rdata;
assign csr_p_priority31_rdata[31:3] = 29'h0;

wire csr_p_priority31_wen;
assign csr_p_priority31_wen = wen && (waddr == 32'h7c);

wire csr_p_priority31_ren;
assign csr_p_priority31_ren = ren && (raddr == 32'h7c);
reg csr_p_priority31_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority31_ren_ff <= 1'b0;
    end else begin
        csr_p_priority31_ren_ff <= csr_p_priority31_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY31[2:0] - PRIORITY31 - Priority level of IRQ31. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority31_priority31_ff;

assign csr_p_priority31_rdata[2:0] = csr_p_priority31_priority31_ff;

assign csr_p_priority31_priority31_out = csr_p_priority31_priority31_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority31_priority31_ff <= 3'h0;
    end else  begin
     if (csr_p_priority31_wen) begin
            if (wstrb[0]) begin
                csr_p_priority31_priority31_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority31_priority31_ff <= csr_p_priority31_priority31_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x80] - P_PRIORITY32 - Priority Register IRQ32 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority32_rdata;
assign csr_p_priority32_rdata[31:3] = 29'h0;

wire csr_p_priority32_wen;
assign csr_p_priority32_wen = wen && (waddr == 32'h80);

wire csr_p_priority32_ren;
assign csr_p_priority32_ren = ren && (raddr == 32'h80);
reg csr_p_priority32_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority32_ren_ff <= 1'b0;
    end else begin
        csr_p_priority32_ren_ff <= csr_p_priority32_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY32[2:0] - PRIORITY32 - Priority level of IRQ32. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority32_priority32_ff;

assign csr_p_priority32_rdata[2:0] = csr_p_priority32_priority32_ff;

assign csr_p_priority32_priority32_out = csr_p_priority32_priority32_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority32_priority32_ff <= 3'h0;
    end else  begin
     if (csr_p_priority32_wen) begin
            if (wstrb[0]) begin
                csr_p_priority32_priority32_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority32_priority32_ff <= csr_p_priority32_priority32_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x84] - P_PRIORITY33 - Priority Register IRQ33 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority33_rdata;
assign csr_p_priority33_rdata[31:3] = 29'h0;

wire csr_p_priority33_wen;
assign csr_p_priority33_wen = wen && (waddr == 32'h84);

wire csr_p_priority33_ren;
assign csr_p_priority33_ren = ren && (raddr == 32'h84);
reg csr_p_priority33_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority33_ren_ff <= 1'b0;
    end else begin
        csr_p_priority33_ren_ff <= csr_p_priority33_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY33[2:0] - PRIORITY33 - Priority level of IRQ33. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority33_priority33_ff;

assign csr_p_priority33_rdata[2:0] = csr_p_priority33_priority33_ff;

assign csr_p_priority33_priority33_out = csr_p_priority33_priority33_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority33_priority33_ff <= 3'h0;
    end else  begin
     if (csr_p_priority33_wen) begin
            if (wstrb[0]) begin
                csr_p_priority33_priority33_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority33_priority33_ff <= csr_p_priority33_priority33_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x88] - P_PRIORITY34 - Priority Register IRQ34 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority34_rdata;
assign csr_p_priority34_rdata[31:3] = 29'h0;

wire csr_p_priority34_wen;
assign csr_p_priority34_wen = wen && (waddr == 32'h88);

wire csr_p_priority34_ren;
assign csr_p_priority34_ren = ren && (raddr == 32'h88);
reg csr_p_priority34_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority34_ren_ff <= 1'b0;
    end else begin
        csr_p_priority34_ren_ff <= csr_p_priority34_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY34[2:0] - PRIORITY34 - Priority level of IRQ34. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority34_priority34_ff;

assign csr_p_priority34_rdata[2:0] = csr_p_priority34_priority34_ff;

assign csr_p_priority34_priority34_out = csr_p_priority34_priority34_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority34_priority34_ff <= 3'h0;
    end else  begin
     if (csr_p_priority34_wen) begin
            if (wstrb[0]) begin
                csr_p_priority34_priority34_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority34_priority34_ff <= csr_p_priority34_priority34_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8c] - P_PRIORITY35 - Priority Register IRQ35 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority35_rdata;
assign csr_p_priority35_rdata[31:3] = 29'h0;

wire csr_p_priority35_wen;
assign csr_p_priority35_wen = wen && (waddr == 32'h8c);

wire csr_p_priority35_ren;
assign csr_p_priority35_ren = ren && (raddr == 32'h8c);
reg csr_p_priority35_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority35_ren_ff <= 1'b0;
    end else begin
        csr_p_priority35_ren_ff <= csr_p_priority35_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY35[2:0] - PRIORITY35 - Priority level of IRQ35. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority35_priority35_ff;

assign csr_p_priority35_rdata[2:0] = csr_p_priority35_priority35_ff;

assign csr_p_priority35_priority35_out = csr_p_priority35_priority35_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority35_priority35_ff <= 3'h0;
    end else  begin
     if (csr_p_priority35_wen) begin
            if (wstrb[0]) begin
                csr_p_priority35_priority35_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority35_priority35_ff <= csr_p_priority35_priority35_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x90] - P_PRIORITY36 - Priority Register IRQ36 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority36_rdata;
assign csr_p_priority36_rdata[31:3] = 29'h0;

wire csr_p_priority36_wen;
assign csr_p_priority36_wen = wen && (waddr == 32'h90);

wire csr_p_priority36_ren;
assign csr_p_priority36_ren = ren && (raddr == 32'h90);
reg csr_p_priority36_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority36_ren_ff <= 1'b0;
    end else begin
        csr_p_priority36_ren_ff <= csr_p_priority36_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY36[2:0] - PRIORITY36 - Priority level of IRQ36. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority36_priority36_ff;

assign csr_p_priority36_rdata[2:0] = csr_p_priority36_priority36_ff;

assign csr_p_priority36_priority36_out = csr_p_priority36_priority36_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority36_priority36_ff <= 3'h0;
    end else  begin
     if (csr_p_priority36_wen) begin
            if (wstrb[0]) begin
                csr_p_priority36_priority36_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority36_priority36_ff <= csr_p_priority36_priority36_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x94] - P_PRIORITY37 - Priority Register IRQ37 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority37_rdata;
assign csr_p_priority37_rdata[31:3] = 29'h0;

wire csr_p_priority37_wen;
assign csr_p_priority37_wen = wen && (waddr == 32'h94);

wire csr_p_priority37_ren;
assign csr_p_priority37_ren = ren && (raddr == 32'h94);
reg csr_p_priority37_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority37_ren_ff <= 1'b0;
    end else begin
        csr_p_priority37_ren_ff <= csr_p_priority37_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY37[2:0] - PRIORITY37 - Priority level of IRQ37. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority37_priority37_ff;

assign csr_p_priority37_rdata[2:0] = csr_p_priority37_priority37_ff;

assign csr_p_priority37_priority37_out = csr_p_priority37_priority37_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority37_priority37_ff <= 3'h0;
    end else  begin
     if (csr_p_priority37_wen) begin
            if (wstrb[0]) begin
                csr_p_priority37_priority37_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority37_priority37_ff <= csr_p_priority37_priority37_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x98] - P_PRIORITY38 - Priority Register IRQ38 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority38_rdata;
assign csr_p_priority38_rdata[31:3] = 29'h0;

wire csr_p_priority38_wen;
assign csr_p_priority38_wen = wen && (waddr == 32'h98);

wire csr_p_priority38_ren;
assign csr_p_priority38_ren = ren && (raddr == 32'h98);
reg csr_p_priority38_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority38_ren_ff <= 1'b0;
    end else begin
        csr_p_priority38_ren_ff <= csr_p_priority38_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY38[2:0] - PRIORITY38 - Priority level of IRQ38. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority38_priority38_ff;

assign csr_p_priority38_rdata[2:0] = csr_p_priority38_priority38_ff;

assign csr_p_priority38_priority38_out = csr_p_priority38_priority38_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority38_priority38_ff <= 3'h0;
    end else  begin
     if (csr_p_priority38_wen) begin
            if (wstrb[0]) begin
                csr_p_priority38_priority38_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority38_priority38_ff <= csr_p_priority38_priority38_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x9c] - P_PRIORITY39 - Priority Register IRQ39 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority39_rdata;
assign csr_p_priority39_rdata[31:3] = 29'h0;

wire csr_p_priority39_wen;
assign csr_p_priority39_wen = wen && (waddr == 32'h9c);

wire csr_p_priority39_ren;
assign csr_p_priority39_ren = ren && (raddr == 32'h9c);
reg csr_p_priority39_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority39_ren_ff <= 1'b0;
    end else begin
        csr_p_priority39_ren_ff <= csr_p_priority39_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY39[2:0] - PRIORITY39 - Priority level of IRQ39. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority39_priority39_ff;

assign csr_p_priority39_rdata[2:0] = csr_p_priority39_priority39_ff;

assign csr_p_priority39_priority39_out = csr_p_priority39_priority39_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority39_priority39_ff <= 3'h0;
    end else  begin
     if (csr_p_priority39_wen) begin
            if (wstrb[0]) begin
                csr_p_priority39_priority39_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority39_priority39_ff <= csr_p_priority39_priority39_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xa0] - P_PRIORITY40 - Priority Register IRQ40 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority40_rdata;
assign csr_p_priority40_rdata[31:3] = 29'h0;

wire csr_p_priority40_wen;
assign csr_p_priority40_wen = wen && (waddr == 32'ha0);

wire csr_p_priority40_ren;
assign csr_p_priority40_ren = ren && (raddr == 32'ha0);
reg csr_p_priority40_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority40_ren_ff <= 1'b0;
    end else begin
        csr_p_priority40_ren_ff <= csr_p_priority40_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY40[2:0] - PRIORITY40 - Priority level of IRQ40. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority40_priority40_ff;

assign csr_p_priority40_rdata[2:0] = csr_p_priority40_priority40_ff;

assign csr_p_priority40_priority40_out = csr_p_priority40_priority40_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority40_priority40_ff <= 3'h0;
    end else  begin
     if (csr_p_priority40_wen) begin
            if (wstrb[0]) begin
                csr_p_priority40_priority40_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority40_priority40_ff <= csr_p_priority40_priority40_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xa4] - P_PRIORITY41 - Priority Register IRQ41 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority41_rdata;
assign csr_p_priority41_rdata[31:3] = 29'h0;

wire csr_p_priority41_wen;
assign csr_p_priority41_wen = wen && (waddr == 32'ha4);

wire csr_p_priority41_ren;
assign csr_p_priority41_ren = ren && (raddr == 32'ha4);
reg csr_p_priority41_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority41_ren_ff <= 1'b0;
    end else begin
        csr_p_priority41_ren_ff <= csr_p_priority41_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY41[2:0] - PRIORITY41 - Priority level of IRQ41. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority41_priority41_ff;

assign csr_p_priority41_rdata[2:0] = csr_p_priority41_priority41_ff;

assign csr_p_priority41_priority41_out = csr_p_priority41_priority41_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority41_priority41_ff <= 3'h0;
    end else  begin
     if (csr_p_priority41_wen) begin
            if (wstrb[0]) begin
                csr_p_priority41_priority41_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority41_priority41_ff <= csr_p_priority41_priority41_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xa8] - P_PRIORITY42 - Priority Register IRQ42 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority42_rdata;
assign csr_p_priority42_rdata[31:3] = 29'h0;

wire csr_p_priority42_wen;
assign csr_p_priority42_wen = wen && (waddr == 32'ha8);

wire csr_p_priority42_ren;
assign csr_p_priority42_ren = ren && (raddr == 32'ha8);
reg csr_p_priority42_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority42_ren_ff <= 1'b0;
    end else begin
        csr_p_priority42_ren_ff <= csr_p_priority42_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY42[2:0] - PRIORITY42 - Priority level of IRQ42. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority42_priority42_ff;

assign csr_p_priority42_rdata[2:0] = csr_p_priority42_priority42_ff;

assign csr_p_priority42_priority42_out = csr_p_priority42_priority42_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority42_priority42_ff <= 3'h0;
    end else  begin
     if (csr_p_priority42_wen) begin
            if (wstrb[0]) begin
                csr_p_priority42_priority42_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority42_priority42_ff <= csr_p_priority42_priority42_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xac] - P_PRIORITY43 - Priority Register IRQ43 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority43_rdata;
assign csr_p_priority43_rdata[31:3] = 29'h0;

wire csr_p_priority43_wen;
assign csr_p_priority43_wen = wen && (waddr == 32'hac);

wire csr_p_priority43_ren;
assign csr_p_priority43_ren = ren && (raddr == 32'hac);
reg csr_p_priority43_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority43_ren_ff <= 1'b0;
    end else begin
        csr_p_priority43_ren_ff <= csr_p_priority43_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY43[2:0] - PRIORITY43 - Priority level of IRQ43. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority43_priority43_ff;

assign csr_p_priority43_rdata[2:0] = csr_p_priority43_priority43_ff;

assign csr_p_priority43_priority43_out = csr_p_priority43_priority43_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority43_priority43_ff <= 3'h0;
    end else  begin
     if (csr_p_priority43_wen) begin
            if (wstrb[0]) begin
                csr_p_priority43_priority43_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority43_priority43_ff <= csr_p_priority43_priority43_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xb0] - P_PRIORITY44 - Priority Register IRQ44 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority44_rdata;
assign csr_p_priority44_rdata[31:3] = 29'h0;

wire csr_p_priority44_wen;
assign csr_p_priority44_wen = wen && (waddr == 32'hb0);

wire csr_p_priority44_ren;
assign csr_p_priority44_ren = ren && (raddr == 32'hb0);
reg csr_p_priority44_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority44_ren_ff <= 1'b0;
    end else begin
        csr_p_priority44_ren_ff <= csr_p_priority44_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY44[2:0] - PRIORITY44 - Priority level of IRQ44. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority44_priority44_ff;

assign csr_p_priority44_rdata[2:0] = csr_p_priority44_priority44_ff;

assign csr_p_priority44_priority44_out = csr_p_priority44_priority44_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority44_priority44_ff <= 3'h0;
    end else  begin
     if (csr_p_priority44_wen) begin
            if (wstrb[0]) begin
                csr_p_priority44_priority44_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority44_priority44_ff <= csr_p_priority44_priority44_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xb4] - P_PRIORITY45 - Priority Register IRQ45 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority45_rdata;
assign csr_p_priority45_rdata[31:3] = 29'h0;

wire csr_p_priority45_wen;
assign csr_p_priority45_wen = wen && (waddr == 32'hb4);

wire csr_p_priority45_ren;
assign csr_p_priority45_ren = ren && (raddr == 32'hb4);
reg csr_p_priority45_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority45_ren_ff <= 1'b0;
    end else begin
        csr_p_priority45_ren_ff <= csr_p_priority45_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY45[2:0] - PRIORITY45 - Priority level of IRQ45. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority45_priority45_ff;

assign csr_p_priority45_rdata[2:0] = csr_p_priority45_priority45_ff;

assign csr_p_priority45_priority45_out = csr_p_priority45_priority45_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority45_priority45_ff <= 3'h0;
    end else  begin
     if (csr_p_priority45_wen) begin
            if (wstrb[0]) begin
                csr_p_priority45_priority45_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority45_priority45_ff <= csr_p_priority45_priority45_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xb8] - P_PRIORITY46 - Priority Register IRQ46 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority46_rdata;
assign csr_p_priority46_rdata[31:3] = 29'h0;

wire csr_p_priority46_wen;
assign csr_p_priority46_wen = wen && (waddr == 32'hb8);

wire csr_p_priority46_ren;
assign csr_p_priority46_ren = ren && (raddr == 32'hb8);
reg csr_p_priority46_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority46_ren_ff <= 1'b0;
    end else begin
        csr_p_priority46_ren_ff <= csr_p_priority46_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY46[2:0] - PRIORITY46 - Priority level of IRQ46. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority46_priority46_ff;

assign csr_p_priority46_rdata[2:0] = csr_p_priority46_priority46_ff;

assign csr_p_priority46_priority46_out = csr_p_priority46_priority46_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority46_priority46_ff <= 3'h0;
    end else  begin
     if (csr_p_priority46_wen) begin
            if (wstrb[0]) begin
                csr_p_priority46_priority46_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority46_priority46_ff <= csr_p_priority46_priority46_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xbc] - P_PRIORITY47 - Priority Register IRQ47 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority47_rdata;
assign csr_p_priority47_rdata[31:3] = 29'h0;

wire csr_p_priority47_wen;
assign csr_p_priority47_wen = wen && (waddr == 32'hbc);

wire csr_p_priority47_ren;
assign csr_p_priority47_ren = ren && (raddr == 32'hbc);
reg csr_p_priority47_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority47_ren_ff <= 1'b0;
    end else begin
        csr_p_priority47_ren_ff <= csr_p_priority47_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY47[2:0] - PRIORITY47 - Priority level of IRQ47. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority47_priority47_ff;

assign csr_p_priority47_rdata[2:0] = csr_p_priority47_priority47_ff;

assign csr_p_priority47_priority47_out = csr_p_priority47_priority47_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority47_priority47_ff <= 3'h0;
    end else  begin
     if (csr_p_priority47_wen) begin
            if (wstrb[0]) begin
                csr_p_priority47_priority47_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority47_priority47_ff <= csr_p_priority47_priority47_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc0] - P_PRIORITY48 - Priority Register IRQ48 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority48_rdata;
assign csr_p_priority48_rdata[31:3] = 29'h0;

wire csr_p_priority48_wen;
assign csr_p_priority48_wen = wen && (waddr == 32'hc0);

wire csr_p_priority48_ren;
assign csr_p_priority48_ren = ren && (raddr == 32'hc0);
reg csr_p_priority48_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority48_ren_ff <= 1'b0;
    end else begin
        csr_p_priority48_ren_ff <= csr_p_priority48_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY48[2:0] - PRIORITY48 - Priority level of IRQ48. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority48_priority48_ff;

assign csr_p_priority48_rdata[2:0] = csr_p_priority48_priority48_ff;

assign csr_p_priority48_priority48_out = csr_p_priority48_priority48_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority48_priority48_ff <= 3'h0;
    end else  begin
     if (csr_p_priority48_wen) begin
            if (wstrb[0]) begin
                csr_p_priority48_priority48_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority48_priority48_ff <= csr_p_priority48_priority48_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc4] - P_PRIORITY49 - Priority Register IRQ49 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority49_rdata;
assign csr_p_priority49_rdata[31:3] = 29'h0;

wire csr_p_priority49_wen;
assign csr_p_priority49_wen = wen && (waddr == 32'hc4);

wire csr_p_priority49_ren;
assign csr_p_priority49_ren = ren && (raddr == 32'hc4);
reg csr_p_priority49_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority49_ren_ff <= 1'b0;
    end else begin
        csr_p_priority49_ren_ff <= csr_p_priority49_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY49[2:0] - PRIORITY49 - Priority level of IRQ49. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority49_priority49_ff;

assign csr_p_priority49_rdata[2:0] = csr_p_priority49_priority49_ff;

assign csr_p_priority49_priority49_out = csr_p_priority49_priority49_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority49_priority49_ff <= 3'h0;
    end else  begin
     if (csr_p_priority49_wen) begin
            if (wstrb[0]) begin
                csr_p_priority49_priority49_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority49_priority49_ff <= csr_p_priority49_priority49_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xc8] - P_PRIORITY50 - Priority Register IRQ50 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority50_rdata;
assign csr_p_priority50_rdata[31:3] = 29'h0;

wire csr_p_priority50_wen;
assign csr_p_priority50_wen = wen && (waddr == 32'hc8);

wire csr_p_priority50_ren;
assign csr_p_priority50_ren = ren && (raddr == 32'hc8);
reg csr_p_priority50_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority50_ren_ff <= 1'b0;
    end else begin
        csr_p_priority50_ren_ff <= csr_p_priority50_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY50[2:0] - PRIORITY50 - Priority level of IRQ50. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority50_priority50_ff;

assign csr_p_priority50_rdata[2:0] = csr_p_priority50_priority50_ff;

assign csr_p_priority50_priority50_out = csr_p_priority50_priority50_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority50_priority50_ff <= 3'h0;
    end else  begin
     if (csr_p_priority50_wen) begin
            if (wstrb[0]) begin
                csr_p_priority50_priority50_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority50_priority50_ff <= csr_p_priority50_priority50_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xcc] - P_PRIORITY51 - Priority Register IRQ51 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority51_rdata;
assign csr_p_priority51_rdata[31:3] = 29'h0;

wire csr_p_priority51_wen;
assign csr_p_priority51_wen = wen && (waddr == 32'hcc);

wire csr_p_priority51_ren;
assign csr_p_priority51_ren = ren && (raddr == 32'hcc);
reg csr_p_priority51_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority51_ren_ff <= 1'b0;
    end else begin
        csr_p_priority51_ren_ff <= csr_p_priority51_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY51[2:0] - PRIORITY51 - Priority level of IRQ51. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority51_priority51_ff;

assign csr_p_priority51_rdata[2:0] = csr_p_priority51_priority51_ff;

assign csr_p_priority51_priority51_out = csr_p_priority51_priority51_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority51_priority51_ff <= 3'h0;
    end else  begin
     if (csr_p_priority51_wen) begin
            if (wstrb[0]) begin
                csr_p_priority51_priority51_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority51_priority51_ff <= csr_p_priority51_priority51_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xd0] - P_PRIORITY52 - Priority Register IRQ52 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority52_rdata;
assign csr_p_priority52_rdata[31:3] = 29'h0;

wire csr_p_priority52_wen;
assign csr_p_priority52_wen = wen && (waddr == 32'hd0);

wire csr_p_priority52_ren;
assign csr_p_priority52_ren = ren && (raddr == 32'hd0);
reg csr_p_priority52_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority52_ren_ff <= 1'b0;
    end else begin
        csr_p_priority52_ren_ff <= csr_p_priority52_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY52[2:0] - PRIORITY52 - Priority level of IRQ52. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority52_priority52_ff;

assign csr_p_priority52_rdata[2:0] = csr_p_priority52_priority52_ff;

assign csr_p_priority52_priority52_out = csr_p_priority52_priority52_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority52_priority52_ff <= 3'h0;
    end else  begin
     if (csr_p_priority52_wen) begin
            if (wstrb[0]) begin
                csr_p_priority52_priority52_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority52_priority52_ff <= csr_p_priority52_priority52_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xd4] - P_PRIORITY53 - Priority Register IRQ53 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority53_rdata;
assign csr_p_priority53_rdata[31:3] = 29'h0;

wire csr_p_priority53_wen;
assign csr_p_priority53_wen = wen && (waddr == 32'hd4);

wire csr_p_priority53_ren;
assign csr_p_priority53_ren = ren && (raddr == 32'hd4);
reg csr_p_priority53_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority53_ren_ff <= 1'b0;
    end else begin
        csr_p_priority53_ren_ff <= csr_p_priority53_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY53[2:0] - PRIORITY53 - Priority level of IRQ53. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority53_priority53_ff;

assign csr_p_priority53_rdata[2:0] = csr_p_priority53_priority53_ff;

assign csr_p_priority53_priority53_out = csr_p_priority53_priority53_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority53_priority53_ff <= 3'h0;
    end else  begin
     if (csr_p_priority53_wen) begin
            if (wstrb[0]) begin
                csr_p_priority53_priority53_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority53_priority53_ff <= csr_p_priority53_priority53_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xd8] - P_PRIORITY54 - Priority Register IRQ54 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority54_rdata;
assign csr_p_priority54_rdata[31:3] = 29'h0;

wire csr_p_priority54_wen;
assign csr_p_priority54_wen = wen && (waddr == 32'hd8);

wire csr_p_priority54_ren;
assign csr_p_priority54_ren = ren && (raddr == 32'hd8);
reg csr_p_priority54_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority54_ren_ff <= 1'b0;
    end else begin
        csr_p_priority54_ren_ff <= csr_p_priority54_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY54[2:0] - PRIORITY54 - Priority level of IRQ54. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority54_priority54_ff;

assign csr_p_priority54_rdata[2:0] = csr_p_priority54_priority54_ff;

assign csr_p_priority54_priority54_out = csr_p_priority54_priority54_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority54_priority54_ff <= 3'h0;
    end else  begin
     if (csr_p_priority54_wen) begin
            if (wstrb[0]) begin
                csr_p_priority54_priority54_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority54_priority54_ff <= csr_p_priority54_priority54_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xdc] - P_PRIORITY55 - Priority Register IRQ55 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority55_rdata;
assign csr_p_priority55_rdata[31:3] = 29'h0;

wire csr_p_priority55_wen;
assign csr_p_priority55_wen = wen && (waddr == 32'hdc);

wire csr_p_priority55_ren;
assign csr_p_priority55_ren = ren && (raddr == 32'hdc);
reg csr_p_priority55_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority55_ren_ff <= 1'b0;
    end else begin
        csr_p_priority55_ren_ff <= csr_p_priority55_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY55[2:0] - PRIORITY55 - Priority level of IRQ55. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority55_priority55_ff;

assign csr_p_priority55_rdata[2:0] = csr_p_priority55_priority55_ff;

assign csr_p_priority55_priority55_out = csr_p_priority55_priority55_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority55_priority55_ff <= 3'h0;
    end else  begin
     if (csr_p_priority55_wen) begin
            if (wstrb[0]) begin
                csr_p_priority55_priority55_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority55_priority55_ff <= csr_p_priority55_priority55_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xe0] - P_PRIORITY56 - Priority Register IRQ56 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority56_rdata;
assign csr_p_priority56_rdata[31:3] = 29'h0;

wire csr_p_priority56_wen;
assign csr_p_priority56_wen = wen && (waddr == 32'he0);

wire csr_p_priority56_ren;
assign csr_p_priority56_ren = ren && (raddr == 32'he0);
reg csr_p_priority56_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority56_ren_ff <= 1'b0;
    end else begin
        csr_p_priority56_ren_ff <= csr_p_priority56_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY56[2:0] - PRIORITY56 - Priority level of IRQ56. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority56_priority56_ff;

assign csr_p_priority56_rdata[2:0] = csr_p_priority56_priority56_ff;

assign csr_p_priority56_priority56_out = csr_p_priority56_priority56_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority56_priority56_ff <= 3'h0;
    end else  begin
     if (csr_p_priority56_wen) begin
            if (wstrb[0]) begin
                csr_p_priority56_priority56_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority56_priority56_ff <= csr_p_priority56_priority56_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xe4] - P_PRIORITY57 - Priority Register IRQ57 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority57_rdata;
assign csr_p_priority57_rdata[31:3] = 29'h0;

wire csr_p_priority57_wen;
assign csr_p_priority57_wen = wen && (waddr == 32'he4);

wire csr_p_priority57_ren;
assign csr_p_priority57_ren = ren && (raddr == 32'he4);
reg csr_p_priority57_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority57_ren_ff <= 1'b0;
    end else begin
        csr_p_priority57_ren_ff <= csr_p_priority57_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY57[2:0] - PRIORITY57 - Priority level of IRQ57. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority57_priority57_ff;

assign csr_p_priority57_rdata[2:0] = csr_p_priority57_priority57_ff;

assign csr_p_priority57_priority57_out = csr_p_priority57_priority57_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority57_priority57_ff <= 3'h0;
    end else  begin
     if (csr_p_priority57_wen) begin
            if (wstrb[0]) begin
                csr_p_priority57_priority57_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority57_priority57_ff <= csr_p_priority57_priority57_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xe8] - P_PRIORITY58 - Priority Register IRQ58 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority58_rdata;
assign csr_p_priority58_rdata[31:3] = 29'h0;

wire csr_p_priority58_wen;
assign csr_p_priority58_wen = wen && (waddr == 32'he8);

wire csr_p_priority58_ren;
assign csr_p_priority58_ren = ren && (raddr == 32'he8);
reg csr_p_priority58_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority58_ren_ff <= 1'b0;
    end else begin
        csr_p_priority58_ren_ff <= csr_p_priority58_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY58[2:0] - PRIORITY58 - Priority level of IRQ58. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority58_priority58_ff;

assign csr_p_priority58_rdata[2:0] = csr_p_priority58_priority58_ff;

assign csr_p_priority58_priority58_out = csr_p_priority58_priority58_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority58_priority58_ff <= 3'h0;
    end else  begin
     if (csr_p_priority58_wen) begin
            if (wstrb[0]) begin
                csr_p_priority58_priority58_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority58_priority58_ff <= csr_p_priority58_priority58_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xec] - P_PRIORITY59 - Priority Register IRQ59 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority59_rdata;
assign csr_p_priority59_rdata[31:3] = 29'h0;

wire csr_p_priority59_wen;
assign csr_p_priority59_wen = wen && (waddr == 32'hec);

wire csr_p_priority59_ren;
assign csr_p_priority59_ren = ren && (raddr == 32'hec);
reg csr_p_priority59_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority59_ren_ff <= 1'b0;
    end else begin
        csr_p_priority59_ren_ff <= csr_p_priority59_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY59[2:0] - PRIORITY59 - Priority level of IRQ59. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority59_priority59_ff;

assign csr_p_priority59_rdata[2:0] = csr_p_priority59_priority59_ff;

assign csr_p_priority59_priority59_out = csr_p_priority59_priority59_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority59_priority59_ff <= 3'h0;
    end else  begin
     if (csr_p_priority59_wen) begin
            if (wstrb[0]) begin
                csr_p_priority59_priority59_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority59_priority59_ff <= csr_p_priority59_priority59_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xf0] - P_PRIORITY60 - Priority Register IRQ60 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority60_rdata;
assign csr_p_priority60_rdata[31:3] = 29'h0;

wire csr_p_priority60_wen;
assign csr_p_priority60_wen = wen && (waddr == 32'hf0);

wire csr_p_priority60_ren;
assign csr_p_priority60_ren = ren && (raddr == 32'hf0);
reg csr_p_priority60_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority60_ren_ff <= 1'b0;
    end else begin
        csr_p_priority60_ren_ff <= csr_p_priority60_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY60[2:0] - PRIORITY60 - Priority level of IRQ60. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority60_priority60_ff;

assign csr_p_priority60_rdata[2:0] = csr_p_priority60_priority60_ff;

assign csr_p_priority60_priority60_out = csr_p_priority60_priority60_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority60_priority60_ff <= 3'h0;
    end else  begin
     if (csr_p_priority60_wen) begin
            if (wstrb[0]) begin
                csr_p_priority60_priority60_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority60_priority60_ff <= csr_p_priority60_priority60_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xf4] - P_PRIORITY61 - Priority Register IRQ61 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority61_rdata;
assign csr_p_priority61_rdata[31:3] = 29'h0;

wire csr_p_priority61_wen;
assign csr_p_priority61_wen = wen && (waddr == 32'hf4);

wire csr_p_priority61_ren;
assign csr_p_priority61_ren = ren && (raddr == 32'hf4);
reg csr_p_priority61_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority61_ren_ff <= 1'b0;
    end else begin
        csr_p_priority61_ren_ff <= csr_p_priority61_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY61[2:0] - PRIORITY61 - Priority level of IRQ61. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority61_priority61_ff;

assign csr_p_priority61_rdata[2:0] = csr_p_priority61_priority61_ff;

assign csr_p_priority61_priority61_out = csr_p_priority61_priority61_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority61_priority61_ff <= 3'h0;
    end else  begin
     if (csr_p_priority61_wen) begin
            if (wstrb[0]) begin
                csr_p_priority61_priority61_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority61_priority61_ff <= csr_p_priority61_priority61_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xf8] - P_PRIORITY62 - Priority Register IRQ62 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority62_rdata;
assign csr_p_priority62_rdata[31:3] = 29'h0;

wire csr_p_priority62_wen;
assign csr_p_priority62_wen = wen && (waddr == 32'hf8);

wire csr_p_priority62_ren;
assign csr_p_priority62_ren = ren && (raddr == 32'hf8);
reg csr_p_priority62_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority62_ren_ff <= 1'b0;
    end else begin
        csr_p_priority62_ren_ff <= csr_p_priority62_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY62[2:0] - PRIORITY62 - Priority level of IRQ62. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority62_priority62_ff;

assign csr_p_priority62_rdata[2:0] = csr_p_priority62_priority62_ff;

assign csr_p_priority62_priority62_out = csr_p_priority62_priority62_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority62_priority62_ff <= 3'h0;
    end else  begin
     if (csr_p_priority62_wen) begin
            if (wstrb[0]) begin
                csr_p_priority62_priority62_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority62_priority62_ff <= csr_p_priority62_priority62_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0xfc] - P_PRIORITY63 - Priority Register IRQ63 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority63_rdata;
assign csr_p_priority63_rdata[31:3] = 29'h0;

wire csr_p_priority63_wen;
assign csr_p_priority63_wen = wen && (waddr == 32'hfc);

wire csr_p_priority63_ren;
assign csr_p_priority63_ren = ren && (raddr == 32'hfc);
reg csr_p_priority63_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority63_ren_ff <= 1'b0;
    end else begin
        csr_p_priority63_ren_ff <= csr_p_priority63_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY63[2:0] - PRIORITY63 - Priority level of IRQ63. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority63_priority63_ff;

assign csr_p_priority63_rdata[2:0] = csr_p_priority63_priority63_ff;

assign csr_p_priority63_priority63_out = csr_p_priority63_priority63_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority63_priority63_ff <= 3'h0;
    end else  begin
     if (csr_p_priority63_wen) begin
            if (wstrb[0]) begin
                csr_p_priority63_priority63_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority63_priority63_ff <= csr_p_priority63_priority63_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x100] - P_PRIORITY64 - Priority Register IRQ64 (3-bit, 8 levels)
//------------------------------------------------------------------------------
wire [31:0] csr_p_priority64_rdata;
assign csr_p_priority64_rdata[31:3] = 29'h0;

wire csr_p_priority64_wen;
assign csr_p_priority64_wen = wen && (waddr == 32'h100);

wire csr_p_priority64_ren;
assign csr_p_priority64_ren = ren && (raddr == 32'h100);
reg csr_p_priority64_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority64_ren_ff <= 1'b0;
    end else begin
        csr_p_priority64_ren_ff <= csr_p_priority64_ren;
    end
end
//---------------------
// Bit field:
// P_PRIORITY64[2:0] - PRIORITY64 - Priority level of IRQ64. 0 = Interrupt disabled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_priority64_priority64_ff;

assign csr_p_priority64_rdata[2:0] = csr_p_priority64_priority64_ff;

assign csr_p_priority64_priority64_out = csr_p_priority64_priority64_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_priority64_priority64_ff <= 3'h0;
    end else  begin
     if (csr_p_priority64_wen) begin
            if (wstrb[0]) begin
                csr_p_priority64_priority64_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_priority64_priority64_ff <= csr_p_priority64_priority64_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x1000] - P_PENDING0 - Pending bits for IRQ1–IRQ31
//------------------------------------------------------------------------------
wire [31:0] csr_p_pending0_rdata;
assign csr_p_pending0_rdata[0] = 1'b0;


wire csr_p_pending0_ren;
assign csr_p_pending0_ren = ren && (raddr == 32'h1000);
reg csr_p_pending0_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_ren_ff <= 1'b0;
    end else begin
        csr_p_pending0_ren_ff <= csr_p_pending0_ren;
    end
end
//---------------------
// Bit field:
// P_PENDING0[1] - PENDING_IRQ1 - Pending status of IRQ1
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq1_ff;

assign csr_p_pending0_rdata[1] = csr_p_pending0_pending_irq1_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq1_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq1_ff <= csr_p_pending0_pending_irq1_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[2] - PENDING_IRQ2 - Pending status of IRQ2
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq2_ff;

assign csr_p_pending0_rdata[2] = csr_p_pending0_pending_irq2_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq2_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq2_ff <= csr_p_pending0_pending_irq2_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[3] - PENDING_IRQ3 - Pending status of IRQ3
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq3_ff;

assign csr_p_pending0_rdata[3] = csr_p_pending0_pending_irq3_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq3_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq3_ff <= csr_p_pending0_pending_irq3_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[4] - PENDING_IRQ4 - Pending status of IRQ4
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq4_ff;

assign csr_p_pending0_rdata[4] = csr_p_pending0_pending_irq4_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq4_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq4_ff <= csr_p_pending0_pending_irq4_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[5] - PENDING_IRQ5 - Pending status of IRQ5
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq5_ff;

assign csr_p_pending0_rdata[5] = csr_p_pending0_pending_irq5_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq5_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq5_ff <= csr_p_pending0_pending_irq5_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[6] - PENDING_IRQ6 - Pending status of IRQ6
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq6_ff;

assign csr_p_pending0_rdata[6] = csr_p_pending0_pending_irq6_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq6_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq6_ff <= csr_p_pending0_pending_irq6_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[7] - PENDING_IRQ7 - Pending status of IRQ7
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq7_ff;

assign csr_p_pending0_rdata[7] = csr_p_pending0_pending_irq7_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq7_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq7_ff <= csr_p_pending0_pending_irq7_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[8] - PENDING_IRQ8 - Pending status of IRQ8
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq8_ff;

assign csr_p_pending0_rdata[8] = csr_p_pending0_pending_irq8_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq8_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq8_ff <= csr_p_pending0_pending_irq8_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[9] - PENDING_IRQ9 - Pending status of IRQ9
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq9_ff;

assign csr_p_pending0_rdata[9] = csr_p_pending0_pending_irq9_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq9_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq9_ff <= csr_p_pending0_pending_irq9_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[10] - PENDING_IRQ10 - Pending status of IRQ10
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq10_ff;

assign csr_p_pending0_rdata[10] = csr_p_pending0_pending_irq10_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq10_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq10_ff <= csr_p_pending0_pending_irq10_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[11] - PENDING_IRQ11 - Pending status of IRQ11
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq11_ff;

assign csr_p_pending0_rdata[11] = csr_p_pending0_pending_irq11_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq11_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq11_ff <= csr_p_pending0_pending_irq11_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[12] - PENDING_IRQ12 - Pending status of IRQ12
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq12_ff;

assign csr_p_pending0_rdata[12] = csr_p_pending0_pending_irq12_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq12_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq12_ff <= csr_p_pending0_pending_irq12_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[13] - PENDING_IRQ13 - Pending status of IRQ13
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq13_ff;

assign csr_p_pending0_rdata[13] = csr_p_pending0_pending_irq13_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq13_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq13_ff <= csr_p_pending0_pending_irq13_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[14] - PENDING_IRQ14 - Pending status of IRQ14
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq14_ff;

assign csr_p_pending0_rdata[14] = csr_p_pending0_pending_irq14_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq14_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq14_ff <= csr_p_pending0_pending_irq14_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[15] - PENDING_IRQ15 - Pending status of IRQ15
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq15_ff;

assign csr_p_pending0_rdata[15] = csr_p_pending0_pending_irq15_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq15_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq15_ff <= csr_p_pending0_pending_irq15_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[16] - PENDING_IRQ16 - Pending status of IRQ16
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq16_ff;

assign csr_p_pending0_rdata[16] = csr_p_pending0_pending_irq16_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq16_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq16_ff <= csr_p_pending0_pending_irq16_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[17] - PENDING_IRQ17 - Pending status of IRQ17
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq17_ff;

assign csr_p_pending0_rdata[17] = csr_p_pending0_pending_irq17_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq17_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq17_ff <= csr_p_pending0_pending_irq17_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[18] - PENDING_IRQ18 - Pending status of IRQ18
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq18_ff;

assign csr_p_pending0_rdata[18] = csr_p_pending0_pending_irq18_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq18_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq18_ff <= csr_p_pending0_pending_irq18_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[19] - PENDING_IRQ19 - Pending status of IRQ19
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq19_ff;

assign csr_p_pending0_rdata[19] = csr_p_pending0_pending_irq19_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq19_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq19_ff <= csr_p_pending0_pending_irq19_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[20] - PENDING_IRQ20 - Pending status of IRQ20
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq20_ff;

assign csr_p_pending0_rdata[20] = csr_p_pending0_pending_irq20_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq20_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq20_ff <= csr_p_pending0_pending_irq20_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[21] - PENDING_IRQ21 - Pending status of IRQ21
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq21_ff;

assign csr_p_pending0_rdata[21] = csr_p_pending0_pending_irq21_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq21_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq21_ff <= csr_p_pending0_pending_irq21_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[22] - PENDING_IRQ22 - Pending status of IRQ22
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq22_ff;

assign csr_p_pending0_rdata[22] = csr_p_pending0_pending_irq22_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq22_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq22_ff <= csr_p_pending0_pending_irq22_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[23] - PENDING_IRQ23 - Pending status of IRQ23
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq23_ff;

assign csr_p_pending0_rdata[23] = csr_p_pending0_pending_irq23_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq23_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq23_ff <= csr_p_pending0_pending_irq23_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[24] - PENDING_IRQ24 - Pending status of IRQ24
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq24_ff;

assign csr_p_pending0_rdata[24] = csr_p_pending0_pending_irq24_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq24_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq24_ff <= csr_p_pending0_pending_irq24_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[25] - PENDING_IRQ25 - Pending status of IRQ25
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq25_ff;

assign csr_p_pending0_rdata[25] = csr_p_pending0_pending_irq25_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq25_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq25_ff <= csr_p_pending0_pending_irq25_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[26] - PENDING_IRQ26 - Pending status of IRQ26
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq26_ff;

assign csr_p_pending0_rdata[26] = csr_p_pending0_pending_irq26_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq26_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq26_ff <= csr_p_pending0_pending_irq26_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[27] - PENDING_IRQ27 - Pending status of IRQ27
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq27_ff;

assign csr_p_pending0_rdata[27] = csr_p_pending0_pending_irq27_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq27_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq27_ff <= csr_p_pending0_pending_irq27_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[28] - PENDING_IRQ28 - Pending status of IRQ28
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq28_ff;

assign csr_p_pending0_rdata[28] = csr_p_pending0_pending_irq28_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq28_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq28_ff <= csr_p_pending0_pending_irq28_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[29] - PENDING_IRQ29 - Pending status of IRQ29
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq29_ff;

assign csr_p_pending0_rdata[29] = csr_p_pending0_pending_irq29_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq29_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq29_ff <= csr_p_pending0_pending_irq29_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[30] - PENDING_IRQ30 - Pending status of IRQ30
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq30_ff;

assign csr_p_pending0_rdata[30] = csr_p_pending0_pending_irq30_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq30_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq30_ff <= csr_p_pending0_pending_irq30_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING0[31] - PENDING_IRQ31 - Pending status of IRQ31
// access: ro, hardware: i
//---------------------
reg  csr_p_pending0_pending_irq31_ff;

assign csr_p_pending0_rdata[31] = csr_p_pending0_pending_irq31_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending0_pending_irq31_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending0_pending_irq31_ff <= csr_p_pending0_pending_irq31_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x1004] - P_PENDING1 - Pending bits for IRQ32–IRQ62
//------------------------------------------------------------------------------
wire [31:0] csr_p_pending1_rdata;
assign csr_p_pending1_rdata[0] = 1'b0;


wire csr_p_pending1_ren;
assign csr_p_pending1_ren = ren && (raddr == 32'h1004);
reg csr_p_pending1_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_ren_ff <= 1'b0;
    end else begin
        csr_p_pending1_ren_ff <= csr_p_pending1_ren;
    end
end
//---------------------
// Bit field:
// P_PENDING1[1] - PENDING_IRQ32 - Pending status of IRQ32
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq32_ff;

assign csr_p_pending1_rdata[1] = csr_p_pending1_pending_irq32_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq32_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq32_ff <= csr_p_pending1_pending_irq32_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[2] - PENDING_IRQ33 - Pending status of IRQ33
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq33_ff;

assign csr_p_pending1_rdata[2] = csr_p_pending1_pending_irq33_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq33_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq33_ff <= csr_p_pending1_pending_irq33_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[3] - PENDING_IRQ34 - Pending status of IRQ34
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq34_ff;

assign csr_p_pending1_rdata[3] = csr_p_pending1_pending_irq34_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq34_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq34_ff <= csr_p_pending1_pending_irq34_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[4] - PENDING_IRQ35 - Pending status of IRQ35
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq35_ff;

assign csr_p_pending1_rdata[4] = csr_p_pending1_pending_irq35_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq35_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq35_ff <= csr_p_pending1_pending_irq35_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[5] - PENDING_IRQ36 - Pending status of IRQ36
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq36_ff;

assign csr_p_pending1_rdata[5] = csr_p_pending1_pending_irq36_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq36_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq36_ff <= csr_p_pending1_pending_irq36_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[6] - PENDING_IRQ37 - Pending status of IRQ37
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq37_ff;

assign csr_p_pending1_rdata[6] = csr_p_pending1_pending_irq37_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq37_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq37_ff <= csr_p_pending1_pending_irq37_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[7] - PENDING_IRQ38 - Pending status of IRQ38
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq38_ff;

assign csr_p_pending1_rdata[7] = csr_p_pending1_pending_irq38_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq38_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq38_ff <= csr_p_pending1_pending_irq38_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[8] - PENDING_IRQ39 - Pending status of IRQ39
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq39_ff;

assign csr_p_pending1_rdata[8] = csr_p_pending1_pending_irq39_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq39_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq39_ff <= csr_p_pending1_pending_irq39_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[9] - PENDING_IRQ40 - Pending status of IRQ40
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq40_ff;

assign csr_p_pending1_rdata[9] = csr_p_pending1_pending_irq40_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq40_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq40_ff <= csr_p_pending1_pending_irq40_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[10] - PENDING_IRQ41 - Pending status of IRQ41
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq41_ff;

assign csr_p_pending1_rdata[10] = csr_p_pending1_pending_irq41_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq41_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq41_ff <= csr_p_pending1_pending_irq41_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[11] - PENDING_IRQ42 - Pending status of IRQ42
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq42_ff;

assign csr_p_pending1_rdata[11] = csr_p_pending1_pending_irq42_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq42_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq42_ff <= csr_p_pending1_pending_irq42_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[12] - PENDING_IRQ43 - Pending status of IRQ43
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq43_ff;

assign csr_p_pending1_rdata[12] = csr_p_pending1_pending_irq43_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq43_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq43_ff <= csr_p_pending1_pending_irq43_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[13] - PENDING_IRQ44 - Pending status of IRQ44
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq44_ff;

assign csr_p_pending1_rdata[13] = csr_p_pending1_pending_irq44_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq44_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq44_ff <= csr_p_pending1_pending_irq44_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[14] - PENDING_IRQ45 - Pending status of IRQ45
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq45_ff;

assign csr_p_pending1_rdata[14] = csr_p_pending1_pending_irq45_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq45_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq45_ff <= csr_p_pending1_pending_irq45_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[15] - PENDING_IRQ46 - Pending status of IRQ46
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq46_ff;

assign csr_p_pending1_rdata[15] = csr_p_pending1_pending_irq46_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq46_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq46_ff <= csr_p_pending1_pending_irq46_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[16] - PENDING_IRQ47 - Pending status of IRQ47
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq47_ff;

assign csr_p_pending1_rdata[16] = csr_p_pending1_pending_irq47_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq47_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq47_ff <= csr_p_pending1_pending_irq47_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[17] - PENDING_IRQ48 - Pending status of IRQ48
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq48_ff;

assign csr_p_pending1_rdata[17] = csr_p_pending1_pending_irq48_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq48_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq48_ff <= csr_p_pending1_pending_irq48_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[18] - PENDING_IRQ49 - Pending status of IRQ49
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq49_ff;

assign csr_p_pending1_rdata[18] = csr_p_pending1_pending_irq49_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq49_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq49_ff <= csr_p_pending1_pending_irq49_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[19] - PENDING_IRQ50 - Pending status of IRQ50
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq50_ff;

assign csr_p_pending1_rdata[19] = csr_p_pending1_pending_irq50_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq50_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq50_ff <= csr_p_pending1_pending_irq50_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[20] - PENDING_IRQ51 - Pending status of IRQ51
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq51_ff;

assign csr_p_pending1_rdata[20] = csr_p_pending1_pending_irq51_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq51_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq51_ff <= csr_p_pending1_pending_irq51_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[21] - PENDING_IRQ52 - Pending status of IRQ52
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq52_ff;

assign csr_p_pending1_rdata[21] = csr_p_pending1_pending_irq52_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq52_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq52_ff <= csr_p_pending1_pending_irq52_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[22] - PENDING_IRQ53 - Pending status of IRQ53
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq53_ff;

assign csr_p_pending1_rdata[22] = csr_p_pending1_pending_irq53_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq53_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq53_ff <= csr_p_pending1_pending_irq53_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[23] - PENDING_IRQ54 - Pending status of IRQ54
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq54_ff;

assign csr_p_pending1_rdata[23] = csr_p_pending1_pending_irq54_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq54_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq54_ff <= csr_p_pending1_pending_irq54_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[24] - PENDING_IRQ55 - Pending status of IRQ55
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq55_ff;

assign csr_p_pending1_rdata[24] = csr_p_pending1_pending_irq55_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq55_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq55_ff <= csr_p_pending1_pending_irq55_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[25] - PENDING_IRQ56 - Pending status of IRQ56
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq56_ff;

assign csr_p_pending1_rdata[25] = csr_p_pending1_pending_irq56_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq56_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq56_ff <= csr_p_pending1_pending_irq56_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[26] - PENDING_IRQ57 - Pending status of IRQ57
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq57_ff;

assign csr_p_pending1_rdata[26] = csr_p_pending1_pending_irq57_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq57_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq57_ff <= csr_p_pending1_pending_irq57_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[27] - PENDING_IRQ58 - Pending status of IRQ58
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq58_ff;

assign csr_p_pending1_rdata[27] = csr_p_pending1_pending_irq58_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq58_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq58_ff <= csr_p_pending1_pending_irq58_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[28] - PENDING_IRQ59 - Pending status of IRQ59
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq59_ff;

assign csr_p_pending1_rdata[28] = csr_p_pending1_pending_irq59_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq59_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq59_ff <= csr_p_pending1_pending_irq59_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[29] - PENDING_IRQ60 - Pending status of IRQ60
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq60_ff;

assign csr_p_pending1_rdata[29] = csr_p_pending1_pending_irq60_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq60_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq60_ff <= csr_p_pending1_pending_irq60_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[30] - PENDING_IRQ61 - Pending status of IRQ61
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq61_ff;

assign csr_p_pending1_rdata[30] = csr_p_pending1_pending_irq61_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq61_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq61_ff <= csr_p_pending1_pending_irq61_in;
        end
    end
end


//---------------------
// Bit field:
// P_PENDING1[31] - PENDING_IRQ62 - Pending status of IRQ62
// access: ro, hardware: i
//---------------------
reg  csr_p_pending1_pending_irq62_ff;

assign csr_p_pending1_rdata[31] = csr_p_pending1_pending_irq62_ff;


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_pending1_pending_irq62_ff <= 1'b0;
    end else  begin
              begin            csr_p_pending1_pending_irq62_ff <= csr_p_pending1_pending_irq62_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x2000] - P_ENABLE0 - Enable bits for IRQ1–IRQ31
//------------------------------------------------------------------------------
wire [31:0] csr_p_enable0_rdata;
assign csr_p_enable0_rdata[0] = 1'b0;

wire csr_p_enable0_wen;
assign csr_p_enable0_wen = wen && (waddr == 32'h2000);

wire csr_p_enable0_ren;
assign csr_p_enable0_ren = ren && (raddr == 32'h2000);
reg csr_p_enable0_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_ren_ff <= 1'b0;
    end else begin
        csr_p_enable0_ren_ff <= csr_p_enable0_ren;
    end
end
//---------------------
// Bit field:
// P_ENABLE0[1] - ENABLE_IRQ1 - Enable mask for IRQ1
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq1_ff;

assign csr_p_enable0_rdata[1] = csr_p_enable0_enable_irq1_ff;

assign csr_p_enable0_enable_irq1_out = csr_p_enable0_enable_irq1_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq1_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq1_ff <= wdata[1];
            end
        end else begin
            csr_p_enable0_enable_irq1_ff <= csr_p_enable0_enable_irq1_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[2] - ENABLE_IRQ2 - Enable mask for IRQ2
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq2_ff;

assign csr_p_enable0_rdata[2] = csr_p_enable0_enable_irq2_ff;

assign csr_p_enable0_enable_irq2_out = csr_p_enable0_enable_irq2_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq2_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq2_ff <= wdata[2];
            end
        end else begin
            csr_p_enable0_enable_irq2_ff <= csr_p_enable0_enable_irq2_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[3] - ENABLE_IRQ3 - Enable mask for IRQ3
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq3_ff;

assign csr_p_enable0_rdata[3] = csr_p_enable0_enable_irq3_ff;

assign csr_p_enable0_enable_irq3_out = csr_p_enable0_enable_irq3_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq3_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq3_ff <= wdata[3];
            end
        end else begin
            csr_p_enable0_enable_irq3_ff <= csr_p_enable0_enable_irq3_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[4] - ENABLE_IRQ4 - Enable mask for IRQ4
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq4_ff;

assign csr_p_enable0_rdata[4] = csr_p_enable0_enable_irq4_ff;

assign csr_p_enable0_enable_irq4_out = csr_p_enable0_enable_irq4_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq4_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq4_ff <= wdata[4];
            end
        end else begin
            csr_p_enable0_enable_irq4_ff <= csr_p_enable0_enable_irq4_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[5] - ENABLE_IRQ5 - Enable mask for IRQ5
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq5_ff;

assign csr_p_enable0_rdata[5] = csr_p_enable0_enable_irq5_ff;

assign csr_p_enable0_enable_irq5_out = csr_p_enable0_enable_irq5_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq5_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq5_ff <= wdata[5];
            end
        end else begin
            csr_p_enable0_enable_irq5_ff <= csr_p_enable0_enable_irq5_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[6] - ENABLE_IRQ6 - Enable mask for IRQ6
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq6_ff;

assign csr_p_enable0_rdata[6] = csr_p_enable0_enable_irq6_ff;

assign csr_p_enable0_enable_irq6_out = csr_p_enable0_enable_irq6_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq6_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq6_ff <= wdata[6];
            end
        end else begin
            csr_p_enable0_enable_irq6_ff <= csr_p_enable0_enable_irq6_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[7] - ENABLE_IRQ7 - Enable mask for IRQ7
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq7_ff;

assign csr_p_enable0_rdata[7] = csr_p_enable0_enable_irq7_ff;

assign csr_p_enable0_enable_irq7_out = csr_p_enable0_enable_irq7_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq7_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[0]) begin
                csr_p_enable0_enable_irq7_ff <= wdata[7];
            end
        end else begin
            csr_p_enable0_enable_irq7_ff <= csr_p_enable0_enable_irq7_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[8] - ENABLE_IRQ8 - Enable mask for IRQ8
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq8_ff;

assign csr_p_enable0_rdata[8] = csr_p_enable0_enable_irq8_ff;

assign csr_p_enable0_enable_irq8_out = csr_p_enable0_enable_irq8_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq8_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq8_ff <= wdata[8];
            end
        end else begin
            csr_p_enable0_enable_irq8_ff <= csr_p_enable0_enable_irq8_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[9] - ENABLE_IRQ9 - Enable mask for IRQ9
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq9_ff;

assign csr_p_enable0_rdata[9] = csr_p_enable0_enable_irq9_ff;

assign csr_p_enable0_enable_irq9_out = csr_p_enable0_enable_irq9_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq9_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq9_ff <= wdata[9];
            end
        end else begin
            csr_p_enable0_enable_irq9_ff <= csr_p_enable0_enable_irq9_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[10] - ENABLE_IRQ10 - Enable mask for IRQ10
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq10_ff;

assign csr_p_enable0_rdata[10] = csr_p_enable0_enable_irq10_ff;

assign csr_p_enable0_enable_irq10_out = csr_p_enable0_enable_irq10_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq10_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq10_ff <= wdata[10];
            end
        end else begin
            csr_p_enable0_enable_irq10_ff <= csr_p_enable0_enable_irq10_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[11] - ENABLE_IRQ11 - Enable mask for IRQ11
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq11_ff;

assign csr_p_enable0_rdata[11] = csr_p_enable0_enable_irq11_ff;

assign csr_p_enable0_enable_irq11_out = csr_p_enable0_enable_irq11_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq11_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq11_ff <= wdata[11];
            end
        end else begin
            csr_p_enable0_enable_irq11_ff <= csr_p_enable0_enable_irq11_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[12] - ENABLE_IRQ12 - Enable mask for IRQ12
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq12_ff;

assign csr_p_enable0_rdata[12] = csr_p_enable0_enable_irq12_ff;

assign csr_p_enable0_enable_irq12_out = csr_p_enable0_enable_irq12_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq12_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq12_ff <= wdata[12];
            end
        end else begin
            csr_p_enable0_enable_irq12_ff <= csr_p_enable0_enable_irq12_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[13] - ENABLE_IRQ13 - Enable mask for IRQ13
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq13_ff;

assign csr_p_enable0_rdata[13] = csr_p_enable0_enable_irq13_ff;

assign csr_p_enable0_enable_irq13_out = csr_p_enable0_enable_irq13_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq13_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq13_ff <= wdata[13];
            end
        end else begin
            csr_p_enable0_enable_irq13_ff <= csr_p_enable0_enable_irq13_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[14] - ENABLE_IRQ14 - Enable mask for IRQ14
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq14_ff;

assign csr_p_enable0_rdata[14] = csr_p_enable0_enable_irq14_ff;

assign csr_p_enable0_enable_irq14_out = csr_p_enable0_enable_irq14_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq14_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq14_ff <= wdata[14];
            end
        end else begin
            csr_p_enable0_enable_irq14_ff <= csr_p_enable0_enable_irq14_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[15] - ENABLE_IRQ15 - Enable mask for IRQ15
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq15_ff;

assign csr_p_enable0_rdata[15] = csr_p_enable0_enable_irq15_ff;

assign csr_p_enable0_enable_irq15_out = csr_p_enable0_enable_irq15_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq15_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[1]) begin
                csr_p_enable0_enable_irq15_ff <= wdata[15];
            end
        end else begin
            csr_p_enable0_enable_irq15_ff <= csr_p_enable0_enable_irq15_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[16] - ENABLE_IRQ16 - Enable mask for IRQ16
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq16_ff;

assign csr_p_enable0_rdata[16] = csr_p_enable0_enable_irq16_ff;

assign csr_p_enable0_enable_irq16_out = csr_p_enable0_enable_irq16_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq16_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq16_ff <= wdata[16];
            end
        end else begin
            csr_p_enable0_enable_irq16_ff <= csr_p_enable0_enable_irq16_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[17] - ENABLE_IRQ17 - Enable mask for IRQ17
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq17_ff;

assign csr_p_enable0_rdata[17] = csr_p_enable0_enable_irq17_ff;

assign csr_p_enable0_enable_irq17_out = csr_p_enable0_enable_irq17_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq17_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq17_ff <= wdata[17];
            end
        end else begin
            csr_p_enable0_enable_irq17_ff <= csr_p_enable0_enable_irq17_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[18] - ENABLE_IRQ18 - Enable mask for IRQ18
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq18_ff;

assign csr_p_enable0_rdata[18] = csr_p_enable0_enable_irq18_ff;

assign csr_p_enable0_enable_irq18_out = csr_p_enable0_enable_irq18_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq18_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq18_ff <= wdata[18];
            end
        end else begin
            csr_p_enable0_enable_irq18_ff <= csr_p_enable0_enable_irq18_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[19] - ENABLE_IRQ19 - Enable mask for IRQ19
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq19_ff;

assign csr_p_enable0_rdata[19] = csr_p_enable0_enable_irq19_ff;

assign csr_p_enable0_enable_irq19_out = csr_p_enable0_enable_irq19_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq19_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq19_ff <= wdata[19];
            end
        end else begin
            csr_p_enable0_enable_irq19_ff <= csr_p_enable0_enable_irq19_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[20] - ENABLE_IRQ20 - Enable mask for IRQ20
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq20_ff;

assign csr_p_enable0_rdata[20] = csr_p_enable0_enable_irq20_ff;

assign csr_p_enable0_enable_irq20_out = csr_p_enable0_enable_irq20_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq20_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq20_ff <= wdata[20];
            end
        end else begin
            csr_p_enable0_enable_irq20_ff <= csr_p_enable0_enable_irq20_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[21] - ENABLE_IRQ21 - Enable mask for IRQ21
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq21_ff;

assign csr_p_enable0_rdata[21] = csr_p_enable0_enable_irq21_ff;

assign csr_p_enable0_enable_irq21_out = csr_p_enable0_enable_irq21_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq21_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq21_ff <= wdata[21];
            end
        end else begin
            csr_p_enable0_enable_irq21_ff <= csr_p_enable0_enable_irq21_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[22] - ENABLE_IRQ22 - Enable mask for IRQ22
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq22_ff;

assign csr_p_enable0_rdata[22] = csr_p_enable0_enable_irq22_ff;

assign csr_p_enable0_enable_irq22_out = csr_p_enable0_enable_irq22_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq22_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq22_ff <= wdata[22];
            end
        end else begin
            csr_p_enable0_enable_irq22_ff <= csr_p_enable0_enable_irq22_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[23] - ENABLE_IRQ23 - Enable mask for IRQ23
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq23_ff;

assign csr_p_enable0_rdata[23] = csr_p_enable0_enable_irq23_ff;

assign csr_p_enable0_enable_irq23_out = csr_p_enable0_enable_irq23_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq23_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[2]) begin
                csr_p_enable0_enable_irq23_ff <= wdata[23];
            end
        end else begin
            csr_p_enable0_enable_irq23_ff <= csr_p_enable0_enable_irq23_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[24] - ENABLE_IRQ24 - Enable mask for IRQ24
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq24_ff;

assign csr_p_enable0_rdata[24] = csr_p_enable0_enable_irq24_ff;

assign csr_p_enable0_enable_irq24_out = csr_p_enable0_enable_irq24_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq24_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq24_ff <= wdata[24];
            end
        end else begin
            csr_p_enable0_enable_irq24_ff <= csr_p_enable0_enable_irq24_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[25] - ENABLE_IRQ25 - Enable mask for IRQ25
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq25_ff;

assign csr_p_enable0_rdata[25] = csr_p_enable0_enable_irq25_ff;

assign csr_p_enable0_enable_irq25_out = csr_p_enable0_enable_irq25_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq25_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq25_ff <= wdata[25];
            end
        end else begin
            csr_p_enable0_enable_irq25_ff <= csr_p_enable0_enable_irq25_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[26] - ENABLE_IRQ26 - Enable mask for IRQ26
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq26_ff;

assign csr_p_enable0_rdata[26] = csr_p_enable0_enable_irq26_ff;

assign csr_p_enable0_enable_irq26_out = csr_p_enable0_enable_irq26_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq26_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq26_ff <= wdata[26];
            end
        end else begin
            csr_p_enable0_enable_irq26_ff <= csr_p_enable0_enable_irq26_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[27] - ENABLE_IRQ27 - Enable mask for IRQ27
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq27_ff;

assign csr_p_enable0_rdata[27] = csr_p_enable0_enable_irq27_ff;

assign csr_p_enable0_enable_irq27_out = csr_p_enable0_enable_irq27_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq27_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq27_ff <= wdata[27];
            end
        end else begin
            csr_p_enable0_enable_irq27_ff <= csr_p_enable0_enable_irq27_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[28] - ENABLE_IRQ28 - Enable mask for IRQ28
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq28_ff;

assign csr_p_enable0_rdata[28] = csr_p_enable0_enable_irq28_ff;

assign csr_p_enable0_enable_irq28_out = csr_p_enable0_enable_irq28_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq28_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq28_ff <= wdata[28];
            end
        end else begin
            csr_p_enable0_enable_irq28_ff <= csr_p_enable0_enable_irq28_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[29] - ENABLE_IRQ29 - Enable mask for IRQ29
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq29_ff;

assign csr_p_enable0_rdata[29] = csr_p_enable0_enable_irq29_ff;

assign csr_p_enable0_enable_irq29_out = csr_p_enable0_enable_irq29_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq29_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq29_ff <= wdata[29];
            end
        end else begin
            csr_p_enable0_enable_irq29_ff <= csr_p_enable0_enable_irq29_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[30] - ENABLE_IRQ30 - Enable mask for IRQ30
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq30_ff;

assign csr_p_enable0_rdata[30] = csr_p_enable0_enable_irq30_ff;

assign csr_p_enable0_enable_irq30_out = csr_p_enable0_enable_irq30_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq30_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq30_ff <= wdata[30];
            end
        end else begin
            csr_p_enable0_enable_irq30_ff <= csr_p_enable0_enable_irq30_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE0[31] - ENABLE_IRQ31 - Enable mask for IRQ31
// access: rw, hardware: o
//---------------------
reg  csr_p_enable0_enable_irq31_ff;

assign csr_p_enable0_rdata[31] = csr_p_enable0_enable_irq31_ff;

assign csr_p_enable0_enable_irq31_out = csr_p_enable0_enable_irq31_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable0_enable_irq31_ff <= 1'b0;
    end else  begin
     if (csr_p_enable0_wen) begin
            if (wstrb[3]) begin
                csr_p_enable0_enable_irq31_ff <= wdata[31];
            end
        end else begin
            csr_p_enable0_enable_irq31_ff <= csr_p_enable0_enable_irq31_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x2004] - P_ENABLE1 - Enable bits for IRQ32–IRQ62
//------------------------------------------------------------------------------
wire [31:0] csr_p_enable1_rdata;
assign csr_p_enable1_rdata[0] = 1'b0;

wire csr_p_enable1_wen;
assign csr_p_enable1_wen = wen && (waddr == 32'h2004);

wire csr_p_enable1_ren;
assign csr_p_enable1_ren = ren && (raddr == 32'h2004);
reg csr_p_enable1_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_ren_ff <= 1'b0;
    end else begin
        csr_p_enable1_ren_ff <= csr_p_enable1_ren;
    end
end
//---------------------
// Bit field:
// P_ENABLE1[1] - ENABLE_IRQ32 - Enable mask for IRQ32
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq32_ff;

assign csr_p_enable1_rdata[1] = csr_p_enable1_enable_irq32_ff;

assign csr_p_enable1_enable_irq32_out = csr_p_enable1_enable_irq32_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq32_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq32_ff <= wdata[1];
            end
        end else begin
            csr_p_enable1_enable_irq32_ff <= csr_p_enable1_enable_irq32_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[2] - ENABLE_IRQ33 - Enable mask for IRQ33
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq33_ff;

assign csr_p_enable1_rdata[2] = csr_p_enable1_enable_irq33_ff;

assign csr_p_enable1_enable_irq33_out = csr_p_enable1_enable_irq33_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq33_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq33_ff <= wdata[2];
            end
        end else begin
            csr_p_enable1_enable_irq33_ff <= csr_p_enable1_enable_irq33_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[3] - ENABLE_IRQ34 - Enable mask for IRQ34
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq34_ff;

assign csr_p_enable1_rdata[3] = csr_p_enable1_enable_irq34_ff;

assign csr_p_enable1_enable_irq34_out = csr_p_enable1_enable_irq34_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq34_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq34_ff <= wdata[3];
            end
        end else begin
            csr_p_enable1_enable_irq34_ff <= csr_p_enable1_enable_irq34_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[4] - ENABLE_IRQ35 - Enable mask for IRQ35
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq35_ff;

assign csr_p_enable1_rdata[4] = csr_p_enable1_enable_irq35_ff;

assign csr_p_enable1_enable_irq35_out = csr_p_enable1_enable_irq35_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq35_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq35_ff <= wdata[4];
            end
        end else begin
            csr_p_enable1_enable_irq35_ff <= csr_p_enable1_enable_irq35_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[5] - ENABLE_IRQ36 - Enable mask for IRQ36
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq36_ff;

assign csr_p_enable1_rdata[5] = csr_p_enable1_enable_irq36_ff;

assign csr_p_enable1_enable_irq36_out = csr_p_enable1_enable_irq36_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq36_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq36_ff <= wdata[5];
            end
        end else begin
            csr_p_enable1_enable_irq36_ff <= csr_p_enable1_enable_irq36_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[6] - ENABLE_IRQ37 - Enable mask for IRQ37
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq37_ff;

assign csr_p_enable1_rdata[6] = csr_p_enable1_enable_irq37_ff;

assign csr_p_enable1_enable_irq37_out = csr_p_enable1_enable_irq37_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq37_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq37_ff <= wdata[6];
            end
        end else begin
            csr_p_enable1_enable_irq37_ff <= csr_p_enable1_enable_irq37_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[7] - ENABLE_IRQ38 - Enable mask for IRQ38
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq38_ff;

assign csr_p_enable1_rdata[7] = csr_p_enable1_enable_irq38_ff;

assign csr_p_enable1_enable_irq38_out = csr_p_enable1_enable_irq38_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq38_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[0]) begin
                csr_p_enable1_enable_irq38_ff <= wdata[7];
            end
        end else begin
            csr_p_enable1_enable_irq38_ff <= csr_p_enable1_enable_irq38_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[8] - ENABLE_IRQ39 - Enable mask for IRQ39
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq39_ff;

assign csr_p_enable1_rdata[8] = csr_p_enable1_enable_irq39_ff;

assign csr_p_enable1_enable_irq39_out = csr_p_enable1_enable_irq39_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq39_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq39_ff <= wdata[8];
            end
        end else begin
            csr_p_enable1_enable_irq39_ff <= csr_p_enable1_enable_irq39_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[9] - ENABLE_IRQ40 - Enable mask for IRQ40
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq40_ff;

assign csr_p_enable1_rdata[9] = csr_p_enable1_enable_irq40_ff;

assign csr_p_enable1_enable_irq40_out = csr_p_enable1_enable_irq40_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq40_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq40_ff <= wdata[9];
            end
        end else begin
            csr_p_enable1_enable_irq40_ff <= csr_p_enable1_enable_irq40_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[10] - ENABLE_IRQ41 - Enable mask for IRQ41
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq41_ff;

assign csr_p_enable1_rdata[10] = csr_p_enable1_enable_irq41_ff;

assign csr_p_enable1_enable_irq41_out = csr_p_enable1_enable_irq41_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq41_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq41_ff <= wdata[10];
            end
        end else begin
            csr_p_enable1_enable_irq41_ff <= csr_p_enable1_enable_irq41_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[11] - ENABLE_IRQ42 - Enable mask for IRQ42
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq42_ff;

assign csr_p_enable1_rdata[11] = csr_p_enable1_enable_irq42_ff;

assign csr_p_enable1_enable_irq42_out = csr_p_enable1_enable_irq42_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq42_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq42_ff <= wdata[11];
            end
        end else begin
            csr_p_enable1_enable_irq42_ff <= csr_p_enable1_enable_irq42_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[12] - ENABLE_IRQ43 - Enable mask for IRQ43
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq43_ff;

assign csr_p_enable1_rdata[12] = csr_p_enable1_enable_irq43_ff;

assign csr_p_enable1_enable_irq43_out = csr_p_enable1_enable_irq43_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq43_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq43_ff <= wdata[12];
            end
        end else begin
            csr_p_enable1_enable_irq43_ff <= csr_p_enable1_enable_irq43_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[13] - ENABLE_IRQ44 - Enable mask for IRQ44
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq44_ff;

assign csr_p_enable1_rdata[13] = csr_p_enable1_enable_irq44_ff;

assign csr_p_enable1_enable_irq44_out = csr_p_enable1_enable_irq44_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq44_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq44_ff <= wdata[13];
            end
        end else begin
            csr_p_enable1_enable_irq44_ff <= csr_p_enable1_enable_irq44_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[14] - ENABLE_IRQ45 - Enable mask for IRQ45
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq45_ff;

assign csr_p_enable1_rdata[14] = csr_p_enable1_enable_irq45_ff;

assign csr_p_enable1_enable_irq45_out = csr_p_enable1_enable_irq45_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq45_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq45_ff <= wdata[14];
            end
        end else begin
            csr_p_enable1_enable_irq45_ff <= csr_p_enable1_enable_irq45_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[15] - ENABLE_IRQ46 - Enable mask for IRQ46
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq46_ff;

assign csr_p_enable1_rdata[15] = csr_p_enable1_enable_irq46_ff;

assign csr_p_enable1_enable_irq46_out = csr_p_enable1_enable_irq46_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq46_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[1]) begin
                csr_p_enable1_enable_irq46_ff <= wdata[15];
            end
        end else begin
            csr_p_enable1_enable_irq46_ff <= csr_p_enable1_enable_irq46_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[16] - ENABLE_IRQ47 - Enable mask for IRQ47
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq47_ff;

assign csr_p_enable1_rdata[16] = csr_p_enable1_enable_irq47_ff;

assign csr_p_enable1_enable_irq47_out = csr_p_enable1_enable_irq47_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq47_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq47_ff <= wdata[16];
            end
        end else begin
            csr_p_enable1_enable_irq47_ff <= csr_p_enable1_enable_irq47_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[17] - ENABLE_IRQ48 - Enable mask for IRQ48
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq48_ff;

assign csr_p_enable1_rdata[17] = csr_p_enable1_enable_irq48_ff;

assign csr_p_enable1_enable_irq48_out = csr_p_enable1_enable_irq48_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq48_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq48_ff <= wdata[17];
            end
        end else begin
            csr_p_enable1_enable_irq48_ff <= csr_p_enable1_enable_irq48_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[18] - ENABLE_IRQ49 - Enable mask for IRQ49
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq49_ff;

assign csr_p_enable1_rdata[18] = csr_p_enable1_enable_irq49_ff;

assign csr_p_enable1_enable_irq49_out = csr_p_enable1_enable_irq49_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq49_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq49_ff <= wdata[18];
            end
        end else begin
            csr_p_enable1_enable_irq49_ff <= csr_p_enable1_enable_irq49_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[19] - ENABLE_IRQ50 - Enable mask for IRQ50
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq50_ff;

assign csr_p_enable1_rdata[19] = csr_p_enable1_enable_irq50_ff;

assign csr_p_enable1_enable_irq50_out = csr_p_enable1_enable_irq50_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq50_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq50_ff <= wdata[19];
            end
        end else begin
            csr_p_enable1_enable_irq50_ff <= csr_p_enable1_enable_irq50_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[20] - ENABLE_IRQ51 - Enable mask for IRQ51
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq51_ff;

assign csr_p_enable1_rdata[20] = csr_p_enable1_enable_irq51_ff;

assign csr_p_enable1_enable_irq51_out = csr_p_enable1_enable_irq51_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq51_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq51_ff <= wdata[20];
            end
        end else begin
            csr_p_enable1_enable_irq51_ff <= csr_p_enable1_enable_irq51_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[21] - ENABLE_IRQ52 - Enable mask for IRQ52
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq52_ff;

assign csr_p_enable1_rdata[21] = csr_p_enable1_enable_irq52_ff;

assign csr_p_enable1_enable_irq52_out = csr_p_enable1_enable_irq52_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq52_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq52_ff <= wdata[21];
            end
        end else begin
            csr_p_enable1_enable_irq52_ff <= csr_p_enable1_enable_irq52_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[22] - ENABLE_IRQ53 - Enable mask for IRQ53
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq53_ff;

assign csr_p_enable1_rdata[22] = csr_p_enable1_enable_irq53_ff;

assign csr_p_enable1_enable_irq53_out = csr_p_enable1_enable_irq53_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq53_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq53_ff <= wdata[22];
            end
        end else begin
            csr_p_enable1_enable_irq53_ff <= csr_p_enable1_enable_irq53_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[23] - ENABLE_IRQ54 - Enable mask for IRQ54
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq54_ff;

assign csr_p_enable1_rdata[23] = csr_p_enable1_enable_irq54_ff;

assign csr_p_enable1_enable_irq54_out = csr_p_enable1_enable_irq54_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq54_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[2]) begin
                csr_p_enable1_enable_irq54_ff <= wdata[23];
            end
        end else begin
            csr_p_enable1_enable_irq54_ff <= csr_p_enable1_enable_irq54_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[24] - ENABLE_IRQ55 - Enable mask for IRQ55
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq55_ff;

assign csr_p_enable1_rdata[24] = csr_p_enable1_enable_irq55_ff;

assign csr_p_enable1_enable_irq55_out = csr_p_enable1_enable_irq55_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq55_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq55_ff <= wdata[24];
            end
        end else begin
            csr_p_enable1_enable_irq55_ff <= csr_p_enable1_enable_irq55_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[25] - ENABLE_IRQ56 - Enable mask for IRQ56
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq56_ff;

assign csr_p_enable1_rdata[25] = csr_p_enable1_enable_irq56_ff;

assign csr_p_enable1_enable_irq56_out = csr_p_enable1_enable_irq56_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq56_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq56_ff <= wdata[25];
            end
        end else begin
            csr_p_enable1_enable_irq56_ff <= csr_p_enable1_enable_irq56_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[26] - ENABLE_IRQ57 - Enable mask for IRQ57
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq57_ff;

assign csr_p_enable1_rdata[26] = csr_p_enable1_enable_irq57_ff;

assign csr_p_enable1_enable_irq57_out = csr_p_enable1_enable_irq57_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq57_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq57_ff <= wdata[26];
            end
        end else begin
            csr_p_enable1_enable_irq57_ff <= csr_p_enable1_enable_irq57_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[27] - ENABLE_IRQ58 - Enable mask for IRQ58
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq58_ff;

assign csr_p_enable1_rdata[27] = csr_p_enable1_enable_irq58_ff;

assign csr_p_enable1_enable_irq58_out = csr_p_enable1_enable_irq58_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq58_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq58_ff <= wdata[27];
            end
        end else begin
            csr_p_enable1_enable_irq58_ff <= csr_p_enable1_enable_irq58_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[28] - ENABLE_IRQ59 - Enable mask for IRQ59
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq59_ff;

assign csr_p_enable1_rdata[28] = csr_p_enable1_enable_irq59_ff;

assign csr_p_enable1_enable_irq59_out = csr_p_enable1_enable_irq59_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq59_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq59_ff <= wdata[28];
            end
        end else begin
            csr_p_enable1_enable_irq59_ff <= csr_p_enable1_enable_irq59_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[29] - ENABLE_IRQ60 - Enable mask for IRQ60
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq60_ff;

assign csr_p_enable1_rdata[29] = csr_p_enable1_enable_irq60_ff;

assign csr_p_enable1_enable_irq60_out = csr_p_enable1_enable_irq60_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq60_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq60_ff <= wdata[29];
            end
        end else begin
            csr_p_enable1_enable_irq60_ff <= csr_p_enable1_enable_irq60_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[30] - ENABLE_IRQ61 - Enable mask for IRQ61
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq61_ff;

assign csr_p_enable1_rdata[30] = csr_p_enable1_enable_irq61_ff;

assign csr_p_enable1_enable_irq61_out = csr_p_enable1_enable_irq61_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq61_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq61_ff <= wdata[30];
            end
        end else begin
            csr_p_enable1_enable_irq61_ff <= csr_p_enable1_enable_irq61_ff;
        end
    end
end


//---------------------
// Bit field:
// P_ENABLE1[31] - ENABLE_IRQ62 - Enable mask for IRQ62
// access: rw, hardware: o
//---------------------
reg  csr_p_enable1_enable_irq62_ff;

assign csr_p_enable1_rdata[31] = csr_p_enable1_enable_irq62_ff;

assign csr_p_enable1_enable_irq62_out = csr_p_enable1_enable_irq62_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_enable1_enable_irq62_ff <= 1'b0;
    end else  begin
     if (csr_p_enable1_wen) begin
            if (wstrb[3]) begin
                csr_p_enable1_enable_irq62_ff <= wdata[31];
            end
        end else begin
            csr_p_enable1_enable_irq62_ff <= csr_p_enable1_enable_irq62_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x200000] - P_THRESHOLD - Priority threshold register
//------------------------------------------------------------------------------
wire [31:0] csr_p_threshold_rdata;
assign csr_p_threshold_rdata[31:3] = 29'h0;

wire csr_p_threshold_wen;
assign csr_p_threshold_wen = wen && (waddr == 32'h200000);

wire csr_p_threshold_ren;
assign csr_p_threshold_ren = ren && (raddr == 32'h200000);
reg csr_p_threshold_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_threshold_ren_ff <= 1'b0;
    end else begin
        csr_p_threshold_ren_ff <= csr_p_threshold_ren;
    end
end
//---------------------
// Bit field:
// P_THRESHOLD[2:0] - THRESHOLD - Only IRQs with priority > threshold will be signaled
// access: rw, hardware: o
//---------------------
reg [2:0] csr_p_threshold_threshold_ff;

assign csr_p_threshold_rdata[2:0] = csr_p_threshold_threshold_ff;

assign csr_p_threshold_threshold_out = csr_p_threshold_threshold_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_threshold_threshold_ff <= 3'h0;
    end else  begin
     if (csr_p_threshold_wen) begin
            if (wstrb[0]) begin
                csr_p_threshold_threshold_ff[2:0] <= wdata[2:0];
            end
        end else begin
            csr_p_threshold_threshold_ff <= csr_p_threshold_threshold_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x200004] - P_CLAIMCOMPLETE - Claim/Complete register
//------------------------------------------------------------------------------
wire [31:0] csr_p_claimcomplete_rdata;
assign csr_p_claimcomplete_rdata[31:6] = 26'h0;

wire csr_p_claimcomplete_wen;
assign csr_p_claimcomplete_wen = wen && (waddr == 32'h200004);

wire csr_p_claimcomplete_ren;
assign csr_p_claimcomplete_ren = ren && (raddr == 32'h200004);
reg csr_p_claimcomplete_ren_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_claimcomplete_ren_ff <= 1'b0;
    end else begin
        csr_p_claimcomplete_ren_ff <= csr_p_claimcomplete_ren;
    end
end
//---------------------
// Bit field:
// P_CLAIMCOMPLETE[5:0] - CP - Read = claim IRQ ID, Write = complete IRQ ID
// access: rw, hardware: io
//---------------------
reg [5:0] csr_p_claimcomplete_cp_ff;

assign csr_p_claimcomplete_rdata[5:0] = csr_p_claimcomplete_cp_ff;

assign csr_p_claimcomplete_cp_out = csr_p_claimcomplete_cp_ff;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        csr_p_claimcomplete_cp_ff <= 6'h0;
        complete_flag <= 1'b0;
        complete_id <= 6'd0;
    end else  begin
     complete_flag <= 1'b0;
     if (csr_p_claimcomplete_wen) begin
            if (wstrb[0]) begin
                complete_id <= wdata[5:0];
                //csr_p_claimcomplete_cp_ff[5:0] <= wdata[5:0];
                complete_flag <= 1'b1;
            end
        end else         begin            csr_p_claimcomplete_cp_ff <= csr_p_claimcomplete_cp_in;
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
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdata_ff <= 32'h0;
        claim_flag <= 1'b0;
    end else if (ren) begin
        claim_flag <= 1'b0;
        case (raddr)
            32'h4: rdata_ff <= csr_p_priority1_rdata;
            32'h8: rdata_ff <= csr_p_priority2_rdata;
            32'hc: rdata_ff <= csr_p_priority3_rdata;
            32'h10: rdata_ff <= csr_p_priority4_rdata;
            32'h14: rdata_ff <= csr_p_priority5_rdata;
            32'h18: rdata_ff <= csr_p_priority6_rdata;
            32'h1c: rdata_ff <= csr_p_priority7_rdata;
            32'h20: rdata_ff <= csr_p_priority8_rdata;
            32'h24: rdata_ff <= csr_p_priority9_rdata;
            32'h28: rdata_ff <= csr_p_priority10_rdata;
            32'h2c: rdata_ff <= csr_p_priority11_rdata;
            32'h30: rdata_ff <= csr_p_priority12_rdata;
            32'h34: rdata_ff <= csr_p_priority13_rdata;
            32'h38: rdata_ff <= csr_p_priority14_rdata;
            32'h3c: rdata_ff <= csr_p_priority15_rdata;
            32'h40: rdata_ff <= csr_p_priority16_rdata;
            32'h44: rdata_ff <= csr_p_priority17_rdata;
            32'h48: rdata_ff <= csr_p_priority18_rdata;
            32'h4c: rdata_ff <= csr_p_priority19_rdata;
            32'h50: rdata_ff <= csr_p_priority20_rdata;
            32'h54: rdata_ff <= csr_p_priority21_rdata;
            32'h58: rdata_ff <= csr_p_priority22_rdata;
            32'h5c: rdata_ff <= csr_p_priority23_rdata;
            32'h60: rdata_ff <= csr_p_priority24_rdata;
            32'h64: rdata_ff <= csr_p_priority25_rdata;
            32'h68: rdata_ff <= csr_p_priority26_rdata;
            32'h6c: rdata_ff <= csr_p_priority27_rdata;
            32'h70: rdata_ff <= csr_p_priority28_rdata;
            32'h74: rdata_ff <= csr_p_priority29_rdata;
            32'h78: rdata_ff <= csr_p_priority30_rdata;
            32'h7c: rdata_ff <= csr_p_priority31_rdata;
            32'h80: rdata_ff <= csr_p_priority32_rdata;
            32'h84: rdata_ff <= csr_p_priority33_rdata;
            32'h88: rdata_ff <= csr_p_priority34_rdata;
            32'h8c: rdata_ff <= csr_p_priority35_rdata;
            32'h90: rdata_ff <= csr_p_priority36_rdata;
            32'h94: rdata_ff <= csr_p_priority37_rdata;
            32'h98: rdata_ff <= csr_p_priority38_rdata;
            32'h9c: rdata_ff <= csr_p_priority39_rdata;
            32'ha0: rdata_ff <= csr_p_priority40_rdata;
            32'ha4: rdata_ff <= csr_p_priority41_rdata;
            32'ha8: rdata_ff <= csr_p_priority42_rdata;
            32'hac: rdata_ff <= csr_p_priority43_rdata;
            32'hb0: rdata_ff <= csr_p_priority44_rdata;
            32'hb4: rdata_ff <= csr_p_priority45_rdata;
            32'hb8: rdata_ff <= csr_p_priority46_rdata;
            32'hbc: rdata_ff <= csr_p_priority47_rdata;
            32'hc0: rdata_ff <= csr_p_priority48_rdata;
            32'hc4: rdata_ff <= csr_p_priority49_rdata;
            32'hc8: rdata_ff <= csr_p_priority50_rdata;
            32'hcc: rdata_ff <= csr_p_priority51_rdata;
            32'hd0: rdata_ff <= csr_p_priority52_rdata;
            32'hd4: rdata_ff <= csr_p_priority53_rdata;
            32'hd8: rdata_ff <= csr_p_priority54_rdata;
            32'hdc: rdata_ff <= csr_p_priority55_rdata;
            32'he0: rdata_ff <= csr_p_priority56_rdata;
            32'he4: rdata_ff <= csr_p_priority57_rdata;
            32'he8: rdata_ff <= csr_p_priority58_rdata;
            32'hec: rdata_ff <= csr_p_priority59_rdata;
            32'hf0: rdata_ff <= csr_p_priority60_rdata;
            32'hf4: rdata_ff <= csr_p_priority61_rdata;
            32'hf8: rdata_ff <= csr_p_priority62_rdata;
            32'hfc: rdata_ff <= csr_p_priority63_rdata;
            32'h100: rdata_ff <= csr_p_priority64_rdata;
            32'h1000: rdata_ff <= csr_p_pending0_rdata;
            32'h1004: rdata_ff <= csr_p_pending1_rdata;
            32'h2000: rdata_ff <= csr_p_enable0_rdata;
            32'h2004: rdata_ff <= csr_p_enable1_rdata;
            32'h200000: rdata_ff <= csr_p_threshold_rdata;
            32'h200004: begin
                rdata_ff <= 1;
                claim_flag <= 1'b1;
            end
            default: rdata_ff <= 32'h0;
        endcase
    end else begin
        rdata_ff <= 32'h0;
        claim_flag <= 1'b0;
    end
end
assign rdata = rdata_ff;

//------------------------------------------------------------------------------
// Read data valid
//------------------------------------------------------------------------------
reg rvalid_ff;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rvalid_ff <= 1'b0;
    end else if (ren && rvalid) begin
        rvalid_ff <= 1'b0;
    end else if (ren) begin
        rvalid_ff <= 1'b1;
    end
end

assign rvalid = rvalid_ff;

endmodule