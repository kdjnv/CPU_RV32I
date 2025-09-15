/*
Document: riscv-plic-1.0.0 - RISC-V Platform-Level Interrupt Controller Specification
                                Version 1.0.0, 3/2023


Priority Registers (ưu tiên từng IRQ): Hỗ trợ 64 ngắt, 8 mức ưu tiên
    base + 0x0000_0004: Priority cho IRQ1
    base + 0x0000_0008: Priority cho IRQ2
    ....
    base + 0x0000_0100: Priority cho IRQ64

Pending Registers (IRQ nào đang pending-chờ)
    base + 0x0000_1000: Pending [31:0 ] - IRQ1->IRQ32
    base + 0x0000_1004: Pending [61:32] - IRQ33->IRQ64

Enable Registers (IRQ nào được enable)
    base + 0x0000_2000: Enable [31:0 ]  - IRQ1->IRQ32
    base + 0x0000_2004: Enable [61:32]  - IRQ33->IRQ64

Threshold Register (ngưỡng ưu tiên): Nếu threshold = N, thì chỉ những IRQ có priority > N mới được gửi tới CPU.
    base + 0x0020_0000: Priority threshold 

Claim/Complete Register(IRQ cần thực hiện nếu là Claim, IRQ nào đã hoàn thành nếu là Complete)
    base + 0x0020_0004: Claim/complete
*/
module PLIC_ip(
    // System
    input           PCLK,
    input           PRESETn,

    // External interrupt sources
    input           irqs1_rxuart,       // Ngoại vi UART báo nhận đc ký hiệu
    input           irqs2_txuart,       // Ngoại vi UART báo hoàn tất truyền
    input           irqs3_pgpio0,       // GPIO 0 positive edge
    input           irqs4_ngpio0,       // GPIO 0 negative edge
    input           irqs5_pgpio1,       // GPIO 1 positive edge
    input           irqs6_ngpio1,       // GPIO 1 negative edge
    input           irqs7_pgpio2,       // GPIO 2 positive edge
    input           irqs8_ngpio2,       // GPIO 2 negative edge
    input           irqs9_pgpio3,       // GPIO 3 positive edge
    input           irqs10_ngpio3,      // GPIO 3 negative edge
    input           irqs11_pgpio4,      // GPIO 4 positive edge
    input           irqs12_ngpio4,      // GPIO 4 negative edge
    input           irqs13_pgpio5,      // GPIO 5 positive edge
    input           irqs14_ngpio5,      // GPIO 5 negative edge
    input           irqs15_pgpio6,      // GPIO 6 positive edge
    input           irqs16_ngpio6,      // GPIO 6 negative edge
    input           irqs17_pgpio7,      // GPIO 7 positive edge
    input           irqs18_ngpio7,      // GPIO 7 negative edge
    input           irqs19_pgpio8,      // GPIO 8 positive edge
    input           irqs20_ngpio8,      // GPIO 8 negative edge
    input           irqs21_pgpio9,      // GPIO 9 positive edge
    input           irqs22_ngpio9,      // GPIO 9 negative edge
    input           irqs23_pgpio10,     // GPIO 10 positive edge
    input           irqs24_ngpio10,     // GPIO 10 negative edge
    input           irqs25_pgpio11,     // GPIO 11 positive edge
    input           irqs26_ngpio11,     // GPIO 11 negative edge
    input           irqs27_pgpio12,     // GPIO 12 positive edge
    input           irqs28_ngpio12,     // GPIO 12 negative edge
    input           irqs29_pgpio13,     // GPIO 13 positive edge
    input           irqs30_ngpio13,     // GPIO 13 negative edge
    input           irqs31_pgpio14,     // GPIO 14 positive edge
    input           irqs32_ngpio14,     // GPIO 14 negative edge
    input           irqs33_pgpio15,     // GPIO 15 positive edge
    input           irqs34_ngpio15,     // GPIO 15 negative edge

    // To CPU
    output reg      irq_out,
    output          irq_external_pending,
    input           trap_en,
//    output [5:0]    claim_id,


    // APB
    input   [31:0]  PADDR,
    input           PWRITE,
    input   [31:0]  PWDATA,
    input   [ 3:0]  PSTRB,
    input           PSEL,
    input           PENABLE,
    output  [31:0]  PRDATA,
    output          PREADY,
    output          PSLVERR
);

    // --- wires between regs_PLIC and PLIC_IP ---

    // Priorities
    wire [2:0] csr_priority [0:64];

    // Enable
    wire [31:0] enable_irq_out0;
    wire [31:0] enable_irq_out1;

    // Threshold
    wire [2:0] csr_p_threshold_threshold_out;

    // Pending (driven by IP -> regs)
    reg  [31:0] pending_irq_in0 = 32'd0;
    reg  [31:0] pending_irq_in1 = 32'd0;

    // Active
    reg  [63:0] active_irq = 64'd0;

    // Claim/complete
    reg  [5:0] csr_p_claimcomplete_cp_in;
    wire [5:0] csr_p_claimcomplete_cp_out;

    wire       complete_flag;
    wire [5:0] complete_id;
    wire       claim_flag;

    // --- Instantiate register block ---
    regs_PLIC u_regs (
        .clk(PCLK),
        .rst_n(PRESETn),

        // Priority outputs (all 64)
        .csr_p_priority1_priority1_out (csr_priority[1]),
        .csr_p_priority2_priority2_out (csr_priority[2]),
        .csr_p_priority3_priority3_out (csr_priority[3]),
        .csr_p_priority4_priority4_out (csr_priority[4]),
        .csr_p_priority5_priority5_out (csr_priority[5]),
        .csr_p_priority6_priority6_out (csr_priority[6]),
        .csr_p_priority7_priority7_out (csr_priority[7]),
        .csr_p_priority8_priority8_out (csr_priority[8]),
        .csr_p_priority9_priority9_out (csr_priority[9]),
        .csr_p_priority10_priority10_out(csr_priority[10]),
        .csr_p_priority11_priority11_out(csr_priority[11]),
        .csr_p_priority12_priority12_out(csr_priority[12]),
        .csr_p_priority13_priority13_out(csr_priority[13]),
        .csr_p_priority14_priority14_out(csr_priority[14]),
        .csr_p_priority15_priority15_out(csr_priority[15]),
        .csr_p_priority16_priority16_out(csr_priority[16]),
        .csr_p_priority17_priority17_out(csr_priority[17]),
        .csr_p_priority18_priority18_out(csr_priority[18]),
        .csr_p_priority19_priority19_out(csr_priority[19]),
        .csr_p_priority20_priority20_out(csr_priority[20]),
        .csr_p_priority21_priority21_out(csr_priority[21]),
        .csr_p_priority22_priority22_out(csr_priority[22]),
        .csr_p_priority23_priority23_out(csr_priority[23]),
        .csr_p_priority24_priority24_out(csr_priority[24]),
        .csr_p_priority25_priority25_out(csr_priority[25]),
        .csr_p_priority26_priority26_out(csr_priority[26]),
        .csr_p_priority27_priority27_out(csr_priority[27]),
        .csr_p_priority28_priority28_out(csr_priority[28]),
        .csr_p_priority29_priority29_out(csr_priority[29]),
        .csr_p_priority30_priority30_out(csr_priority[30]),
        .csr_p_priority31_priority31_out(csr_priority[31]),
        .csr_p_priority32_priority32_out(csr_priority[32]),
        .csr_p_priority33_priority33_out(csr_priority[33]),
        .csr_p_priority34_priority34_out(csr_priority[34]),
        .csr_p_priority35_priority35_out(csr_priority[35]),
        .csr_p_priority36_priority36_out(csr_priority[36]),
        .csr_p_priority37_priority37_out(csr_priority[37]),
        .csr_p_priority38_priority38_out(csr_priority[38]),
        .csr_p_priority39_priority39_out(csr_priority[39]),
        .csr_p_priority40_priority40_out(csr_priority[40]),
        .csr_p_priority41_priority41_out(csr_priority[41]),
        .csr_p_priority42_priority42_out(csr_priority[42]),
        .csr_p_priority43_priority43_out(csr_priority[43]),
        .csr_p_priority44_priority44_out(csr_priority[44]),
        .csr_p_priority45_priority45_out(csr_priority[45]),
        .csr_p_priority46_priority46_out(csr_priority[46]),
        .csr_p_priority47_priority47_out(csr_priority[47]),
        .csr_p_priority48_priority48_out(csr_priority[48]),
        .csr_p_priority49_priority49_out(csr_priority[49]),
        .csr_p_priority50_priority50_out(csr_priority[50]),
        .csr_p_priority51_priority51_out(csr_priority[51]),
        .csr_p_priority52_priority52_out(csr_priority[52]),
        .csr_p_priority53_priority53_out(csr_priority[53]),
        .csr_p_priority54_priority54_out(csr_priority[54]),
        .csr_p_priority55_priority55_out(csr_priority[55]),
        .csr_p_priority56_priority56_out(csr_priority[56]),
        .csr_p_priority57_priority57_out(csr_priority[57]),
        .csr_p_priority58_priority58_out(csr_priority[58]),
        .csr_p_priority59_priority59_out(csr_priority[59]),
        .csr_p_priority60_priority60_out(csr_priority[60]),
        .csr_p_priority61_priority61_out(csr_priority[61]),
        .csr_p_priority62_priority62_out(csr_priority[62]),
        .csr_p_priority63_priority63_out(csr_priority[63]),
        .csr_p_priority64_priority64_out(csr_priority[64]),




        // PENDING inputs
        .csr_p_pending0_pending_irq1_in  (pending_irq_in0[1]),
        .csr_p_pending0_pending_irq2_in  (pending_irq_in0[2]),
        .csr_p_pending0_pending_irq3_in  (pending_irq_in0[3]),
        .csr_p_pending0_pending_irq4_in  (pending_irq_in0[4]),
        .csr_p_pending0_pending_irq5_in  (pending_irq_in0[5]),
        .csr_p_pending0_pending_irq6_in  (pending_irq_in0[6]),
        .csr_p_pending0_pending_irq7_in  (pending_irq_in0[7]),
        .csr_p_pending0_pending_irq8_in  (pending_irq_in0[8]),
        .csr_p_pending0_pending_irq9_in  (pending_irq_in0[9]),
        .csr_p_pending0_pending_irq10_in (pending_irq_in0[10]),
        .csr_p_pending0_pending_irq11_in (pending_irq_in0[11]),
        .csr_p_pending0_pending_irq12_in (pending_irq_in0[12]),
        .csr_p_pending0_pending_irq13_in (pending_irq_in0[13]),
        .csr_p_pending0_pending_irq14_in (pending_irq_in0[14]),
        .csr_p_pending0_pending_irq15_in (pending_irq_in0[15]),
        .csr_p_pending0_pending_irq16_in (pending_irq_in0[16]),
        .csr_p_pending0_pending_irq17_in (pending_irq_in0[17]),
        .csr_p_pending0_pending_irq18_in (pending_irq_in0[18]),
        .csr_p_pending0_pending_irq19_in (pending_irq_in0[19]),
        .csr_p_pending0_pending_irq20_in (pending_irq_in0[20]),
        .csr_p_pending0_pending_irq21_in (pending_irq_in0[21]),
        .csr_p_pending0_pending_irq22_in (pending_irq_in0[22]),
        .csr_p_pending0_pending_irq23_in (pending_irq_in0[23]),
        .csr_p_pending0_pending_irq24_in (pending_irq_in0[24]),
        .csr_p_pending0_pending_irq25_in (pending_irq_in0[25]),
        .csr_p_pending0_pending_irq26_in (pending_irq_in0[26]),
        .csr_p_pending0_pending_irq27_in (pending_irq_in0[27]),
        .csr_p_pending0_pending_irq28_in (pending_irq_in0[28]),
        .csr_p_pending0_pending_irq29_in (pending_irq_in0[29]),
        .csr_p_pending0_pending_irq30_in (pending_irq_in0[30]),
        .csr_p_pending0_pending_irq31_in (pending_irq_in0[31]),

        .csr_p_pending1_pending_irq32_in (pending_irq_in1[1]),
        .csr_p_pending1_pending_irq33_in (pending_irq_in1[2]),
        .csr_p_pending1_pending_irq34_in (pending_irq_in1[3]),
        .csr_p_pending1_pending_irq35_in (pending_irq_in1[4]),
        .csr_p_pending1_pending_irq36_in (pending_irq_in1[5]),
        .csr_p_pending1_pending_irq37_in (pending_irq_in1[6]),
        .csr_p_pending1_pending_irq38_in (pending_irq_in1[7]),
        .csr_p_pending1_pending_irq39_in (pending_irq_in1[8]),
        .csr_p_pending1_pending_irq40_in (pending_irq_in1[9]),
        .csr_p_pending1_pending_irq41_in (pending_irq_in1[10]),
        .csr_p_pending1_pending_irq42_in (pending_irq_in1[11]),
        .csr_p_pending1_pending_irq43_in (pending_irq_in1[12]),
        .csr_p_pending1_pending_irq44_in (pending_irq_in1[13]),
        .csr_p_pending1_pending_irq45_in (pending_irq_in1[14]),
        .csr_p_pending1_pending_irq46_in (pending_irq_in1[15]),
        .csr_p_pending1_pending_irq47_in (pending_irq_in1[16]),
        .csr_p_pending1_pending_irq48_in (pending_irq_in1[17]),
        .csr_p_pending1_pending_irq49_in (pending_irq_in1[18]),
        .csr_p_pending1_pending_irq50_in (pending_irq_in1[19]),
        .csr_p_pending1_pending_irq51_in (pending_irq_in1[20]),
        .csr_p_pending1_pending_irq52_in (pending_irq_in1[21]),
        .csr_p_pending1_pending_irq53_in (pending_irq_in1[22]),
        .csr_p_pending1_pending_irq54_in (pending_irq_in1[23]),
        .csr_p_pending1_pending_irq55_in (pending_irq_in1[24]),
        .csr_p_pending1_pending_irq56_in (pending_irq_in1[25]),
        .csr_p_pending1_pending_irq57_in (pending_irq_in1[26]),
        .csr_p_pending1_pending_irq58_in (pending_irq_in1[27]),
        .csr_p_pending1_pending_irq59_in (pending_irq_in1[28]),
        .csr_p_pending1_pending_irq60_in (pending_irq_in1[29]),
        .csr_p_pending1_pending_irq61_in (pending_irq_in1[30]),
        .csr_p_pending1_pending_irq62_in (pending_irq_in1[31]),

        // ENABLE outputs
        .csr_p_enable0_enable_irq1_out  (enable_irq_out0[1]),
        .csr_p_enable0_enable_irq2_out  (enable_irq_out0[2]),
        .csr_p_enable0_enable_irq3_out  (enable_irq_out0[3]),
        .csr_p_enable0_enable_irq4_out  (enable_irq_out0[4]),
        .csr_p_enable0_enable_irq5_out  (enable_irq_out0[5]),
        .csr_p_enable0_enable_irq6_out  (enable_irq_out0[6]),
        .csr_p_enable0_enable_irq7_out  (enable_irq_out0[7]),
        .csr_p_enable0_enable_irq8_out  (enable_irq_out0[8]),
        .csr_p_enable0_enable_irq9_out  (enable_irq_out0[9]),
        .csr_p_enable0_enable_irq10_out (enable_irq_out0[10]),
        .csr_p_enable0_enable_irq11_out (enable_irq_out0[11]),
        .csr_p_enable0_enable_irq12_out (enable_irq_out0[12]),
        .csr_p_enable0_enable_irq13_out (enable_irq_out0[13]),
        .csr_p_enable0_enable_irq14_out (enable_irq_out0[14]),
        .csr_p_enable0_enable_irq15_out (enable_irq_out0[15]),
        .csr_p_enable0_enable_irq16_out (enable_irq_out0[16]),
        .csr_p_enable0_enable_irq17_out (enable_irq_out0[17]),
        .csr_p_enable0_enable_irq18_out (enable_irq_out0[18]),
        .csr_p_enable0_enable_irq19_out (enable_irq_out0[19]),
        .csr_p_enable0_enable_irq20_out (enable_irq_out0[20]),
        .csr_p_enable0_enable_irq21_out (enable_irq_out0[21]),
        .csr_p_enable0_enable_irq22_out (enable_irq_out0[22]),
        .csr_p_enable0_enable_irq23_out (enable_irq_out0[23]),
        .csr_p_enable0_enable_irq24_out (enable_irq_out0[24]),
        .csr_p_enable0_enable_irq25_out (enable_irq_out0[25]),
        .csr_p_enable0_enable_irq26_out (enable_irq_out0[26]),
        .csr_p_enable0_enable_irq27_out (enable_irq_out0[27]),
        .csr_p_enable0_enable_irq28_out (enable_irq_out0[28]),
        .csr_p_enable0_enable_irq29_out (enable_irq_out0[29]),
        .csr_p_enable0_enable_irq30_out (enable_irq_out0[30]),
        .csr_p_enable0_enable_irq31_out (enable_irq_out0[31]),

        .csr_p_enable1_enable_irq32_out (enable_irq_out1[1]),
        .csr_p_enable1_enable_irq33_out (enable_irq_out1[2]),
        .csr_p_enable1_enable_irq34_out (enable_irq_out1[3]),
        .csr_p_enable1_enable_irq35_out (enable_irq_out1[4]),
        .csr_p_enable1_enable_irq36_out (enable_irq_out1[5]),
        .csr_p_enable1_enable_irq37_out (enable_irq_out1[6]),
        .csr_p_enable1_enable_irq38_out (enable_irq_out1[7]),
        .csr_p_enable1_enable_irq39_out (enable_irq_out1[8]),
        .csr_p_enable1_enable_irq40_out (enable_irq_out1[9]),
        .csr_p_enable1_enable_irq41_out (enable_irq_out1[10]),
        .csr_p_enable1_enable_irq42_out (enable_irq_out1[11]),
        .csr_p_enable1_enable_irq43_out (enable_irq_out1[12]),
        .csr_p_enable1_enable_irq44_out (enable_irq_out1[13]),
        .csr_p_enable1_enable_irq45_out (enable_irq_out1[14]),
        .csr_p_enable1_enable_irq46_out (enable_irq_out1[15]),
        .csr_p_enable1_enable_irq47_out (enable_irq_out1[16]),
        .csr_p_enable1_enable_irq48_out (enable_irq_out1[17]),
        .csr_p_enable1_enable_irq49_out (enable_irq_out1[18]),
        .csr_p_enable1_enable_irq50_out (enable_irq_out1[19]),
        .csr_p_enable1_enable_irq51_out (enable_irq_out1[20]),
        .csr_p_enable1_enable_irq52_out (enable_irq_out1[21]),
        .csr_p_enable1_enable_irq53_out (enable_irq_out1[22]),
        .csr_p_enable1_enable_irq54_out (enable_irq_out1[23]),
        .csr_p_enable1_enable_irq55_out (enable_irq_out1[24]),
        .csr_p_enable1_enable_irq56_out (enable_irq_out1[25]),
        .csr_p_enable1_enable_irq57_out (enable_irq_out1[26]),
        .csr_p_enable1_enable_irq58_out (enable_irq_out1[27]),
        .csr_p_enable1_enable_irq59_out (enable_irq_out1[28]),
        .csr_p_enable1_enable_irq60_out (enable_irq_out1[29]),
        .csr_p_enable1_enable_irq61_out (enable_irq_out1[30]),
        .csr_p_enable1_enable_irq62_out (enable_irq_out1[31]),


        // Threshold
        .csr_p_threshold_threshold_out(csr_p_threshold_threshold_out),

        // Claim/Complete
        .csr_p_claimcomplete_cp_out(csr_p_claimcomplete_cp_out),
        .csr_p_claimcomplete_cp_in(csr_p_claimcomplete_cp_in),

        .complete_flag  (complete_flag),
        .complete_id    (complete_id),
        .claim_flag     (claim_flag),

        // APB
        .psel           (PSEL),
        .paddr          (PADDR),
        .penable        (PENABLE),
        .pwrite         (PWRITE),
        .pwdata         (PWDATA),
        .pstrb          (PSTRB),
        .prdata         (PRDATA),
        .pready         (PREADY),
        .pslverr        (PSLVERR)
    );



    reg [5:0] highest_id;
    reg [2:0] highest_prio;
    reg claim_flagpre = 0;
    reg complete_flagpre = 0;


//Bật/tắt chờ ngắt 
    always @(posedge PCLK) begin
        if (!PRESETn) begin
            pending_irq_in0 <= 1'b0;
        end
        else begin
            if (irqs1_rxuart)       pending_irq_in0[1] <= 1'b1;
            if (irqs2_txuart)       pending_irq_in0[2] <= 1'b1;

            if (irqs3_pgpio0)       pending_irq_in0[3]  <= 1'b1;
            if (irqs4_ngpio0)       pending_irq_in0[4]  <= 1'b1;
            if (irqs5_pgpio1)       pending_irq_in0[5]  <= 1'b1;
            if (irqs6_ngpio1)       pending_irq_in0[6]  <= 1'b1;
            if (irqs7_pgpio2)       pending_irq_in0[7]  <= 1'b1;
            if (irqs8_ngpio2)       pending_irq_in0[8]  <= 1'b1;
            if (irqs9_pgpio3)       pending_irq_in0[9]  <= 1'b1;
            if (irqs10_ngpio3)      pending_irq_in0[10] <= 1'b1;
            if (irqs11_pgpio4)      pending_irq_in0[11] <= 1'b1;
            if (irqs12_ngpio4)      pending_irq_in0[12] <= 1'b1;
            if (irqs13_pgpio5)      pending_irq_in0[13] <= 1'b1;
            if (irqs14_ngpio5)      pending_irq_in0[14] <= 1'b1;
            if (irqs15_pgpio6)      pending_irq_in0[15] <= 1'b1;
            if (irqs16_ngpio6)      pending_irq_in0[16] <= 1'b1;
            if (irqs17_pgpio7)      pending_irq_in0[17] <= 1'b1;
            if (irqs18_ngpio7)      pending_irq_in0[18] <= 1'b1;
            if (irqs19_pgpio8)      pending_irq_in0[19] <= 1'b1;
            if (irqs20_ngpio8)      pending_irq_in0[20] <= 1'b1;
            if (irqs21_pgpio9)      pending_irq_in0[21] <= 1'b1;
            if (irqs22_ngpio9)      pending_irq_in0[22] <= 1'b1;
            if (irqs23_pgpio10)     pending_irq_in0[23] <= 1'b1;
            if (irqs24_ngpio10)     pending_irq_in0[24] <= 1'b1;
            if (irqs25_pgpio11)     pending_irq_in0[25] <= 1'b1;
            if (irqs26_ngpio11)     pending_irq_in0[26] <= 1'b1;
            if (irqs27_pgpio12)     pending_irq_in0[27] <= 1'b1;
            if (irqs28_ngpio12)     pending_irq_in0[28] <= 1'b1;
            if (irqs29_pgpio13)     pending_irq_in0[29] <= 1'b1;
            if (irqs30_ngpio13)     pending_irq_in0[30] <= 1'b1;
            if (irqs31_pgpio14)     pending_irq_in0[31] <= 1'b1;

            if (irqs32_ngpio14)     pending_irq_in1[1] <= 1'b1;
            if (irqs33_pgpio15)     pending_irq_in1[2] <= 1'b1;
            if (irqs34_ngpio15)     pending_irq_in1[3] <= 1'b1;
          
            if (claim_flag && !claim_flagpre) begin
                if(PRDATA[5:0] < 32) pending_irq_in0[PRDATA[5:0]] <= 1'b0;
                else pending_irq_in1[PRDATA[5:0]-31] <= 1'b0;
            end
        end
    end

//Bật/tắt báo ngắt đang được xử lý
    always @(posedge PCLK) begin
        if (!PRESETn) begin
            active_irq <= 1'b0;
        end
        else begin
            claim_flagpre <= claim_flag;
            complete_flagpre <= complete_flag;
            if(claim_flag && !claim_flagpre) active_irq[highest_id] <= 1'b1;
            if(complete_flag && !complete_flagpre) active_irq[complete_id] <= 1'b0;
        end
    end

//Tìm ngắt có ưu tiên cao nhất
//    reg [5:0] highest_id;
//    reg [2:0] highest_prio;

    integer i;
    always @(*) begin
        highest_id   = 6'd0;
        highest_prio = 3'd0;

        // duyệt toàn bộ IRQ
//        for (i = 1; i <= 31; i = i + 1) begin
//            if ( (pending_irq_in0[i]) &&
//                 (enable_irq_out0[i]  == 1'b1) &&
//                 (csr_priority[i] > csr_p_threshold_threshold_out) ) begin
//                if (csr_priority[i] > highest_prio) begin
//                    highest_prio = csr_priority[i];
//                    highest_id   = i[5:0];
//                end
//            end
//        end

//        for (i = 1; i <= 31; i = i + 1) begin
//            if ( (pending_irq_in1[i]) &&
//                 (enable_irq_out1[i]  == 1'b1) &&
//                 (csr_priority[i+31] > csr_p_threshold_threshold_out) ) begin
//                if (csr_priority[i+31] > highest_prio) begin
//                    highest_prio = csr_priority[i+31];
//                    highest_id   = i[5:0] + 31;
//                end
//            end
//        end
    end

//Báo ngắt khi có yêu cầu, ghi ID cần xử lý vào Claim/Complete Register
    always @(posedge PCLK) begin
        if (!PRESETn) begin
            csr_p_claimcomplete_cp_in <= 6'd0;
            irq_out <= 1'b0;
        end
        else begin
            if (highest_prio != 0 && !(|active_irq) && trap_en) begin
                irq_out <= 1'b1;
            end 
            else begin      
                irq_out <= 1'b0;
            end
            csr_p_claimcomplete_cp_in <= highest_id;
        end
    end

//MEIP cho CSR mip: irq_external_pending
assign irq_external_pending = highest_prio != 0;

endmodule