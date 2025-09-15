module RV32I_core(
    input           clk,    
    input           rst,

    output  [31:0 ] peri_addr,
    output  [31:0 ] peri_wdata,
    output  [ 3:0 ] peri_wmask,
    input   [31:0 ] peri_rdata,
    output          peri_wen,
    output          peri_ren,

    output  [ 2:0 ] peri_burst,
     
    output  [ 1:0 ] peri_htrans,

    input           peri_rvalid,
    input           peri_wdone,
    input           peri_err,

    input           irq_flag,
    input           irq_external_pending,
    output          trap_en
//    input   [ 5:0 ] irq_claim_id
);
/*===========================================================================
1. Basic Mode (No Pipeline – 5 Cycles per Instruction)
When FIVE_STATE_EN is defined, the CPU operates in its most basic form without pipelining.  
Each instruction goes through 5 sequential stages (such as Fetch, Decode, Execute, Memory,
and Writeback) but only one stage is processed at a time. As a result, one instruction re-
-quires 5 clock cycles to complete. This mode prioritizes simplicity and ease of verifica-
-tion over speed.
                            `define FIVE_STATE_EN
===========================================================================*/
//`define FIVE_STATE_EN


/*===========================================================================
2. Fast Mode (No Pipeline – 3 Cycles per Instruction)
When THREE_STATE_EN is defined, the CPU still operates without pipelining but optimizes t-
-he execution flow to merge or shorten certain stages, allowing each instruction to compl-
-ete in just 3 clock cycles. While this is faster than the basic mode, it is still sequen-
-tial and does not overlap instructions like a true pipeline.
                            `define THREE_STATE_EN
===========================================================================*/
//`define THREE_STATE_EN


/*===========================================================================
3. Advanced Mode (Pipelined – 1 Cycle per Instruction)
When FIVES_PIPELINE_EN is defined, the CPU uses pipeline techniques to overlap multiple i-
-nstruction stages (e.g., while one instruction is in Decode, another can be in Execute).
With this approach, once the pipeline is filled, the CPU can achieve a throughput of 1 in-
-struction per clock cycle, significantly improving performance compared to non-pipelined
modes.
                            `define FIVES_PIPELINE_EN
===========================================================================*/
`define FIVES_PIPELINE_EN
`define DEBUG_EN
//`define CLKDIV8
//`define CLK60M

`ifdef FIVE_STATE_EN
    `undef THREE_STATE_EN
    `undef FIVES_PIPELINE_EN
`endif
`ifdef THREE_STATE_EN
    `undef FIVE_STATE_EN
    `undef FIVES_PIPELINE_EN
`endif
`ifdef FIVES_PIPELINE_EN
    `undef THREE_STATE_EN
    `undef FIVE_STATE_EN
`endif




localparam  INSTR_FIRST     =   32'h00000000;
localparam  INSTR_NOP       =   32'h00000013;
localparam  VALUE_RESET32   =   32'h00000000;
localparam  VALUE_RESET     =   0;


//for CPU 5 instruction cycles
localparam  IF              =   0;
localparam  ID              =   1;
localparam  EX              =   2;
localparam  MEM             =   3;
localparam  WB              =   4;

localparam  Fetchoh         =   5'b00001 <<  IF;        //Fetch
localparam  Decodeoh        =   5'b00001 <<  ID;        //Decode
localparam  Executeoh       =   5'b00001 <<  EX;        //Execute
localparam  MemoryAoh       =   5'b00001 <<  MEM;       //Memory access
localparam  WriteBoh        =   5'b00001 <<  WB;        //Write back


//for CPU 3 instruction cycles
localparam  FD              =   0;                      //Fetch and decodes
localparam  EM              =   1;                      //Execute and access memory
localparam  WM              =   2;                      //Write back reg or mem

localparam  FetADec         =   5'b00001 <<  FD;
localparam  ExeAMem         =   5'b00001 <<  EM;
localparam  WRegOMem        =   5'b00001 <<  WM;

localparam  CLOCKSYS        =   27;//27Mhz



wire    [31:0 ] memd_ldata, memins_rdata;
reg     [31:0 ] mem_addr;
wire    [31:0 ] memins_addr;

reg     [31:0 ] processor_data;
wire            s0_sel_mem = (mem_addr[31:28] == 4'h2);
wire clksys;

`ifdef FIVE_STATE_EN
    always @(*) begin
        case(s0_sel_mem)
            1'b1: processor_data = memd_ldata;
            1'b0: processor_data = peri_rdata;
            default: processor_data = 32'h00000000;
        endcase
    end
`endif
`ifdef THREE_STATE_EN
    always @(*) begin
        case(s0_sel_mem)
            1'b1: processor_data = memd_ldata;
            1'b0: processor_data = peri_rdata;
            default: processor_data = 32'h00000000;
        endcase
    end
`endif
`ifdef FIVES_PIPELINE_EN
    reg s0_sel_memshift1;

    always @(posedge clksys) begin
        s0_sel_memshift1 <= s0_sel_mem;
    end

    always @(*) begin
        case(s0_sel_memshift1)
            1'b1: processor_data = memd_ldata;
            1'b0: processor_data = peri_rdata;
            default: processor_data = 32'h00000000;
        endcase
    end
`endif

    
`ifndef CLKDIV8
`ifndef CLK60M
    assign clksys = clk;//27M
`endif
`endif

`ifdef CLKDIV8
`ifndef CLK60M
    Gowin_CLKDIV CLKDIV8(
        .clkout(clksys), //output clkout
        .hclkin(clk), //input hclkin
        .resetn(rst) //input resetn
    );
`endif
`endif

`ifdef CLK60M
`ifndef CLKDIV8
    Gowin_rPLL CLK60M(
        .clkout(clksys), //output clkout
        .clkin(clk) //input clkin
    );
`endif
`endif


/*-------------------------------------------------
This module decodes 32-bit RV32I instructions by extracting fields such as opcode, rd, rs1, rs2, funct3, funct7, and immediate values. 
It identifies the instruction format (R, I, S, B, U, J) and generates control signals for execution units.
-------------------------------------------------*/
reg     [31:0 ] instr_data;
wire    [31:0 ] Immediate;
wire            insALUImm, insALUReg, insLUI,
                insAUIPC, insJAL, insJALR,
                insBRA, insLOAD, insSTORE,
                insSYS, insFENCE;
wire    [ 2:0 ] funct3;
wire    [ 7:0 ] funct3oh;
wire    [ 6:0 ] funct7;
wire    [ 4:0 ] regrs2;
wire    [ 4:0 ] regrs1;
wire    [ 4:0 ] regrd;
wire            en_rs1;
wire            en_rs2;
wire            en_rd;
wire    [ 3:0 ] pred;
wire    [ 3:0 ] succ;
wire    [11:0 ] csr_addr;
wire            insMRET;


RV32_Decoder Maindecoder(
    .instr_data         (instr_data),
    .Immediate          (Immediate),
    .insALUImm          (insALUImm),
    .insALUReg          (insALUReg),
    .insLUI             (insLUI),
    .insAUIPC           (insAUIPC),
    .insJAL             (insJAL),
    .insJALR            (insJALR),
    .insBRA             (insBRA),
    .insLOAD            (insLOAD),
    .insSTORE           (insSTORE),
    .insSYS             (insSYS),
    .insFENCE           (insFENCE),
    .funct3             (funct3),
    .funct3oh           (funct3oh),
    .funct7             (funct7),
    .regrs2             (regrs2),
    .regrs1             (regrs1),
    .regrd              (regrd),
    .en_rs1             (en_rs1),
    .en_rs2             (en_rs2),
    .en_rd              (en_rd),
    .pred               (pred),
    .succ               (succ),
    .csr_addr           (csr_addr),
    .insMRET            (insMRET)
);



/*---------------------------------------------------
Instruction Memory:
A read-only memory that stores machine instructions. 
The CPU provides an address and receives the corresponding 32-bit instruction. 
It is used during the instruction fetch stage and does not support write operations.

Data Memory:
A read-write memory used to store program data. 
The CPU can read from or write to a specified address. 
It is accessed during the load/store stage of instruction execution.
---------------------------------------------------*/
//reg     [31:0 ] mem_addr, memint_addr;
reg     [31:0 ] memd_sdata;
wire    [31:0 ] memd_sdatafn;       assign  memd_sdatafn    =   memd_sdata << (mem_addr[1:0]*8);    //Shift data for store byte or haftw
reg             memd_lready; 
wire            memins_read;
wire    [ 3:0 ] memd_mask;
reg             memd_senable = 1'b0;

wire            flag_branch;
reg             predict_taken2  =   VALUE_RESET;

reg     [31:0 ] PCnext          =   INSTR_FIRST;
reg     [31:0 ] PCnext_fast     =   INSTR_FIRST;
reg     [31:0 ] PCnext_actual     =   INSTR_FIRST;
reg             wrong_pred      =   VALUE_RESET;
reg             wrong_predshift1;
wire            wrong_predfast; 
assign          wrong_predfast  =   insBRA & (flag_branch != predict_taken2) & !wrong_pred & !wrong_predshift1;//Lưu ý chỉ bật khi trước đó 2 chu kì không có sai rẽ nhánh.
wire    [31:0 ] memins_rdata_pred;

wire            isRAW_Hazardrs1_2cyc_forJALR;   //Vi JALR su du data rs1
wire            wait_peri_as;
wire            mem_renablepred;

Instruction_memory #(
//    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/FW RV32I/firmware/firmware.hex"),
    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware_instr.hex"),
    .SIZE       (4096)  //64Kb
) ins_mem(
    .clk            (clksys),
`ifdef FIVES_PIPELINE_EN
    .l_pause        (isRAW_Hazardrs1_2cyc_forJALR || wait_peri_as),
`endif
`ifdef THREE_STATE_EN
    .l_pause        (1'b0),
`endif
`ifdef FIVE_STATE_EN
    .l_pause        (1'b0),
`endif
    .mem_addr       (memins_addr),
    .mem_addrpred   (PCnext_fast),
    .mem_rdata      (memins_rdata),
    .mem_rdata_pred (memins_rdata_pred),
    .mem_renable    (memins_read),
    .mem_renablepred(mem_renablepred)
);

wire    [ 3:0 ] memd_maskfn_mem;
wire            memd_lenfn_mem;
`ifdef THREE_STATE_EN
    assign memd_maskfn_mem  =   memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTORE}};
    assign memd_lenfn_mem   =   memd_lready & insLOAD;
`endif
`ifdef FIVE_STATE_EN
    assign memd_maskfn_mem  =   memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTORE}};
    assign memd_lenfn_mem   =   memd_lready & insLOAD;
`endif
`ifdef FIVES_PIPELINE_EN
reg insSTOREshift1;
reg insLOADshift1;
reg insSTOREshift2;
reg insLOADshift2;
reg insLOADshift3;
    assign memd_maskfn_mem  =   memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTORE}};
    assign memd_lenfn_mem   =   memd_lready & insLOAD & s0_sel_mem;
`endif

Data_memory #(
    //.MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/FW RV32I/firmware/firmware.hex"),
    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware_data.hex"),
    .SIZE       (4096)  //16Kb
) data_mem(
    .clk        (clksys),
    .rst        (rst),
    .mem_addr   ({4'h0, mem_addr[27:0]}),
    .mem_ldata  (memd_ldata),
    .mem_sdata  (memd_sdatafn),    
    .mem_lenable(memd_lenfn_mem),
    .mem_mask   (memd_maskfn_mem)
);

wire    [31:0 ] mem_addrforl; 
//load, store 8bit, 16bit or 32bit
wire    lsByte ;// =   (funct3[1:0] == 2'b00);
wire    lsHaftW;// =   (funct3[0]);
wire    lsWord ;// =   (funct3[1]);
wire    lsSign ;// =   (!funct3[2]);

wire    lsBytefn;
wire    lsHaftWfn;
wire    lsWordfn ;
wire    lsSignfn ;
`ifdef THREE_STATE_EN
    assign lsByte           =   (funct3[1:0] == 2'b00);
    assign lsHaftW          =   (funct3[0]);
    assign lsWord           =   (funct3[1]);
    assign lsSign           =   (!funct3[2]);

    assign lsBytefn         =   lsByte;
    assign lsHaftWfn        =   lsHaftW;
    assign lsWordfn         =   lsWord;
    assign lsSignfn         =   lsSign;

    assign mem_addrforl     =   mem_addr;
`endif
`ifdef FIVE_STATE_EN
    assign lsByte           =   (funct3[1:0] == 2'b00);
    assign lsHaftW          =   (funct3[0]);
    assign lsWord           =   (funct3[1]);
    assign lsSign           =   (!funct3[2]);

    assign lsBytefn         =   lsByte;
    assign lsHaftWfn        =   lsHaftW;
    assign lsWordfn         =   lsWord;
    assign lsSignfn         =   lsSign;

    assign mem_addrforl     =   mem_addr;
`endif
`ifdef FIVES_PIPELINE_EN
reg     [ 2:0 ] funct3shift1;
reg     [ 2:0 ] funct3shift2;
reg     [31:0 ] mem_addrshift1;

always @(posedge clksys) mem_addrshift1 <= mem_addr;
    assign lsByte           =   (funct3[1:0] == 2'b00);
    assign lsHaftW          =   (funct3[0]);
    assign lsWord           =   (funct3[1]);
    assign lsSign           =   (!funct3[2]);

    assign lsBytefn =   (funct3shift1[1:0] == 2'b00);
    assign lsHaftWfn=   (funct3shift1[0]);
    assign lsWordfn =   (funct3shift1[1]);
    assign lsSignfn =   (!funct3shift1[2]);

    assign mem_addrforl     =   mem_addrshift1;
`endif

//gen mem_mask and make load data 8bit, 16bit or 32bit
assign          memd_mask   = 
                    lsWord  ?   4'b1111                         :
                    lsHaftW ?   (mem_addr[1]?4'b1100:4'b0011)  :
                    lsByte  ?   (mem_addr[1]?(mem_addr[0]?4'b1000:4'b0100):
                                              (mem_addr[0]?4'b0010:4'b0001)):
                    4'b0;

wire    [31:0]  byte_data;
wire    [31:0]  halfw_data;

assign byte_data = (mem_addrforl[1:0] == 2'b00) ? processor_data[7:0]   :
                   (mem_addrforl[1:0] == 2'b01) ? processor_data[15:8]  :
                   (mem_addrforl[1:0] == 2'b10) ? processor_data[23:16] :
                                                  processor_data[31:24] ;
assign halfw_data = mem_addrforl[1] ? processor_data[31:16] : processor_data[15:0];


wire    [31:0]  mem_ldmask;     //Mem load data(mask)

assign mem_ldmask = lsWordfn  ? processor_data :
                    lsHaftWfn ? (lsSignfn ? {{16{halfw_data[15]}}, halfw_data[15:0]} :
                                        {16'b0, halfw_data[15:0]}) :
                    lsBytefn  ? (lsSignfn ? {{24{byte_data[7]}}, byte_data[7:0]} :
                                        {24'b0, byte_data[7:0]}) :
                    32'b0;

/*---------------------------------------------------------
The Arithmetic Logic Unit performs integer operations such as addition, subtraction, bitwise logic, and shifts. 
It also supports comparison operations used for branch instructions, including signed and unsigned comparisons. 
The ALU receives two operands and an operation code, and outputs the result along with a branch condition cr if needed.
---------------------------------------------------------*/
wire    [31:0 ] result_ALU;
//wire            flag_branch;
wire    [31:0 ] data_rs1;
wire    [31:0 ] data_rs1fn;
wire    [31:0 ] data_rs2;   
wire    [31:0 ] data_rs2fn;
wire    [31:0 ] temphz;

//Hazard RAW
reg     [ 4:0 ] regrd_shifthz1  =   5'h00;          //Du lai rd 1 chu ki truoc de so sanh. Phục vụ phát hiện hazard 1 chu kì
reg     [ 4:0 ] regrd_shifthz2  =   5'h00;          //Du lai rd 2 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì
reg     [ 4:0 ] regrd_shifthz3  =   5'h00;          //Du lai rd 3 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì

reg     [ 4:0 ] regrs1_shifthz1 =   5'h00;          //Du lai rs1 1 chu ki truoc de so sanh. Phục vụ phát hiện hazard 1 chu kì
reg     [ 4:0 ] regrs1_shifthz2 =   5'h00;          //Du lai rs1 2 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì
reg     [ 4:0 ] regrs1_shifthz3 =   5'h00;          //Du lai rs1 3 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì

reg     [11:0 ] csr_addr_shift1 =   5'h00;         //Du lai csr_addr 1 chu ki truoc de so sanh. Phục vụ phát hiện hazard 1 chu kì
reg     [11:0 ] csr_addr_shift2 =   5'h00;         //Du lai csr_addr 2 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì
reg     [11:0 ] csr_addr_shift3 =   5'h00;         //Du lai csr_addr 3 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì


wire            isRAW_Hazardrs1_1cyc_forWB;    //phat hien hazard 1 chu kì
wire            isRAW_Hazardrs2_1cyc_forWB;    //phat hien hazard 1 chu kì
wire            isRAW_Hazardrs1_2cyc_forWB;    //phat hien hazard 2 chu kì
wire            isRAW_Hazardrs2_2cyc_forWB;    //phat hien hazard 2 chu kì
wire            isRAW_Hazardrs1_3cyc_forWB;    //phat hien hazard 2 chu kì
wire            isRAW_Hazardrs2_3cyc_forWB;    //phat hien hazard 2 chu kì


reg             en_rdhz1        =   VALUE_RESET;
reg             en_rdhz2        =   VALUE_RESET;
reg             en_rdhz3        =   VALUE_RESET;

reg             en_rs1hz1        =   VALUE_RESET;
reg             en_rs1hz2        =   VALUE_RESET;

//Result of ins
reg     [31:0 ] result;
wire    [31:0 ] resulthz_1cyc;
wire    [31:0 ] resulthz_2cyc;
wire    [31:0 ] resulthz_3cyc;

`ifdef THREE_STATE_EN
    assign data_rs2fn   =   (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;
    assign data_rs1fn   =   (insSYS)?((funct3[2])?regrs1:data_rs1):data_rs1;
`endif
`ifdef FIVE_STATE_EN
    assign data_rs2fn   =   (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;
    assign data_rs1fn   =   (insSYS)?((funct3[2])?regrs1:data_rs1):data_rs1;
`endif
`ifdef FIVES_PIPELINE_EN
wire    [31:0 ] data_rs1hz_1cyc;
wire    [31:0 ] data_rs2hz_1cyc;
wire    [31:0 ] temprs2_hz;
wire    [31:0 ] temprs1_hz;

wire    [31:0 ] data_rs1hz_2cyc;
wire    [31:0 ] data_rs2hz_2cyc;

wire    [31:0 ] data_rs1hz_3cyc;
wire    [31:0 ] data_rs2hz_3cyc;

reg     [31:0 ] mem_ldmaskshift1    =   VALUE_RESET32; 
reg     [31:0 ] mem_ldmaskshift2    =   VALUE_RESET32; 

    assign temprs2_hz       =   (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;
    assign temprs1_hz       =   (insSYS)?((funct3[2])?regrs1:data_rs1):data_rs1;
    assign data_rs2hz_1cyc  =   (isRAW_Hazardrs2_1cyc_forWB)?((insLOADshift1)?mem_ldmask:resulthz_1cyc):
                                                               temprs2_hz;
    assign data_rs1hz_1cyc  =   (isRAW_Hazardrs1_1cyc_forWB)?((insLOADshift1)?mem_ldmask:resulthz_1cyc):
                                                               temprs1_hz;
    assign data_rs2hz_2cyc  =   (isRAW_Hazardrs2_2cyc_forWB)?((insLOADshift2)?mem_ldmaskshift1:resulthz_2cyc):
                                                               temprs2_hz;
    assign data_rs1hz_2cyc  =   (isRAW_Hazardrs1_2cyc_forWB)?((insLOADshift2)?mem_ldmaskshift1:resulthz_2cyc):
                                                               temprs1_hz;
    assign data_rs2hz_3cyc  =   (isRAW_Hazardrs2_3cyc_forWB)?((insLOADshift3)?mem_ldmaskshift2:resulthz_3cyc):
                                                               temprs2_hz;
    assign data_rs1hz_3cyc  =   (isRAW_Hazardrs1_3cyc_forWB)?((insLOADshift3)?mem_ldmaskshift2:resulthz_3cyc):
                                                               temprs1_hz;

    assign data_rs2fn       =   (isRAW_Hazardrs2_1cyc_forWB)?data_rs2hz_1cyc:
                                (isRAW_Hazardrs2_2cyc_forWB)?data_rs2hz_2cyc:
                                                             data_rs2hz_3cyc;
    assign data_rs1fn       =   (isRAW_Hazardrs1_1cyc_forWB)?data_rs1hz_1cyc:
                                (isRAW_Hazardrs1_2cyc_forWB)?data_rs1hz_2cyc:
                                                             data_rs1hz_3cyc;
`endif



wire    [31:0 ] csr_rdata;
ALU_unit ALU(
    .isALUimm   (insALUImm),
    .isALUreg   (insALUReg),
    .isBranch   (insBRA),
    .isSYS      (insSYS),
    .funct3oh   (funct3oh),
    .funct3     (funct3),
    .funct7     (funct7),
    .rs1        (data_rs1fn),
    .rs2        (data_rs2fn),
    .csr_rdata  (csr_rdata),
    .result     (result_ALU),
    .correct    (flag_branch)
);



/*---------------------------------------------------------
This block contains 32 general-purpose 32-bit registers (x0–x31).
Register x0 is hardwired to zero and cannot be modified. The register file supports
two read ports and one write port, allowing simultaneous access to rs1, rs2, and rd
operands. It is used for storing intermediate and final results during instruction execution.
---------------------------------------------------------*/
wire    [31:0 ] data_desfn;
wire    [ 4:0 ] regrdfn;
//reg     [31:0 ] result;
reg     [31:0 ] data_des;
reg     [ 4:0 ] regrd_shiftpl   =   5'h00;
`ifdef THREE_STATE_EN
    assign data_desfn = (insLOAD)?mem_ldmask:result;
    assign regrdfn = regrd;
`endif
`ifdef FIVE_STATE_EN
    assign data_desfn = data_des;
    assign regrdfn = regrd;
`endif
`ifdef FIVES_PIPELINE_EN
    assign data_desfn = data_des;
    assign regrdfn = regrd_shiftpl;
`endif


//wire    [31:0 ] data_rs1;
//wire    [31:0 ] data_rs2;
wire    [31:0 ] data_rs1pred;
wire    [ 4:0 ] regrs1pred;
reg             data_valid;

//wire    [31:0 ] csr_rdata;
reg             csr_we;
reg     [31:0 ] csr_wdata = VALUE_RESET32;


Registers_unit Regunit(
    .clk            (clksys), 
    .rst            (rst),

    .rs1            (regrs1),
    .rs1pred        (regrs1pred),
    .rs2            (regrs2),
    .rd             (regrdfn),
    .data_des       (data_desfn),
    .data_valid     (data_valid),
    .data_rs1       (data_rs1),
    .data_rs2       (data_rs2),
    .data_rs1pred   (data_rs1pred)
);




/*--------------------------------------------------------------------
This module implements a simplified RISC-V CSR unit for machine mode. It contains 
key machine CSRs such as mstatus, mie, mtvec, mepc, mcause, mtval, and mip, and
also supports counters like cycle, instret, and mtime. The unit updates counters
every clock cycle, allows CSR read/write through addresses, and automatically upd-
-ates trap-related CSRs (mepc, mcause, mtval) when a trap occurs. It provides a s-
-tandard interface for CPU core access to control, status, and performance monito-
-ring registers.
--------------------------------------------------------------------*/
reg     [11:0 ] csr_addrpl;
reg     [63:0 ] csr_instret =   64'b0;
reg     [63:0 ] csr_real_mtime  =   64'b0;
reg     [ 7:0 ] cnt         =   8'b1;

reg             csr_trap_taken  = 1'b0;
reg     [31:0 ] csr_trap_pc     = 32'd0;
wire    [31:0 ] csr_trap_rpc;
wire    [31:0 ] csr_trap_addr;
reg     [31:0 ] csr_trap_cause  = 32'd0;
reg             UpdateMEPC      = 1'b0;

CSR_unit CSRreg(
    .clk            (clksys), 
    .rst            (rst),
    .csr_addrr      (csr_addr),
`ifndef FIVES_PIPELINE_EN
    .csr_addrw      (csr_addr),
`endif
`ifdef FIVES_PIPELINE_EN
    .csr_addrw      (csr_addrpl),
`endif
    .csr_wdata      (csr_wdata),
    .csr_we         (csr_we),
    .csr_rdata      (csr_rdata),
    .csr_instret    (csr_instret),
    .real_mtime     (csr_real_mtime),    //tick

    .trap_taken     (csr_trap_taken),
    .trap_complete  (insMRET),
    .updateMEPC     (UpdateMEPC),
    .trap_pc        (csr_trap_pc),
    .trap_rpc       (csr_trap_rpc),
    .trap_addr      (csr_trap_addr),
    .trap_cause     (csr_trap_cause),
    
    .irq_software   (1'b0),
    .irq_timer      (1'b0),
    .irq_external   (irq_external_pending),
    .global_ie      (irq_en)
    
);
assign trap_en = irq_en;

//Count time (1tick = 1us)
always @(posedge clksys) begin
    if(!rst) begin
        cnt <= 1;
        csr_real_mtime <= 64'b0;
    end
    else begin
        cnt <= cnt + 1;
        if(cnt == CLOCKSYS) begin
            cnt <= 1;
            csr_real_mtime <= csr_real_mtime + 1;
        end
    end
end


assign      peri_burst      =   3'b000;
assign      peri_htrans     =  ((peri_ren || peri_wen) && !s0_sel_mem)?2'b10:2'b00;
assign      peri_addr       =   mem_addr;
assign      peri_wdata      =   memd_sdata;
assign      peri_wmask      =   memd_mask;
assign      peri_wen        =   insSTORE && memd_senable;
assign      peri_ren        =   memd_lready & insLOAD;
wire        peri_trans_done =   (peri_ren && peri_rvalid) || (peri_wen && peri_wdone);




reg     [31:0 ] PC      =   INSTR_FIRST;    //The program counter (PC) is a 32-bit register that holds the address of the current instruction being executed. 
                                            //After each instruction, the PC is typically incremented by 4 to point to the next instruction. 
//reg     [31:0 ] PCnext  =   INSTR_FIRST;
reg     [ 4:0 ] state   =   5'b00001;
reg     [31:0 ] load_data;
wire            insALU  =   insALUImm || insALUReg;


/*------------------------------------------------------------------------------------------
/*----------------------------------------------------------
This processor is organized into five stages:
    1. IF (Instruction Fetch): Fetches the next instruction from instruction memory.
    2. ID (Instruction Decode): Decodes the instruction, reads registers, and generates control signals.
    3. EX (Execute): Performs arithmetic or logic operations in the ALU, or computes branch targets.
    4. MEM (Memory Access): Accesses data memory for load/store instructions.
    5. WB (Write Back): Writes results back to the register file.
No pileline
------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
`ifdef FIVE_STATE_EN
    assign memins_read = state[IF]; //only Instruction Fetch
    assign memins_addr = PC;

//Bật cờ ngắt
reg irq_flagh = 0;
reg irq_active = 0;
    always @(posedge clksys) begin
        if(!rst) begin
            irq_flagh <= 1'b0;
        end
        else begin
            if(irq_flag) irq_flagh <= 1'b1;
            if(irq_active) irq_flagh <= 1'b0;
        end
    end

//Tính địa chỉ trở lại với ngắt
reg [31:0 ] PCreturnIRQ;
    always @(*) begin
        if(!rst) begin
            PCreturnIRQ = INSTR_FIRST;
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                insBRA:     PCreturnIRQ = flag_branch ? PC + Immediate : PC+4;
                insJAL:     PCreturnIRQ = PC + Immediate;
                insJALR:    PCreturnIRQ = (data_rs1 + Immediate) & ~32'h1;
                default:    PCreturnIRQ = PC + 4;
            endcase
        end
    end

    always @(posedge clksys) begin
        if(!rst) begin
            PC <= INSTR_FIRST;
            PCnext <= INSTR_FIRST;
            data_valid <= VALUE_RESET; 
            memd_lready <= VALUE_RESET;
            memd_senable <= VALUE_RESET;
            data_des <= VALUE_RESET32;
            memd_sdata <= VALUE_RESET32;
            state <= Fetchoh;
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                state[IF]   : begin
                    data_valid <= 1'b0;
                    csr_we <= 1'b0;
                    //PC <= PCnext;
                    state <= Decodeoh;    //ID
                end
                state[ID]   : begin
                    instr_data <= memins_rdata;
                    state <= Executeoh;   //EX
                end
                state[EX]   : begin
                    (*parallel_case*)
                    case(1'b1)
                        insBRA: begin
                            PCnext <= flag_branch ? PC + Immediate : PC+4;
                        end
                        insLOAD: begin
                            memd_lready <= 1'b1;
                            mem_addr <= data_rs1 + Immediate;
                        end
                        insSTORE: begin
                            mem_addr <= data_rs1 + Immediate;
                        end
                        insALU: begin
                            result <= result_ALU;
                        end
                        insLUI: begin
                            result <= Immediate;
                        end
                        insAUIPC: begin 
                            result <= PC + Immediate;
                        end
                        insJAL: begin
                            result <= PC + 4;
                            PCnext <= PC + Immediate;
                        end
                        insJALR: begin
                            result <= PC + 4;
                            PCnext <= (data_rs1 + Immediate) & ~32'h1;
                        end
                        insSYS: begin
                            result <= result_ALU;
                        end
                        default: state <= Fetchoh;
                    endcase

                    state <= (insLOAD || insSTORE) ? MemoryAoh : WriteBoh;
                    if (!(insBRA||insJAL||insJALR)) PCnext <= PC + 4;

                    csr_trap_taken <= 1'b0;
                    irq_active <= 1'b0;
                    if(irq_flagh) begin
                        irq_active <= 1'b1;
                        PCnext <= csr_trap_addr; 
                        csr_trap_taken <= 1'b1;
                        csr_trap_pc <= PCreturnIRQ;
                        csr_trap_cause <= 32'h8000000B;
                    end
                    if(insMRET) begin
                        PCnext <= csr_trap_rpc;
                    end
                    
                end
                state[MEM]  : begin
                    if(insLOAD) begin
                        load_data <= mem_ldmask;
                    end
                    else begin //insSTORE
                        memd_sdata <= data_rs2;
                        memd_senable <= 1'b1;
                    end
                    if(s0_sel_mem || peri_trans_done) begin
                        memd_lready <= 1'b0;
                        memd_senable <= s0_sel_mem;
                        state <= WriteBoh;
                    end
                end
                state[WB]   : begin
                    memd_senable <= 1'b0;
                    case(1'b1)
                        insLOAD: begin
                            data_des <= mem_ldmask;
                            data_valid <= 1'b1;
                        end
                        insALU | insLUI | insAUIPC | insJAL | insJALR: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                        insSYS: begin
                            data_valid <= 1'b1;
                            data_des <= csr_rdata;
                            csr_we <= !(regrs1 == 5'h00 && en_rs1);
                            csr_wdata <= result;
                        end
                    endcase
                    state <= Fetchoh;
                    PC <= PCnext;               //thay doi PC de fetch
                    csr_instret <= csr_instret + 1;
                end
            endcase
        end
    end
`endif


/*------------------------------------------------------------------------------------------
/*----------------------------------------------------------
This processor is organized into three states:
    1. DE (Decode): Fetches the instruction, decodes it, and reads registers.
    2. EM (Execute & Memory): Executes ALU operations or accesses data memory.
    3. WF (Write Back and Fetch): Writes results back to the register file or memory.
No pipeline
------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
`ifdef THREE_STATE_EN
    assign memins_read = 1'b1; //always read
    assign memins_addr = PC;

//Bật cờ ngắt
reg irq_flagh = 0;
reg irq_active = 0;
    always @(posedge clksys) begin
        if(!rst) begin
            irq_flagh <= 1'b0;
        end
        else begin
            if(irq_flag) irq_flagh <= 1'b1;
            if(irq_active) irq_flagh <= 1'b0;
        end
    end

//Tính địa chỉ trở lại với ngắt
reg [31:0 ] PCreturnIRQ;
    always @(*) begin
        if(!rst) begin
            PCreturnIRQ = INSTR_FIRST;
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                insBRA:     PCreturnIRQ = flag_branch ? PC + Immediate : PC+4;
                insJAL:     PCreturnIRQ = PC + Immediate;
                insJALR:    PCreturnIRQ = (data_rs1 + Immediate) & ~32'h1;
                default:    PCreturnIRQ = PC + 4;
            endcase
        end
    end

    always @(*) begin
        instr_data = memins_rdata;
    end

    always @(posedge clksys) begin
        if(!rst) begin
            PC <= INSTR_FIRST;
            PCnext <= INSTR_FIRST;
            data_valid <= VALUE_RESET; 
            memd_lready <= VALUE_RESET;
            memd_senable <= VALUE_RESET;
            data_des <= VALUE_RESET32;
            memd_sdata <= VALUE_RESET32;
            state <= FetADec;
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                state[FD]: begin
                    csr_we <= 1'b0;
                    data_valid <= 1'b0;
                    state <= ExeAMem;
                    csr_instret <= csr_instret + 1;
                end
                state[EM]: begin
                    (*parallel_case*)
                    case(1'b1)
                        insBRA: begin
                            PCnext <= flag_branch ? PC + Immediate : PC+4;
                        end
                        insLOAD: begin
                            memd_lready <= 1'b1;
                            mem_addr <= data_rs1 + Immediate;
                        end
                        insSTORE: begin
                            mem_addr <= data_rs1 + Immediate;
                            memd_sdata <= data_rs2;
                            memd_senable <= 1'b1;
                        end
                        insALU: begin
                            result <= result_ALU;
                            data_valid <= 1'b1;
                        end
                        insLUI: begin
                            result <= Immediate;
                            data_valid <= 1'b1;
                        end
                        insAUIPC: begin 
                            result <= PC + Immediate;
                            data_valid <= 1'b1;
                        end
                        insJAL: begin
                            result <= PC + 4;
                            PCnext <= PC + Immediate;
                            data_valid <= 1'b1;
                        end
                        insJALR: begin
                            data_valid <= 1'b1;
                            result <= PC + 4;
                            PCnext <= (data_rs1 + Immediate) & ~32'h1;
                        end
                        insSYS: begin
                            data_valid <= 1'b1;
                            result <= csr_rdata;
                            csr_we <= !(regrs1 == 5'h00 && en_rs1);
                            csr_wdata <= result_ALU;
                        end
                    endcase    
                    if (!(insBRA||insJAL||insJALR)) PCnext <= PC + 4;

                    csr_trap_taken <= 1'b0;
                    irq_active <= 1'b0;
                    if(irq_flagh) begin
                        irq_active <= 1'b1;
                        PCnext <= csr_trap_addr; 
                        csr_trap_taken <= 1'b1;
                        csr_trap_pc <= PCreturnIRQ;
                        csr_trap_cause <= 32'h8000000B;
                    end
                    if(insMRET) begin
                        PCnext <= csr_trap_rpc;
                    end

                    state <= WRegOMem;
                end
                state[WM]: begin
                    csr_we <= 1'b0;
                    case(1'b1)
                        insLOAD: begin
                            if(s0_sel_mem || peri_trans_done) begin
                                memd_lready <= 1'b0;
                                data_valid <= 1'b1;
                                state <= FetADec;
                                PC <= PCnext;
                            end
                        end
                        insSTORE: begin
//                            memd_sdata <= data_rs2;
                            if(s0_sel_mem || peri_trans_done) begin
                                memd_senable <= 1'b0;
                                state <= FetADec;
                                PC <= PCnext;
                            end
                        end
                        default: begin
                            //data_des <= result;
                            data_valid <= 1'b0;
                            state <= FetADec;
                            PC <= PCnext;
                        end
                    endcase 
                end
            endcase   
        end
    end
`endif


/*------------------------------------------------------------------------------------------
--------------------------------------------------------
 This processor is organized into five pipeline stages:
    1. IF  (Instruction Fetch) : Fetch the instruction from instruction memory.
    2. ID  (Instruction Decode): Decode the instruction and read operands from registers.
    3. EX  (Execute)          : Perform ALU operations or compute branch targets.
    4. MEM (Memory Access)    : Access data memory for load/store instructions.
    5. WB  (Write Back)       : Write results back to the register file.

 Pipelined architecture enables instruction-level parallelism.
 Each instruction passes through these stages in successive clock cycles.
 After the pipeline is filled, one instruction is completed per clock cycle.

 -------------------------------------------------------------------------------
 EXCEPTIONS / HAZARDS (summary):
   - In general, data hazards are resolved by forwarding (bypassing) where possible,
     so most RAW cases (ALU → ALU, EX→EX forwarding, MEM→EX when available) do not
     require pipeline bubbles.

   - JALR / indirect-jump hazards are special:
       * A JALR computes its next-PC from a register (rs1 + imm). That means the
         PC_next depends on the **value in rs1** rather than a PC-relative immediate.
       * If rs1 is produced by a prior instruction that is not yet available (e.g. a load),
         the pipeline **cannot** always forward the value early enough to compute PC_next
         at the usual pipeline point. Consequently a stall is required.

   - Stall policy (as used in this design):
       * If the implementation computes JALR's PC_next in the **ID stage (1-cycle JALR path)**:
           - A prior load → JALR pair will require **2 cycles of stall** to wait for the load data
             to become valid before computing the JALR target.
       * If the implementation computes JALR's PC_next in the **EX stage (2-cycle JALR path)**:
           - A prior load → JALR pair will require **1 cycle of stall** (because the load result
             can be forwarded from MEM into EX in time).
       * In all other normal data-dependency cases (ALU results, registers written earlier) the
         forwarding network supplies data so no extra stall is needed.

   - Control hazards (branches, jal) that use PC-relative immediates are easier:
       * For PC-relative JAL and conditional branches the target can be computed from the
         current PC and the immediate (no rs1) — these do not incur the same rs1-dependent
         JALR penalty. They still may suffer from misprediction flushes, which are handled
         by the branch control unit.
------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
`ifdef FIVES_PIPELINE_EN
reg     IDen                    =   VALUE_RESET;
reg     EXen                    =   VALUE_RESET;
reg     MEMen                   =   VALUE_RESET;
reg     WBen                    =   VALUE_RESET;
reg     Fetchena                =   VALUE_RESET;    
reg     [31:0 ] PCBraoJum       =   VALUE_RESET32;
reg     [31:0 ] temp            =   VALUE_RESET32;

reg     [ 2:0 ] CntfMem         =   3'd0;           //counter for memory access
reg     [31:0 ] Store_datapl1   =   VALUE_RESET32;
reg     [31:0 ] Store_datapl2   =   VALUE_RESET32;

reg     [31:0 ] mem_addrup      =   VALUE_RESET32;  //địa chỉ tới trước 1 chu kì cho memory access
reg     [31:0 ] result_shiftpl  =   VALUE_RESET32;
reg     [31:0 ] result_shiftpl1 =   VALUE_RESET32;

reg     [31:0 ] dwb_csr         =   VALUE_RESET32;
reg     [31:0 ] dwb_csr_shiftpl =   VALUE_RESET32;
reg     [31:0 ] dwb_csr_shiftpl1=   VALUE_RESET32;

reg     [31:0 ] choose1_1       =   VALUE_RESET32;
reg     [31:0 ] choose1_2       =   VALUE_RESET32;
reg     [31:0 ] choose1_3       =   VALUE_RESET32;
reg     [31:0 ] choose2_1       =   VALUE_RESET32;
reg     [31:0 ] choose2_2       =   VALUE_RESET32;
reg     [31:0 ] choose2_3       =   VALUE_RESET32;

//reg             wrong_pred      =   VALUE_RESET;
//reg             wrong_predshift1=   VALUE_RESET;

reg             predict_taken1  =   VALUE_RESET;
//reg             predict_taken2  =   VALUE_RESET;
reg             predict_taken3  =   VALUE_RESET;


reg     [ 4:0 ] regrd_shiftpl1  =   5'h00;
reg     [ 4:0 ] regrd_shiftpl2  =   5'h00;
reg     [ 4:0 ] regrd_shiftpl3  =   5'h00;
//reg     [ 4:0 ] regrd_shiftpl   =   5'h00;

reg             insALUImmshift1, insALURegshift1, insLUIshift1,
                insAUIPCshift1, insJALshift1, insJALRshift1,
                insBRAshift1, //insLOADshift1, insSTOREshift1,
                insSYSshift1, insFENCEshift1;
reg             insALUImmshift2, insALURegshift2, insLUIshift2,
                insAUIPCshift2, insJALshift2, insJALRshift2,
                insBRAshift2, //insLOADshift2, insSTOREshift2,
                insSYSshift2, insFENCEshift2;

reg             holdplus_forDE        =   VALUE_RESET;  
reg             holdplus_forEX        =   VALUE_RESET;  
reg             holdplus_forMEM       =   VALUE_RESET; 
reg             holdplus_forWB        =   VALUE_RESET; 

wire            en_rs1pred; 

//reg             insLOADshift3;

reg     [31:0 ] PCshift1        =   VALUE_RESET;




//reg     [ 4:0 ] regrd_shifthz1  =   5'h00;          //Du lai rd 1 chu ki truoc de so sanh. Phục vụ phát hiện hazard 1 chu kì
//reg     [ 4:0 ] regrd_shifthz2  =   5'h00;          //Du lai rd 2 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì
//reg             isRAW_Hazardrs1_1cyc_forWB =   VALUE_RESET;    //phat hien hazard 1 chu kì
//reg             isRAW_Hazardrs2_1cyc_forWB =   VALUE_RESET;    //phat hien hazard 1 chu kì
//reg             isRAW_Hazardrs1_2cyc_forWB =   VALUE_RESET;    //phat hien hazard 2 chu kì
//reg             isRAW_Hazardrs2_2cyc_forWB =   VALUE_RESET;    //phat hien hazard 2 chu kì



wire            isRAW_Hazardrs1_1cyc_forSTORE;  //Phat hien hazard 1 chu kì cho store
wire            isRAW_Hazardrs2_1cyc_forSTORE;  //Phat hien hazard 1 chu kì cho store
wire            isRAW_Hazardrs1_2cyc_forSTORE;  //Phat hien hazard 2 chu kì cho store
wire            isRAW_Hazardrs2_2cyc_forSTORE;  //Phat hien hazard 2 chu kì cho store
wire            isRAW_Hazardrs1_3cyc_forSTORE;  //Phat hien hazard 2 chu kì cho store
wire            isRAW_Hazardrs2_3cyc_forSTORE;  //Phat hien hazard 2 chu kì cho store

wire            insJALRpred;
wire            isRAW_Hazardrs1_1cyc_forJALR;
//wire            isRAW_Hazardrs1_2cyc_forJALR;   //Vi JALR su du data rs1
wire            isRAW_Hazardrs1_3cyc_forJALR;
wire            isRAW_Hazardrs1_4cyc_forJALR;

reg             en_pause = 1'b1;
reg             en_pauseshift1 = 1'b1;

reg             en_rs1pred_shift1   =   VALUE_RESET;
reg     [ 4:0 ] regrs1pred_shift1   =   5'h00;
reg     [31:0 ] resultfast          =   VALUE_RESET32;

wire            wait_peri           =   !s0_sel_mem && (!peri_trans_done) && (insLOAD || insSTORE) && PC != 32'h0 && !wrong_pred && !wrong_predshift1;
reg             IRQ_UpdatePC        =   VALUE_RESET;    //Cờ báo hiệu chuẩn bị Update PC do trap xảy ra
                                                        //Sử dụng để tránh trường hợp vừa ngắt xong thì phát hiện rẽ nhánh sai lại nhảy ra khỏi ngắt
reg             IRQ_UpdatePCshift1  =   VALUE_RESET;
reg             IRQ_UpdatePCshift2  =   VALUE_RESET;
reg     [31:0 ] PCnext_actual_shift1=   VALUE_RESET32;
reg     [31:0 ] PCnext_returnIRQ    =   VALUE_RESET32;
reg             mem_renablepredff   =   VALUE_RESET;


assign          wait_peri_as        =   wait_peri;
assign          mem_renablepred =   mem_renablepredff;
always @(*) begin
    if(!rst) mem_renablepredff <= 1'b1;
    else mem_renablepredff = !isRAW_Hazardrs1_2cyc_forJALR && !wait_peri;
end

//Xử lý hazard nếu dữ liệu phụ thuộc không đến từ lệnh load
    assign isRAW_Hazardrs1_1cyc_forWB   =   (en_rdhz1&&en_rs1)? (regrs1 == regrd_shifthz1)  &&
                                                                (regrd_shifthz1 != 5'h00)   
                                                                :1'b0; 
    assign isRAW_Hazardrs2_1cyc_forWB   =   (en_rdhz1&&en_rs2)? (regrs2 == regrd_shifthz1)  &&
                                                                (regrd_shifthz1 != 5'h00)   
                                                                :1'b0;
    assign isRAW_Hazardrs1_2cyc_forWB   =   (en_rdhz2&&en_rs1)? (regrs1 == regrd_shifthz2)  &&
                                                                (regrd_shifthz2 != 5'h00)   
                                                                :1'b0;
    assign isRAW_Hazardrs2_2cyc_forWB   =   (en_rdhz2&&en_rs2)? (regrs2 == regrd_shifthz2)  &&
                                                                (regrd_shifthz2 != 5'h00)   
                                                                :1'b0;
    assign isRAW_Hazardrs1_3cyc_forWB   =   (en_rdhz3&&en_rs1)? (regrs1 == regrd_shifthz3)  &&
                                                                (regrd_shifthz3 != 5'h00)  
                                                                :1'b0; 
    assign isRAW_Hazardrs2_3cyc_forWB   =   (en_rdhz3&&en_rs2)? (regrs2 == regrd_shifthz3)  &&
                                                                (regrd_shifthz3 != 5'h00)
                                                                :1'b0; 


    assign isRAW_Hazardrs1_1cyc_forSTORE=   isRAW_Hazardrs1_1cyc_forWB;
    assign isRAW_Hazardrs2_1cyc_forSTORE=   isRAW_Hazardrs2_1cyc_forWB;
    assign isRAW_Hazardrs1_2cyc_forSTORE=   isRAW_Hazardrs1_2cyc_forWB;
    assign isRAW_Hazardrs2_2cyc_forSTORE=   isRAW_Hazardrs2_2cyc_forWB;
    assign isRAW_Hazardrs1_3cyc_forSTORE=   isRAW_Hazardrs1_3cyc_forWB;
    assign isRAW_Hazardrs2_3cyc_forSTORE=   isRAW_Hazardrs2_3cyc_forWB;

    
    assign isRAW_Hazardrs1_4cyc_forJALR =   insJALRpred && (regrs1pred == regrd_shifthz2)   && (en_rdhz2&&en_rs1pred);
    assign isRAW_Hazardrs1_3cyc_forJALR =   insJALRpred && (regrs1pred == regrd_shifthz1)   && (en_rdhz1&&en_rs1pred);
    assign isRAW_Hazardrs1_2cyc_forJALR =   insJALRpred && (regrs1pred == regrd)            && (en_rd&&en_rs1pred) && en_pause;
    assign isRAW_Hazardrs1_1cyc_forJALR =   insJALRpred && (regrs1pred_shift1 == regrd)     && (en_rd&&en_rs1pred_shift1);//cần test cái này. 3cyc và 4 cyc cần xem sét dịch regrd cơ

    assign resulthz_1cyc                =   result;
    assign resulthz_2cyc                =   result_shiftpl;
    assign resulthz_3cyc                =   result_shiftpl1;


`ifdef DEBUG_EN
    
`endif



/*-------------------------------------------------------------------------
This module implements a 2-bit bimodal branch predictor using a 256-entry table of saturating counters.
It predicts branch direction based on the most significant bit (MSB) of each counter. On each clock cy-
-cle, the predictor updates its state according to the actual branch outcome if update_en is high.
-------------------------------------------------------------------------*/
reg             update_BHT      =   VALUE_RESET;
reg             actual_taken    =   VALUE_RESET;
wire            predict_taken;  
wire            insBRApred; 
reg     [31:0 ] pc_pred_in      =   VALUE_RESET32;
reg             enUpdate        =   VALUE_RESET;
wire    [31:0 ] Instruc_bpre    =   (enUpdate)?choose2_3:PCnext + 4;
branch_predictor BRA_PRED(
    .clk             (clksys),
    .rst             (rst), 
    .insBRA          (insBRApred),
    .pc_pred_in      (pc_pred_in),           //Dư đoán lệnh kế tiếp 
    .Instruc_bpre    (Instruc_bpre),
    .update_en       (update_BHT),    
    .actual_taken    (actual_taken),
    .predict_taken   (predict_taken)   //0: not take, 1: take
);

/*-------------
Decoder for predictor
-------------*/
wire            insJALpred;
wire            insMRETpred;
//wire            insJALRpred;
wire    [ 4:0 ] regrdpred;
wire    [31:0 ] Immediatepred;
RV32_Decoder Decoder_for_pred(
    .instr_data         (memins_rdata_pred),
    .Immediate          (Immediatepred),
    .insJAL             (insJALpred),
    .insJALR            (insJALRpred),
    .insBRA             (insBRApred),
    .insMRET            (insMRETpred),
    .regrs1             (regrs1pred),
    .en_rs1             (en_rs1pred)
    //.regrd              (regrdpred)
);
 
//ALU_unit ALU(
//    .isALUimm   (insALUImmshift1),
//    .isALUreg   (insALURegshift1),
//    .isBranch   (insBRAshift1),
//    .funct3oh   (funct3oh),
//    .funct7     (funct7),
//    .rs1        (data_rs1fn),
//    .rs2        (data_rs2fn),
//    .result     (result_ALU),
//    .correct    (flag_branch)
//);


/*----------------------------------------------------------
    1. IF (Instruction Fetch): Fetches the instruction from instruction memory.
    2. ID (Instruction Decode): Decodes the instruction and reads operands from registers.
    3. EX (Execute): Performs ALU operations or computes branch targets.
    4. MEM (Memory Access): Accesses data memory for load/store instructions.
    5. WB (Write Back): Writes results back to the register file.
----------------------------------------------------------*/
//----------------------------FETCH---------------------------//

//Bật cờ ngắt
reg irq_flagh = 0;
reg irq_active = 0;
    always @(posedge clksys) begin
        if(!rst) begin
            irq_flagh <= 1'b0;
        end
        else begin
            if(irq_flag) irq_flagh <= 1'b1;
            if(irq_active) irq_flagh <= 1'b0;
        end
    end

    always @(*) begin
        if(!rst) begin
            PCnext_returnIRQ = INSTR_FIRST + 4;
        end
        else begin
            PCnext_returnIRQ = PCnext + 4;
            (*parallel_case*)
            case(1'b1) 
                insJALpred: begin
                    PCnext_returnIRQ = PCnext + Immediatepred;

                end
                insJALRpred: begin
                    PCnext_returnIRQ = (((!en_pause)?((insLOAD)?mem_ldmask:result)                                       :
                              (isRAW_Hazardrs1_3cyc_forJALR)?((insLOADshift1)?mem_ldmask:result)                :
                              (isRAW_Hazardrs1_4cyc_forJALR)?((insLOADshift2)?mem_ldmaskshift1:result_shiftpl)  :
                               data_rs1pred) + Immediatepred) & ~32'h1;

                end
                insBRApred: begin
                    PCnext_returnIRQ = (predict_taken)?PCnext + Immediatepred:PCnext+4;
                end
            endcase
            if(wrong_predfast) begin
                PCnext_returnIRQ = PCnext_actual;
            end
        end
    end
    
    always @(*) begin
        if(!rst) begin
            PCnext_fast = INSTR_FIRST + 4;
        end
        else begin
            PCnext_fast = PCnext_returnIRQ;

            if(wrong_predfast) begin
                if (!IRQ_UpdatePC) PCnext_fast = PCnext_actual;
            end

            if(irq_flagh && !irq_active && !insBRApred) PCnext_fast = csr_trap_addr; 

            if(insMRETpred) PCnext_fast = csr_trap_rpc;
        end
    end
    
    assign memins_addr = PC;
    assign memins_read = 1'b1;          //Luôn đọc
    always @(posedge clksys) begin
        if(!rst) begin 
            PC <= INSTR_FIRST;
            PCnext <= INSTR_FIRST;
            PCshift1 <= INSTR_FIRST;
            PCnext_actual <= INSTR_FIRST;
            PCnext_actual_shift1 <= INSTR_FIRST;
            Fetchena <= VALUE_RESET;
            pc_pred_in <= VALUE_RESET32;
            IDen <= VALUE_RESET;
            update_BHT <= VALUE_RESET;
            actual_taken <= VALUE_RESET;
            predict_taken1 <= VALUE_RESET;
            predict_taken2 <= VALUE_RESET;
            predict_taken3 <= VALUE_RESET;
            en_pause <= 1'b1;
            en_pauseshift1 <= 1'b1;
            IRQ_UpdatePC <= VALUE_RESET;
            UpdateMEPC <= VALUE_RESET;
            
            choose1_1 <= VALUE_RESET32; choose1_2 <= VALUE_RESET32;
            choose2_1 <= VALUE_RESET32; choose2_2 <= VALUE_RESET32;
        end
        else if(!isRAW_Hazardrs1_2cyc_forJALR && !wait_peri) begin
            PC <= PCnext;   PCshift1 <= PC;
            Fetchena <= 1'b1;
            en_pause <= 1'b1;
            en_pauseshift1 <= en_pause;
            PCnext <= PCnext_fast;
            PCnext_actual <= (predict_taken1)?choose2_1+4:choose1_1;
            PCnext_actual_shift1 <= PCnext_actual;
            pc_pred_in <= PCnext + 4;
            update_BHT <= 1'b0;

//            (*parallel_case*)
//            case(1'b1) 
//                insJALpred: begin
//                    PCnext <= PCnext + Immediatepred;
//                end
//                insJALRpred: begin
//                    PCnext <= (((!en_pause)?((insLOAD)?mem_ldmask:result)                                       ://isRAW_Hazardrs1_2cyc_forJALR
//                              (isRAW_Hazardrs1_3cyc_forJALR)?((insLOADshift1)?mem_ldmask:result)                :
//                              (isRAW_Hazardrs1_4cyc_forJALR)?((insLOADshift2)?mem_ldmaskshift1:result_shiftpl)  :
//                               data_rs1pred) + Immediatepred) & ~32'h1;
//                end
//                insBRApred: begin
//                    PCnext <= (predict_taken)?PCnext + Immediatepred:PCnext+4;
//                end
//            endcase 

            UpdateMEPC <= 1'b0;
//            if(wrong_predfast) begin
//                if (!IRQ_UpdatePC) PCnext <= PCnext_actual;
//            end

            if(IRQ_UpdatePC) begin        //Update lại mepc chính xác do rẽ nhánh sai.
                if(wrong_predfast) begin
                    csr_trap_pc <= PCnext_actual;
                    UpdateMEPC  <= 1'b1;
                end
                if(wrong_pred) begin
                    UpdateMEPC  <= 1'b1;
                    csr_trap_pc <= PCnext_actual_shift1;
                end
            end

            //Trap handler
            csr_trap_taken <= 1'b0;
            IRQ_UpdatePC <= 1'b0;
            IRQ_UpdatePCshift1 <= IRQ_UpdatePC;
            IRQ_UpdatePCshift2 <= IRQ_UpdatePCshift1;
            if(irq_flagh && !irq_active && !insBRApred) begin //Nếu lệnh PCnext trước đó là branch sẽ không ngắt
                irq_active <= 1'b1;
//                PCnext <= csr_trap_addr; 
                csr_trap_taken <= 1'b1;
                csr_trap_pc <= PCnext_returnIRQ;
                csr_trap_cause <= 32'h8000000B;

                IRQ_UpdatePC <= 1'b1;
            end
            if(insMRETpred) begin
//                PCnext <= csr_trap_rpc;
                irq_active <= 1'b0;
            end

            if(enUpdate) begin
                //Update BHT
                update_BHT <= 1'b1;
                actual_taken <= (wrong_pred)?!predict_taken3:predict_taken3;
                pc_pred_in <= choose2_3;                          //Địa chỉ lệnh branch
            end

            choose1_1 <= PCnext + Immediatepred;    choose1_2 <= choose1_1; choose1_3 <= choose1_2;
            choose2_1 <= PCnext;                    choose2_2 <= choose2_1; choose2_3 <= choose2_2;

            IDen <= 1'b1;
            predict_taken1 <= predict_taken;    
            predict_taken2 <= predict_taken1;
            predict_taken3 <= predict_taken2;
        end
        else begin
            if (isRAW_Hazardrs1_2cyc_forJALR) en_pause <= 1'b0;
            en_pauseshift1 <= en_pause;
        end
    end


//----------------------------DECODE---------------------------//
    reg [31:0] memins_rdata_ff = VALUE_RESET32;
    always @(posedge clk) begin
        memins_rdata_ff <= memins_rdata;
    end
    always @(*) begin
        instr_data = memins_rdata;
    end
    
    always @(posedge clk) begin
        if(!rst || !IDen) begin
            funct3shift1 <= 3'h0;
        end
        else begin
            funct3shift1 <= funct3; 
            if(wrong_pred || holdplus_forDE) funct3shift1 <= funct3shift1;
        end
    end

    always @(posedge clksys or negedge IDen) begin
        if (!rst || !IDen) begin
            EXen <= 1'b0;

            insALUImmshift1 <= 1'b0;    insALURegshift1 <= 1'b0;    insLUIshift1    <= 1'b0;
            insAUIPCshift1  <= 1'b0;    insJALshift1    <= 1'b0;    insJALRshift1   <= 1'b0;
            insBRAshift1    <= 1'b0;    insLOADshift1   <= 1'b0;    insSTOREshift1  <= 1'b0;
            insSYSshift1    <= 1'b0;    insFENCEshift1  <= 1'b0;

            insALUImmshift2 <= 1'b0;    insALURegshift2 <= 1'b0;    insLUIshift2    <= 1'b0;
            insAUIPCshift2  <= 1'b0;    insJALshift2    <= 1'b0;    insJALRshift2   <= 1'b0;
            insBRAshift2    <= 1'b0;    insLOADshift2   <= 1'b0;    insSTOREshift2  <= 1'b0;
            insSYSshift2    <= 1'b0;    insFENCEshift2  <= 1'b0;

            insLOADshift3   <= 1'b0;

            regrd_shiftpl1  <= 1'b0;    regrd_shiftpl2  <= 1'b0;    regrd_shiftpl3  <= 1'b0;
            regrd_shifthz1  <= 1'b0;    regrd_shifthz2  <= 1'b0;    regrd_shifthz3  <= 1'b0;
            en_rdhz1        <= 1'b0;    en_rdhz2        <= 1'b0;    en_rdhz3        <= 1'b0;

            regrs1pred_shift1<= 1'b0;
            en_rs1pred_shift1<= 1'b0;

            holdplus_forDE <= VALUE_RESET;
        end
        else if(!isRAW_Hazardrs1_2cyc_forJALR && !wait_peri) begin
            //instr_data <= memins_rdata;
            EXen <= 1'b1;
//            Store_datapl1 <= data_rs2;          //Lưu trễ 1 chu kì
//            Store_datapl2 <= Store_datapl1;     //Lưu trễ 2 chu kì để dành cho memory access


            regrd_shiftpl1 <= regrd;            //Lưu trễ 1 chu kì
            regrd_shiftpl2 <= regrd_shiftpl1;   //Lưu trẽ 2 chu kì
            regrd_shiftpl3 <= regrd_shiftpl2;   //Lưu trễ 3 chu kì     
         
            regrd_shifthz1 <= regrd;
            regrd_shifthz2 <= regrd_shifthz1;
            regrd_shifthz3 <= regrd_shifthz2;

            regrs1_shifthz1 <= regrs1;
            regrs1_shifthz2 <= regrs1_shifthz1;
            regrs1_shifthz3 <= regrs1_shifthz2;

            csr_addr_shift1 <= csr_addr;
            csr_addr_shift2 <= csr_addr_shift1;
            csr_addr_shift3 <= csr_addr_shift2;

            regrs1pred_shift1 <= regrs1pred;

            en_rdhz1 <= en_rd;
            en_rdhz2 <= en_rdhz1;
            en_rdhz3 <= en_rdhz2;

            en_rs1hz1 <= en_rs1;
            en_rs1hz2 <= en_rs1hz1;

            en_rs1pred_shift1 <= en_rs1pred;

            // Lưu trễ 1 chu kỳ 
            insALUImmshift1  <= insALUImm;      insALURegshift1  <= insALUReg;
            insLUIshift1     <= insLUI;         insAUIPCshift1   <= insAUIPC;
            insJALshift1     <= insJAL;         insJALRshift1    <= insJALR;
            insBRAshift1     <= insBRA;         insLOADshift1    <= insLOAD;
            insSTOREshift1   <= insSTORE;       insSYSshift1     <= insSYS;
            insFENCEshift1   <= insFENCE;
     

            // Lưu trễ 2 chu kỳ 
            insALUImmshift2  <= insALUImmshift1;insALURegshift2  <= insALURegshift1;
            insLUIshift2     <= insLUIshift1;   insAUIPCshift2   <= insAUIPCshift1;
            insJALshift2     <= insJALshift1;   insJALRshift2    <= insJALRshift1;
            insBRAshift2     <= insBRAshift1;   insLOADshift2    <= insLOADshift1;
            insSTOREshift2   <= insSTOREshift1; insSYSshift2     <= insSYSshift1;
            insFENCEshift2   <= insFENCEshift1;

            //Lưu trễ 3 chu kì
            insLOADshift3    <= insLOADshift2;

            holdplus_forDE <= 1'b0;
            if(wrong_pred || (holdplus_forDE&&!IRQ_UpdatePCshift2)) begin  //Nếu phát hiện dự đoán sai -> không lưu các thanh ghi 2 lệnh kế tiếp tính từ branch.
                regrd_shifthz1 <= regrd_shifthz1;
                regrd_shifthz2 <= regrd_shifthz2;
                regrd_shifthz3 <= regrd_shifthz3;
                regrs1pred_shift1 <= regrs1pred_shift1;

                regrs1_shifthz1 <= regrs1_shifthz1;
                regrs1_shifthz2 <= regrs1_shifthz2;
                regrs1_shifthz3 <= regrs1_shifthz3;

                csr_addr_shift1 <= csr_addr_shift1;
                csr_addr_shift2 <= csr_addr_shift2;
                csr_addr_shift3 <= csr_addr_shift3;

                en_rdhz1 <= en_rdhz1;
                en_rdhz2 <= en_rdhz2;
                en_rdhz3 <= en_rdhz3;

                en_rs1hz1 <= en_rs1hz1;
                en_rs1hz2 <= en_rs1hz2;
                en_rs1pred_shift1 <= en_rs1pred_shift1;

                // Lưu trễ 1 chu kỳ (giữ nguyên chính nó)
                insALUImmshift1  <= insALUImmshift1;
                insALURegshift1  <= insALURegshift1;
                insLUIshift1     <= insLUIshift1;
                insAUIPCshift1   <= insAUIPCshift1;
                insJALshift1     <= insJALshift1;
                insJALRshift1    <= insJALRshift1;
                insBRAshift1     <= insBRAshift1;
                insLOADshift1    <= insLOADshift1;
                insSTOREshift1   <= insSTOREshift1;
                insSYSshift1     <= insSYSshift1;
                insFENCEshift1   <= insFENCEshift1;
                funct3shift1     <= funct3shift1;

                // Lưu trễ 2 chu kỳ (giữ nguyên chính nó)
                insALUImmshift2  <= insALUImmshift2;
                insALURegshift2  <= insALURegshift2;
                insLUIshift2     <= insLUIshift2;
                insAUIPCshift2   <= insAUIPCshift2;
                insJALshift2     <= insJALshift2;
                insJALRshift2    <= insJALRshift2;
                insBRAshift2     <= insBRAshift2;
                insLOADshift2    <= insLOADshift2;
                insSTOREshift2   <= insSTOREshift2;
                insSYSshift2     <= insSYSshift2;
                insFENCEshift2   <= insFENCEshift2;

                // Lưu trễ 3 chu kỳ (giữ nguyên chính nó)
                insLOADshift3    <= insLOADshift3;
                
                if(wrong_pred) holdplus_forDE <= 1'b1;
            end  


//            isRAW_Hazardrs1_1cyc_forWB <= (en_rdhz1&&en_rs1)?(regrs1 == regrd_shifthz1):1'b0;
//            isRAW_Hazardrs2_1cyc_forWB <= (en_rdhz1&&en_rs2)?(regrs2 == regrd_shifthz1):1'b0;
//            isRAW_Hazardrs1_2cyc_forWB <= (en_rdhz2&&en_rs1)?(regrs1 == regrd_shifthz2):1'b0;
//            isRAW_Hazardrs2_2cyc_forWB <= (en_rdhz2&&en_rs2)?(regrs2 == regrd_shifthz2):1'b0;

            if(wrong_pred && !IRQ_UpdatePCshift1) begin
                EXen <= 1'b0;                                       //Lập lại pipeline, xóa kết quả cũ
            end
        end
    end


//----------------------------EXECUTE---------------------------//
    always @(*) begin//-> giúp load và store sớm hơn 1 chu kì
        if(!rst || !EXen) begin
            mem_addr = VALUE_RESET32;
            memd_sdata = VALUE_RESET32;
            memd_lready = VALUE_RESET;
            memd_senable = VALUE_RESET;
        end
        else begin
            mem_addr = VALUE_RESET32;
            memd_sdata = VALUE_RESET32;
            memd_lready = VALUE_RESET;
            memd_senable = VALUE_RESET;
            (*parallel_case*)
            case(1'b1)
                insLOAD: begin
                    mem_addr =      (isRAW_Hazardrs1_1cyc_forSTORE)?((insLOADshift1)?mem_ldmask + Immediate:result + Immediate)                 :    //Xư lý cả hazard của store nếu trước đó là lệnh load
                                    (isRAW_Hazardrs1_2cyc_forSTORE)?((insLOADshift2)?mem_ldmaskshift1 + Immediate:result_shiftpl + Immediate)   :
                                    (isRAW_Hazardrs1_3cyc_forSTORE)?((insLOADshift3)?mem_ldmaskshift2 + Immediate:result_shiftpl1 + Immediate)  :
                                                                      data_rs1 + Immediate;
                    memd_lready = 1'b1 && !wrong_pred;
                end
                insSTORE: begin
                    mem_addr =      (isRAW_Hazardrs1_1cyc_forSTORE)?((insLOADshift1)?mem_ldmask + Immediate:result + Immediate)                 :    //Xư lý cả hazard của store nếu trước đó là lệnh load
                                    (isRAW_Hazardrs1_2cyc_forSTORE)?((insLOADshift2)?mem_ldmaskshift1 + Immediate:result_shiftpl + Immediate)   :
                                    (isRAW_Hazardrs1_3cyc_forSTORE)?((insLOADshift3)?mem_ldmaskshift2 + Immediate:result_shiftpl1 + Immediate)  :
                                                                      data_rs1 + Immediate;
                    memd_sdata =    (isRAW_Hazardrs2_1cyc_forSTORE)?((insLOADshift1)?mem_ldmask:result)                 :
                                    (isRAW_Hazardrs2_2cyc_forSTORE)?((insLOADshift2)?mem_ldmaskshift1:result_shiftpl)   :
                                    (isRAW_Hazardrs2_3cyc_forSTORE)?((insLOADshift3)?mem_ldmaskshift2:result_shiftpl1)  :
                                                                      data_rs2;
                    memd_senable = 1'b1 && !wrong_pred;
                end
                default: begin
                    memd_lready = 1'b0;
                    memd_senable = 1'b0;
                end
            endcase
        end
    end

/////////////////Max frequecy giảm mạnh từ 27 -> 23////////////////////
//-> Chấp nhận stall thêm 1 chu kì sẽ có lời hơn rất nhiều
//    always @(*) begin//tính toán kết quả nhanh sớm 1 chu kì để sử dụng cho HZ JALR giúp nếu hz jalr 2cyc sẽ không cần stall và 1cyc chỉ stall 1 chu kì.
//        if(!rst || !EXen) begin
//            resultfast = VALUE_RESET32;
//        end
//        else begin
//            resultfast = VALUE_RESET32;
//            (*parallel_case*)
//            case(1'b1)
//                insALU: begin
//                    resultfast = result_ALU;
//                end
//                insLUI: begin
//                    resultfast = Immediate;
//                end
//                insAUIPC: begin 
//                    resultfast = PCshift1 + Immediate;
//                end
//                insJAL: begin
//                    resultfast = PCshift1 + 4;                 
//                end
//                insJALR: begin
//                    resultfast = PCshift1 + 4; 
//                end
//                default: resultfast = resultfast;
//            endcase
//        end
//    end

    always @(posedge clksys) begin
        if(!rst) begin
            wrong_predshift1 <= VALUE_RESET;
        end
        else begin
            wrong_predshift1 <= wrong_pred;
        end
    end

    always @(posedge clksys or negedge EXen) begin
        if(!rst || !EXen) begin
            MEMen <= 1'b0;
            wrong_pred <= VALUE_RESET;
            PCBraoJum <= VALUE_RESET;
            mem_addrup <= VALUE_RESET32;
            result <= VALUE_RESET32;
//            result_shiftpl <= VALUE_RESET32;
//            result_shiftpl1 <= VALUE_RESET32;
            holdplus_forEX <= VALUE_RESET;
        end
        else if(!isRAW_Hazardrs1_2cyc_forJALR && !wait_peri) begin
            wrong_pred <= wrong_predfast; enUpdate <= 1'b0;
//            memd_lready <= 1'b0;
//            memd_senable <= 1'b0;
            (*parallel_case*)
            case(1'b1)
                insBRA: begin
                    //PCnext <= flag_branch ? PCshift1 + Immediate : PCshift1+4; //was predicted
//                    if(flag_branch != predict_taken2) begin
//                        wrong_pred <= 1'b1;
//                    end
                    enUpdate <= 1'b1;
                end
//                insLOAD: begin
//                    mem_addr <= (isRAW_Hazardrs1_1cyc_forSTORE)?result + Immediate:
//                                  (isRAW_Hazardrs1_2cyc_forSTORE)?result_shiftpl + Immediate:
//                                                                  data_rs1 + Immediate;
//                    memd_lready <= 1'b1;
//                end
//                insSTORE: begin
//                    mem_addr <=   (isRAW_Hazardrs1_1cyc_forSTORE)?result + Immediate:
//                                  (isRAW_Hazardrs1_2cyc_forSTORE)?result_shiftpl + Immediate:
//                                                                  data_rs1 + Immediate;
//                    memd_sdata <=    (isRAW_Hazardrs2_1cyc_forSTORE)?result:
//                                     (isRAW_Hazardrs2_2cyc_forSTORE)?result_shiftpl:
//                                                                     data_rs2;
//                    memd_senable <= 1'b1;
//                end
                insALU: begin
                    result <= result_ALU;
                end
                insLUI: begin
                    result <= Immediate;
                end
                insAUIPC: begin 
                    result <= PCshift1 + Immediate;
                end
                insJAL: begin
                    result <= PCshift1 + 4;                 
                    //PCnext <= PCshift1 + Immediate;         //was predicted
                end
                insJALR: begin
                    result <= PCshift1 + 4;               
                    //PCnext <= data_rs1 + Immediate;   //was predicted
                end
                insSYS: begin
                    result <= csr_rdata;
                    dwb_csr <= result_ALU;
                end
            endcase

            result_shiftpl <= result;
            result_shiftpl1 <= result_shiftpl;

            dwb_csr_shiftpl <= dwb_csr;
            dwb_csr_shiftpl1 <= dwb_csr_shiftpl;

            holdplus_forEX <= 1'b0;
            if((wrong_pred || (holdplus_forEX&&!IRQ_UpdatePCshift2))) begin  //Nếu phát hiện dự đoán sai -> dữ lại kết quả cũ 2 chu kì nữa
                result_shiftpl <= result_shiftpl;
                result_shiftpl1 <= result_shiftpl1;

                dwb_csr_shiftpl <= dwb_csr_shiftpl;
                dwb_csr_shiftpl1 <= dwb_csr_shiftpl1;
                if(wrong_pred) holdplus_forEX <= 1'b1;
            end 

            MEMen <= 1'b1;
            if(wrong_pred && IRQ_UpdatePCshift1) begin
                MEMen <= 1'b0;
            end
        end
    end


//----------------------------MEMORY ACCESS---------------------------//
    always @(posedge clksys or negedge MEMen) begin
        if(!rst || !MEMen) begin
            WBen <= VALUE_RESET;
            holdplus_forMEM <= VALUE_RESET;
//            mem_ldmaskshift1 <= VALUE_RESET32;
//            mem_ldmaskshift2 <= VALUE_RESET32;
//            memd_sdata <= VALUE_RESET32;
        end
        else if(!isRAW_Hazardrs1_2cyc_forJALR && !wait_peri) begin                
            mem_ldmaskshift1 <= mem_ldmask;
            mem_ldmaskshift2 <= mem_ldmaskshift1;

            holdplus_forMEM <= 1'b0;
            csr_instret <= csr_instret + 1;
            if(wrong_pred || holdplus_forMEM) begin  //Nếu phát hiện dự đoán sai -> dư lại kêt quả cũ 2 chu kì nữa
                mem_ldmaskshift1 <= mem_ldmaskshift1;
                mem_ldmaskshift2 <= mem_ldmaskshift2;
                if(wrong_pred) holdplus_forMEM <= 1'b1;
                csr_instret <= csr_instret;
            end 
//            if(insLOADshift1) begin
                //mem_addr <= mem_addrup;
                //memd_lready <= 1'b1;
//            end
//            else if(insSTOREshift1) begin
                //memd_sdata <= Store_datapl1;
                //mem_addr <= mem_addrup;
                //memd_senable <= 1'b1;
//            end
//            else begin
//                memd_lready <= 1'b0;
//                memd_senable <= 1'b0;
//            end
            WBen <= 1'b1;
        end
    end

//38535601 tại đây do dự đoán sai nên lệnh thực thi tại đó không được thực hiện. hay nói cách khác địa chỉ nhảy tới khi ngắt 188 bị vô hiệu hóa bởi lệnh trước đó nữa bị dự đoán sai.
//----------------------------WRITE BACK---------------------------//
    always @(posedge clksys) begin//sử dụng giá trị cũ của WBen
        if(!rst || !WBen) begin  //cho phep ghi nốt lệnh trước
            data_des <= VALUE_RESET32;
            regrd_shiftpl <= 5'h00;
            data_valid <= VALUE_RESET;
        end
        else if(MEMen || (!isRAW_Hazardrs1_2cyc_forJALR && !wait_peri)) begin
            case(1'b1)
                insLOADshift2: begin
                    data_des <= (en_pauseshift1)?mem_ldmaskshift1:mem_ldmaskshift2;
                    regrd_shiftpl <= regrd_shiftpl2;
                    data_valid <= 1'b1;
                end
                insALUImmshift2 | insALURegshift2 | insLUIshift2 | 
                insAUIPCshift2  | insJALshift2    | insJALRshift2   : begin
                    data_des <= result_shiftpl;
                    regrd_shiftpl <= regrd_shiftpl2;
                    data_valid <= 1'b1;
                end
                insSYSshift2: begin
                    data_des <= result_shiftpl;
                    regrd_shiftpl <= regrd_shiftpl2;
                    data_valid <= 1'b1;

                    csr_addrpl <= csr_addr_shift2;
                    csr_we <= !(regrs1_shifthz2 == 5'h00 && en_rs1hz2);
                    csr_wdata <= dwb_csr_shiftpl;
                end
                default:begin
                    data_valid <= 1'b0;
                    csr_we <= 1'b0;
                end
            endcase
        end
    end
`endif

endmodule