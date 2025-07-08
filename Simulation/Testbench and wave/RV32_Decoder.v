/*
Integer Computational Instructions
    Integer Register-Immediate Instructions:
        ADDI        Add immediate                               (rd = rs1 + sign_extend(imm))
        SLTI        set less than immediate                     (rd = 1 if rs1 < imm, signed)
        SLTIU       set Less Than Immediate (Unsigned)          (rd = 1 if rs1 < imm, unsigned)
        ANDI        logical operation - AND                     (rd = rs1 & sign_extend(imm))
        ORI, XORI   //        //          //      
        SLLI        Shift Left Logical Immediate                (rd = rs1 << shamt)
        SRLI        Shift Right Logical Immediate               (rd = rs1 >> shamt)
        SRAI        Shift Right Arithmetic                      (rd = rs1 >>> shamt (arithmetic, signed))
        LUI         Load upper immediate                        (rd = Uimm)
        AUIPC       Add Upper Immediate to PC                   (rd = PC + Uimm)
    Integer Register-Register Operations:
        ADD/SLT/SLTU/AND/OR/XOR/SLL/SRL/SRA
Control Transfer Instructions
    Unconditional Jumps:
        JAL         Jump and link                               (PC = PC + Jimm, rd = PC+4) x1 - return address register, x5 - alternate link register
        JALR        Jump and link register                      (PC = rs1 + Iimm, rd = PC+4) x0 = rd if not required
    Conditional Branches:
        BEQ         Branch if Equal                                                 
        BNE         Branch if Not Equal
        BLT/BLTU    //              //
        BGE/BGEU    //              //
Load and Store Instructions
    Load Instructions:
        LW          Load Word                                   (rd = MEM[rs1 + imm])
        LH          Load Halfword                               (rd = sign_extend_32(MEM[rs1 + imm][15:0])
        LHU         Load Halfword Unsigned                      (rd = zero_extend_32(MEM[rs1 + imm][15:0]))
        LB          Load Byte                                   (rd = sign_extend_32(MEM[rs1 + imm][7:0]))
        LBU         Load Byte Unsigned                          (rd = zero_extend_32(MEM[rs1 + imm][7:0]))
    Store Instructions:
        SW          Store Word                                  (MEM[rs1 + imm] = rs2)
        SH          Store Halfword                              (MEM[rs1 + imm] = rs2[15:0])
        SB          Store Byte                                  (MEM[rs1 + imm] = rs2[7:0])
Memory Model
        FENCE
        FENCE.I
Control and Status Register Instructions
    CSR Instructions:
        CSRRW       Atomic Read/Write CSR                       (rd = CSR, CSR = rs1) if rd = xo, not read CSR
        CSRRS       Atomic Read and Set Bits in CSR             (rd = CSR, Setbit CSR, rs1 - bitmask) if rs1 = x0, not write CSR
        CSRRC       Atomic Read and Clear Bits in CSR           (//     // Clearbit  //      //      //  )
        CSRRWI      Atomic Read/Write CSR Immediate             (rd = CSR, CSR = imm(unsigned)) if rd = xo, not read CSR
        CSRRSI      //              //          //                  //          //
        CSRRCI      //              //          //                  //          //
    Timers and Counters: CSRRS - RDCYCLE[H](count of the number of clock cycles executed by the processor core) cycle CSR
                                 RDTIME[H](wall-clock real time that has passed from an arbitrary start time in the past.) time CSR
                                 RDINSTRET[H](instructions retired by this hart from some arbitrary start point in the past) instret CSR
Environment Call and Breakpoints
        ECALL       Environment Call
        EBREAK      Environment Break
*/

module RV32_Decoder(
    input           rst,
    input           clk,

//Instruction
    input   [31:0]  instr_data,
    
//decode comm
    output  [31:0]  Immediate,
    output          insALUImm,insALUReg,insLUI,insAUIPC,insJAL,insJALR,insBRA,insLOAD,insSTORE,insSYS,insFENCE,
    output  [2:0]   funct3,
    output  [7:0]   funct3oh,
    output  [3:0]   funct3b,
    output  [6:0]   funct7,
    output  [4:0]   regrs2, //Shamt
    output  [4:0]   regrs1, //Zimm
    output  [4:0]   regrd,

//decode CSR
    output  [3:0]   pred,
    output  [3:0]   succ,
    output  [11:0]  indcsr,

//branch
    output          isSigned
);

/*
Rtype - Register:           add, sub, and, or, sll          (rd = rs1 op rs2)
Itype - Immediate:          addi, lw, jalr, ecall, csrrw    (rd = rs1 op imm)
Stype - Store:              sw, sh, sb                      (RAM[rs1 + offset] = rs2)
Btype - Branch:             beq, bne, blt, bge              (condition PC = PC + offset)
Utype - Upper immediate:    lui, auipc                      (rd = imm << 12)
Jtype - Jump:               jal                             (rd = PC+4, PC = PC + offset)
*/
    
//Types of immediate. Sign extension always uses inst[31].
wire    [31:0]  Iimm    =   {{21{instr_data[31]}}, instr_data[30:20]};
wire    [31:0]  Simm    =   {{21{instr_data[31]}}, instr_data[30:25], instr_data[11:8], instr_data[7]}; 
wire    [31:0]  Bimm    =   {{20{instr_data[31]}}, instr_data[7], instr_data[30:25], instr_data[11:8], 1'b0};
wire    [31:0]  Uimm    =   {instr_data[31], instr_data[30:12], 12'd0};
wire    [31:0]  Jimm    =   {{12{instr_data[31]}}, instr_data[19:12], instr_data[20], instr_data[30:21], 1'b0};


wire    [31:0]  Imm     =   
                    (insLUI | insAUIPC) ?   Uimm:   
                    (insJAL)            ?   Jimm:
                    (insJALR|insLOAD|insALUImm|insFENCE|insSYS) ?   Iimm:
                    (insBRA)            ?   Bimm:
                    (insALUReg)         ?   32'd0:   //Haven't imm
                    (insSTORE)          ?   Simm:32'd0;
assign Immediate = Imm;

//Instructions - opcode
assign  insALUImm  = (instr_data[6:2] == 5'b00100);// rd <- rs1 OP Iimm
assign  insALUReg  = (instr_data[6:2] == 5'b01100);// rd <- rs1 OP rs2
assign  insLUI     = (instr_data[6:2] == 5'b01101);// rd <- Uimm
assign  insAUIPC   = (instr_data[6:2] == 5'b00101);// rd <- PC + Uimm
assign  insJAL     = (instr_data[6:2] == 5'b11011);// rd <- PC+4; PC<-PC+Jimm
assign  insJALR    = (instr_data[6:2] == 5'b11001);// rd <- PC+4; PC<-rs1+Iimm
assign  insBRA     = (instr_data[6:2] == 5'b11000);// if(rs1 OP rs2) PC<-PC+Bimm
assign  insLOAD    = (instr_data[6:2] == 5'b00000);// rd <- mem[rs1+Iimm]
assign  insSTORE   = (instr_data[6:2] == 5'b01000);// mem[rs1+Simm] <- rs2
assign  insSYS     = (instr_data[6:2] == 5'b11100);
assign  insFENCE   = (instr_data[6:2] == 5'b00011);

/*          FUTURE
Return-Address Stack (RAS)
Control and Status Resgisters (CSRs)
ECALL and EBREAK
FENCE
*/

//funct3, funct7, rs2, rs1, rd, csr, shamt
assign  funct3oh=   8'b0000_0001 << instr_data[14:12];
assign  funct3  =   instr_data[14:12];
assign  funct3b =   4'b0001 << {instr_data[14], instr_data[12]};//Only BEQ, BNE, BLT, BGE
assign  funct7  =   instr_data[31:25];
assign  regrs2  =   instr_data[24:20];
assign  regrs1  =   instr_data[19:15];
assign  regrd   =   instr_data[11:7];

//wire    [4:0]   shamt   =   regrs2;
//wire    [4:0]   zimm    =   regrs1;

//FENCE: pred, succ; CSR;
assign  pred    =   instr_data[27:24];
assign  succ    =   instr_data[23:20];
assign  indcsr  =   instr_data[31:20];

//signed
assign  isSigned =   !instr_data[13]; 

endmodule
