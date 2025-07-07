module ALU_unit(
    input           rst,
    input           clk,

    input           isALUimm,
    input           isALUreg,
    input           isBranch,
    input   [3:0]   funct3b,
    input   [7:0]   funct3oh,
    input   [6:0]   funct7,
    input   [31:0]  rs1,
    input   [31:0]  rs2,        //Imm 
    
    output  [31:0]  result
);

//ALU 
wire    isALU   =   isALUimm|   isALUreg;
wire    ADD     =   isALU   &   funct3oh[0] &   !funct7[5]; //ADDI or ADD
wire    SUB     =   isALUreg&   funct3oh[0] &   funct7[5];  //SUB
wire    AND     =   isALU   &   funct3oh[7];                //ANDI or AND
wire    OR      =   isALU   &   funct3oh[6];                //ORI or OR
wire    XOR     =   isALU   &   funct3oh[4];                //XORI or XOR
wire    SLL     =   isALU   &   funct3oh[1];                //SLLI or SLL
wire    SRL     =   isALU   &   funct3oh[5] &   !funct7[5]; //SRLI or SRL
wire    SRA     =   isALU   &   funct3oh[5] &   funct7[5];  //SRAI or SRA
wire    SLT     =   isALU   &   funct3oh[2];                //SLTI or SLT
wire    SLTIU   =   isALU   &   funct3oh[3];                //SLTIU or SLTU

//Compare
wire    CP  =   rs1 < rs2;
wire    CS  =   (rs1[31] ^ rs2[31])?rs1[31]:CP;//Compare signed; 1 = LT; 0 = LT
wire    EQ  =   rs1 == rs2;
wire    LT  =   isSigned?CS:CP;
wire    GE  =   !LT;

//correct branch
assign  correct =    
            funct3b[0]  &   EQ  |
            funct3b[1]  &  !EQ  |
            funct3b[2]  &   LT  |
            funct3b[3]  &   GE;

//result
assign  result  =   ADD ?   in1 +   in2:
                    SUB ?   in1 -   in2:
                    AND ?   in1 &   in1:
                    OR  ?   in1 |   in2:
                    XOR ?   in1 ^   in2:
                    SLL ?   in1 <<  in2:
                    SRL ?   in1 >>  in2:
                    SRA ?   
                    SLT ?   (CS?32'd1:32'd0):
                    SLTU?   (CP?32'd1:32'd0);        

endmodule