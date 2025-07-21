module ALU_unit(

    input           isALUimm,
    input           isALUreg,
    input           isBranch,
    input   [ 7:0 ] funct3oh,
    input   [ 6:0 ] funct7,
    input   [31:0 ] rs1,
    input   [31:0 ] rs2,            //Imm 
    
    output  [31:0]  result,
    output          correct
);

//ALU 
wire    isALU   =   isALUimm|   isALUreg;

wire    SUB     =   isALUreg&   funct3oh[0] &   funct7[5];  //SUB
wire    ADD     =   isALU   &   funct3oh[0] &   !SUB;       //ADDI or ADD
wire    AND     =   isALU   &   funct3oh[7];                //ANDI or AND
wire    OR      =   isALU   &   funct3oh[6];                //ORI or OR
wire    XOR     =   isALU   &   funct3oh[4];                //XORI or XOR
wire    SLL     =   isALU   &   funct3oh[1];                //SLLI or SLL
wire    SRL     =   isALU   &   funct3oh[5] &   !funct7[5]; //SRLI or SRL
wire    SRA     =   isALU   &   funct3oh[5] &   funct7[5];  //SRAI or SRA
wire    SLT     =   isALU   &   funct3oh[2];                //SLTI or SLT
wire    SLTIU   =   isALU   &   funct3oh[3];                //SLTIU or SLTU

//Sign for compare
wire   isSigned    =   !(funct3oh[7] & funct3oh[6]);

//Compare
wire    CP  =   rs1 < rs2;                      //Compare unsigned
wire    CS  =  (rs1[31] ^ rs2[31])?rs1[31]:CP;  //Compare signed; 1 = LT;
wire    EQ  =   rs1 == rs2;                     //Equal
wire    LT  =   isSigned?CS:CP;                 //Less than
wire    GE  =   !LT;                            //Greater than

//correct branch
assign  correct =    
            funct3oh[0]             &   EQ  |   //is BEQ 
            funct3oh[1]             &  !EQ  |   //is BNE
           (funct3oh[4]|funct3oh[6])&   LT  |   //is BLT or BLTU
           (funct3oh[5]|funct3oh[7])&   GE;     //is BGE or BGEU

//result of ALU
assign  result  =   ADD ?   rs1 +   rs2:
                    SUB ?   rs1 -   rs2:
                    AND ?   rs1 &   rs2:
                    OR  ?   rs1 |   rs2:
                    XOR ?   rs1 ^   rs2:
                    SLL ?   rs1 <<  rs2:        //note shami (SLLI)
                    SRL ?   rs1 >>  rs2:        //note shami (SRLI)
                    SRA ?  (rs1 >> rs2)|((~(32'hffffffff>>rs2))&{32{rs1[31]}}):        //note shami (SRAI)
                    SLT ?  (CS?32'd1:32'd0):
                    SLTIU? (CP?32'd1:32'd0):32'd0;        

endmodule