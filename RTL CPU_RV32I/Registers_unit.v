module Registers_unit(
    input           clk,
    input           rst,

    //common port
    input   [ 4:0 ] rs1,
    input   [ 4:0 ] rs2,
    input   [ 4:0 ] rd,
    input   [31:0 ] data_des,
    output  [31:0]  data_rs1,
    output  [31:0]  data_rs2,
    input           data_valid,

    //port for predictor    
    input   [ 4:0 ] rs1pred,
    output  [31:0]  data_rs1pred

);
//31 general-purpose registers x1–x31, which hold integer values.
    reg     [31:0]  xreg[0:31];        
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            xreg[i] = 32'h00000000;
        end
    end
    
//Values source
    assign  data_rs1    =   xreg[rs1];
    assign  data_rs2    =   xreg[rs2];
    assign  data_rs1pred=   xreg[rs1pred];

//Values destination
    integer i;
    always @(posedge clk) begin
        if(data_valid && rd != 5'b00000) 
            xreg[rd] <= data_des;
        xreg[0] <= 32'h00000000;     //Register x0 is hardwired to the constant 0

        if(!rst) begin
            for(i = 0; i < 32; i = i + 1) begin
                xreg[i] <= 32'h00000000;
            end
        end
    end
endmodule