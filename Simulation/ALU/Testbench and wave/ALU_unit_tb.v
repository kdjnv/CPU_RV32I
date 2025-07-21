`timescale 1ns/1ps
module ALU_unit_tb;
  reg        clk       = 0;
  reg        isALUimm  = 0;
  reg        isALUreg  = 0;
  reg        isBranch  = 0;
  reg [ 7:0] funct3oh   = 8'd0;
  reg [ 6:0] funct7     = 7'd0;
  reg [31:0] rs1        = 32'd0;
  reg [31:0] rs2        = 32'd0;

  wire [31:0] result;
  wire        correct;

  ALU_unit ALUtest(
    .clk      (clk),
    .isALUimm (isALUimm),
    .isALUreg (isALUreg),
    .isBranch (isBranch),
    .funct3oh (funct3oh),
    .funct7   (funct7),
    .rs1      (rs1),
    .rs2      (rs2),
    .result   (result),
    .correct  (correct)
  );

    task print_state;
    begin
        $display("Time: %0t ns", $time);
        $display("  isALUimm=%b, isALUreg=%b, isBranch=%b", isALUimm, isALUreg, isBranch);
        $display("  funct3oh: %08b", funct3oh);
        $display("  funct7:   %07b", funct7);
        $display("  rs1: 0x%08h (%0d)", rs1, rs1);
        $display("  rs2: 0x%08h (%0d)", rs2, rs2);
        $display("  --> Result = 0x%08h (%0d)", result, result);
        $display("  --> Correct = %b", correct);
        $display("-----------------------------------\n");
    end
    endtask


  always #5 clk = ~clk;

  initial begin
    $dumpfile("ALU_unit_tb.vcd");
    $dumpvars(0, ALU_unit_tb);

    // Test: ADDI
    $display("ADDI: a3 = a3 + (-1)");
    isALUimm = 1; isALUreg = 0; isBranch = 0;
    funct3oh = 8'b0000_0001;
    rs1 = 32'd10; rs2 = -1;
    #10; print_state();

    // Test: SLTI
    $display("SLTI: x5 = (x6 < 10) ? 1 : 0");
    isALUimm = 1; funct3oh = 8'b0000_0100; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: SLTIU
    $display("SLTIU: x5 = (x6 < 10) ? 1 : 0 (unsigned)");
    isALUimm = 1; funct3oh = 8'b0000_1000; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: XORI
    $display("XORI: x5 = x6 ^ imm");
    funct3oh = 8'b0001_0000; rs1 = 32'hAA55AA55; rs2 = 32'hFFFF0000;
    #10; print_state();

    // Test: ORI
    $display("ORI: x5 = x6 | imm");
    funct3oh = 8'b0100_0000; rs1 = 32'h0F0F0F0F; rs2 = 32'hF0F0F0F0;
    #10; print_state();

    // Test: ANDI
    $display("ANDI: x5 = x6 & imm");
    funct3oh = 8'b1000_0000; rs1 = 32'hFFFF0000; rs2 = 32'h0F0F0F0F;
    #10; print_state();

    // Test: ADD
    $display("ADD: x5 = x6 + x7");
    isALUimm = 0; isALUreg = 1;
    funct3oh = 8'b0000_0001; funct7 = 7'b0000000; rs1 = 32'd7; rs2 = 32'd8;
    #10; print_state();

    // Test: SUB
    $display("SUB: x5 = x6 - x7");
    funct7 = 7'b0100000; rs1 = 32'd8; rs2 = 32'd5;
    #10; print_state();

    // Test: SLL
    $display("SLL: x5 = x6 << x7");
    funct7 = 7'b0000000;
    funct3oh = 8'b0000_0010; funct7 = 7'b0000000; rs1 = 32'd1; rs2 = 32'd3;
    #10; print_state();

    // Test: SLT
    $display("SLT: x5 = (x6 < x7) ? 1 : 0");
    funct3oh = 8'b0000_0100; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: SLTU
    $display("SLTU: x5 = (x6 < x7) ? 1 : 0 (unsigned)");
    funct3oh = 8'b0000_1000; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: SRL
    $display("SRL: x5 = x6 >> x7 (logical)");
    funct3oh = 8'b0010_0000; rs1 = 32'hF0000000; rs2 = 32'd4;
    #10; print_state();

    // Test: SRA
    $display("SRA: x5 = x6 >> x7 (arithmetic)");
    funct7 = 7'b0100000; rs1 = 32'hF0000000; rs2 = 32'd12;
    #10; print_state();

    // Test: OR
    $display("OR: x5 = x6 | x7");
    funct7 = 7'b0000000;
    funct3oh = 8'b0100_0000; rs1 = 32'h0000FFFF; rs2 = 32'hFFFF0000;
    #10; print_state();

    // Test: AND
    $display("AND: x5 = x6 & x7");
    funct3oh = 8'b1000_0000; rs1 = 32'h12345678; rs2 = 32'hFFFFFFFF;
    #10; print_state();

    // Test: BEQ
    $display("BEQ: x6 == x7");
    isALUreg = 0; isBranch = 1;
    funct3oh = 8'b0000_0001; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: BNE
    $display("BNE: x6 != x7");
    funct3oh = 8'b0000_0010; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: BLT
    $display("BLT: x6 < x7 (signed)");
    funct3oh = 8'b0001_0000; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: BLTU
    $display("BLTU: x6 < x7 (unsigned)");
    funct3oh = 8'b0100_0000; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: BGE
    $display("BGE: x6 >= x7 (signed)");
    funct3oh = 8'b0010_0000; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    // Test: BGEU
    $display("BGEU: x6 >= x7 (unsigned)");
    funct3oh = 8'b1000_0000; rs1 = 32'hfffffff3; rs2 = 32'h7ffffff4;
    #10; print_state();

    $finish;
  end
endmodule
