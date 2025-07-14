`timescale 1ns/1ps

module RV32I_tb;

    reg clk = 0;
    reg rst = 0;
    reg [31:0] instr_data;

    wire [31:0] Immediate;
    wire        insALUImm,insALUReg,insLUI,insAUIPC,insJAL,insJALR,insBRA,insLOAD,insSTORE,insSYS,insFENCE;
    wire [2:0]  funct3;
    wire [7:0]  funct3oh;
    wire [3:0]  funct3b;
    wire [6:0]  funct7;
    wire [4:0]  regrs2, regrs1, regrd;
    wire [3:0]  pred, succ;
    wire [11:0] indcsr;
    wire        isSigned;

    
    RV32_Decoder dut (
        .rst(rst),
        .clk(clk),
        .instr_data(instr_data),
        .Immediate(Immediate),
        .insALUImm(insALUImm),
        .insALUReg(insALUReg),
        .insLUI(insLUI),
        .insAUIPC(insAUIPC),
        .insJAL(insJAL),
        .insJALR(insJALR),
        .insBRA(insBRA),
        .insLOAD(insLOAD),
        .insSTORE(insSTORE),
        .insSYS(insSYS),
        .insFENCE(insFENCE),
        .funct3(funct3),
        .funct3oh(funct3oh),
        .funct3b(funct3b),
        .funct7(funct7),
        .regrs2(regrs2),
        .regrs1(regrs1),
        .regrd(regrd),
        .pred(pred),
        .succ(succ),
        .indcsr(indcsr),
        .isSigned(isSigned)
    );

    
    always #5 clk = ~clk;

    task print_state;
    begin
        $display("Time %0t ns", $time);
        $display("Instruction: 0x%h", instr_data);
        $display("  Immediate:  0x%h", Immediate);
        $display("  rs1: x%0d, rs2: x%0d, rd: x%0d", regrs1, regrs2, regrd);
        $display("  funct3: %03b, funct7: %07b", funct3, funct7);
        $display("  isSigned: %b", isSigned);
        $display("  insALUImm=%b, insALUReg=%b, insLUI=%b, insAUIPC=%b", insALUImm, insALUReg, insLUI, insAUIPC);
        $display("  insJAL=%b, insJALR=%b, insBRA=%b, insLOAD=%b, insSTORE=%b", insJAL, insJALR, insBRA, insLOAD, insSTORE);
        $display("  insSYS=%b, insFENCE=%b", insSYS, insFENCE);
        $display("-----------------------------------\n");
    end
    endtask

    initial begin
        $dumpfile("RV32I_tb.vcd");
        $dumpvars(0, RV32I_tb);

        
        rst = 1; #10;
        rst = 0;

        //addi	a0,a0,-576
	$display("addi	a0,a0,-576");
        instr_data = 32'hdc050513; #10;
        print_state();

  	//beq	a5,a4,abc
	$display("beq	a5,a4,abc");
        instr_data = 32'hece784e3; #10;
        print_state();

  	//lbu	a0,0(a5)
	$display("lbu	a0,0(a5)");
        instr_data = 32'h0007c503; #10;
        print_state();

  	//sw	s0,24(sp)
	$display("sw	s0,24(sp)");
        instr_data = 32'h00812c23; #10;
        print_state();

  	//add	s0,a5,sp
	$display("add	s0,a5,sp");
        instr_data = 32'h00278433; #10;
        print_state();

     	//fence	unknown,unknown
	$display("fence	unknown,unknown");
        instr_data = 32'h0000000f; #10;
        print_state();

    	//ebreak
	$display("ebreak");
        instr_data = 32'h00100073; #10;
        print_state();

	//lui	a0,0x1
	$display("lui	a0,0x1");
        instr_data = 32'h00001537; #10;
        print_state();

  	//jal	31c <uart_sendint>
	$display("jal	31c <uart_sendint>");
        instr_data = 32'h805ff0ef; #10;
        print_state();

        $finish;
    end

endmodule
