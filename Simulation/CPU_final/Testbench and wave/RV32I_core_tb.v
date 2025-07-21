`timescale 1ns/1ps

module RV32I_core_tb;

    reg clk;
    reg rst;

    RV32I_core dut (
        .clk(clk),
        .rst(rst)
    );

    always #5 clk = ~clk;

    initial begin
        $dumpfile("rv32i_core.vcd");          
        $dumpvars(0, RV32I_core_tb);          
        $display("=== Starting RV32I_core simulation ===");


        clk = 0;
        rst = 0;


        #20;
        rst = 1;

 
        #1000000;

        $display("=== Simulation completed ===");
        $finish;
    end

endmodule

