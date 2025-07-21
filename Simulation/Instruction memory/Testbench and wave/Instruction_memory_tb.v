`timescale 1ns/1ps

module Instruction_memory_tb;

    reg         clk;
    reg  [31:0] mem_addr;
    wire [31:0] mem_rdata;
    reg  [31:0] mem_wdata = 32'hDEADBEEF;  // Dummy (not used)
    reg         mem_renable;
    reg  [3:0]  mem_mask = 4'b0000;        // Dummy (not used)

    integer i;

    //================================
    // Instantiate Instruction Memory
    //================================
    Instruction_memory #(
        .MEM_FILE("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware.hex"),
        .SIZE(1024)
    ) dut (
        .clk        (clk),
        .mem_addr   (mem_addr),
        .mem_rdata  (mem_rdata),
        .mem_wdata  (mem_wdata),
        .mem_renable(mem_renable),
        .mem_mask   (mem_mask)
    );

    //================================
    // Clock: 100 MHz
    //================================
    always #5 clk = ~clk;

    //================================
    // Task: ??c l?nh và hi?n th? toàn b? tín hi?u
    //================================
    task read_instr;
        input [31:0] addr;
        input integer index;
        begin
            mem_addr    = addr;
            mem_renable = 1;
            @(posedge clk); #1;
            $display("--------------------------------------------------");
            $display("READ #%0d", index);
            $display("  mem_addr    = 0x%08X", mem_addr);
            $display("  mem_rdata   = 0x%08X", mem_rdata);
            $display("  mem_renable = %b",    mem_renable);
            $display("  mem_mask    = %b",    mem_mask);
            $display("  mem_wdata   = 0x%08X", mem_wdata);
            $display("--------------------------------------------------");
            mem_renable = 0;
        end
    endtask

    //================================
    // Main Simulation
    //================================
    initial begin
        $dumpfile("instr_mem_tb.vcd");
        $dumpvars(0, Instruction_memory_tb);

        $display("=== Start Instruction Memory Test ===");

        clk = 0;
        mem_addr = 0;
        mem_renable = 0;
        #20;

        // ??c 10 l?nh t? ??a ch? 0 ??n 0x24
        for (i = 0; i < 10; i = i + 1) begin
            read_instr(i * 4, i);
            #5;
        end

        $display("=== End of Test ===");
        $finish;
    end

endmodule

