`timescale 1ns/1ps

module Data_memory_tb;
    reg         clk = 0;
    reg  [31:0] mem_addr	= 32'b0;
    wire [31:0] mem_ldata;	
    reg  [31:0] mem_sdata	= 32'b0;
    reg         mem_lenable	= 1'b0;
    reg  [3:0]  mem_mask	= 4'b0000;

    Data_memory DatamemTest (
        .clk        (clk),
        .mem_addr   (mem_addr),
        .mem_ldata  (mem_ldata),
        .mem_sdata  (mem_sdata),
        .mem_lenable(mem_lenable),
        .mem_mask   (mem_mask)
    );

    // Clock generator
    always #5 clk = ~clk;

    task print_state;
    begin
        $display("[%0t ns]", $time);
        $display("  Address  = 0x%08h", mem_addr);
        $display("  Store_data = 0x%08h", mem_sdata);
        $display("  Load_data = 0x%08h", mem_ldata);
        $display("  ActLoad = %b, mem_mask = %b", mem_lenable, mem_mask);
        $display("-----------------------------");
    end
    endtask

    initial begin
        $dumpfile("Data_memory_tb.vcd");
        $dumpvars(0, Data_memory_tb);

        
        // Write to address 0x00
	$display("Write 0xDEADBEEF to 0x00");
        mem_addr   = 32'h00000000;
        mem_sdata  = 32'hDEADBEEF;
        mem_mask   = 4'b1111;  // write full 32-bit word
        #10;
        print_state;
	mem_mask = 4'b0000;

        // Write to address 0x04
	$display("Write 0xCAFEBABE to 0x04");
        mem_addr   = 32'h00000004;
        mem_sdata  = 32'hCAFEBABE;
	mem_mask = 4'b1111;
        #10;
        print_state;
	mem_mask = 4'b0000;

        // Read from address 0x00
	$display("Read from 0x00");
        mem_lenable = 1;
        mem_addr = 32'h00000000;
        #10;
	mem_lenable = 0;
        print_state;

        // Read from address 0x04
	$display("Read from 0x04");
        mem_addr = 32'h00000004;
	mem_lenable = 1;
        #10;
	mem_lenable = 0;
        print_state;

        $finish;
    end

endmodule
