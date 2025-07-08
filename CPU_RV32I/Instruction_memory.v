module Instruction_memory #(parameter MEM_FILE = "",
                parameter SIZE = 1024)(
    input           clk,

//Bus
    input   [31:0]  mem_addr,   //address for read or write
    output  [31:0]  mem_rdata,  //read data  
    input   [31:0]  mem_wdata,  //write data
    input   	    mem_renable,//high when CPU wants to read data
    input   [ 3:0]  mem_mask    //Choose byte (1 for write)
);
`define ENABLE_READ_INSTR_MEM;
//`define ENABLE_WRITE_INSTR_MEM;

    wire    [31:0]  addr_word;   assign addr_word = mem_addr[31:2]; //because MEM 32bit
//Boot
    (* ram_style = "block" *) reg [31:0] MEM [0:SIZE-1];            //1024 * 4 = 4096 = 4Kb  
    initial begin
        $readmemh(MEM_FILE,MEM);
    end

//read instr
    reg     [31: 0] rdata;  assign mem_rdata = rdata;
`ifdef ENABLE_READ_INSTR_MEM
    always @(posedge clk) begin
        if(mem_renable)
            rdata <= MEM[addr_word];   
    end
`endif

//write instr
`ifdef ENABLE_WRITE_INSTR_MEM
    always @(posedge clk) begin
        if(mem_mask[0]) MEM[addr_word][ 7:0 ] <= mem_wdata[ 7:0 ];
        if(mem_mask[1]) MEM[addr_word][15:8 ] <= mem_wdata[15:8 ];
        if(mem_mask[2]) MEM[addr_word][23:16] <= mem_wdata[23:16];
        if(mem_mask[3]) MEM[addr_word][31:24] <= mem_wdata[31:24];
    end
`endif

endmodule