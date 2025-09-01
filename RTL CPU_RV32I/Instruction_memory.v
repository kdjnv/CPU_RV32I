module Instruction_memory #(parameter MEM_FILE = "",
                parameter SIZE = 1024)(
    input           clk,
    input           l_pause,

//Bus
    input   [31:0]  mem_addr,   //address for read or write
    input   [31:0]  mem_addrpred,
    output  [31:0]  mem_rdata,  //read data  
    output  [31:0]  mem_rdata_pred,  
    input   	    mem_renable //high when CPU wants to read data

);
`define ENABLE_READ_INSTR_MEM;
`define PREDICT_EN;
//`define ENABLE_WRITE_INSTR_MEM;

    wire    [31:0]  addr_word;   assign addr_word = mem_addr[31:2]; //because MEM 32bit
//Boot
    (* ram_style = "block" *) reg [31:0] MEM [0:SIZE-1];            //1024 * 4 = 4096 = 4Kb 
    integer i;
    initial begin
        $readmemh(MEM_FILE,MEM);
    end

//read instr
    reg     [31: 0] rdata;  assign mem_rdata = rdata;
`ifdef PREDICT_EN
    reg     [31: 0] rdata_pred;  assign mem_rdata_pred = MEM[mem_addrpred[31:2]];   
`endif 
`ifdef ENABLE_READ_INSTR_MEM
    always @(posedge clk) begin
        if(mem_renable && !l_pause) begin
            rdata <= MEM[addr_word]; 
            //rdata_pred <= MEM[mem_addrpred[31:2]+1];
        end
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