module Data_memory #(parameter SIZE = 4096)(
    input           clk,

//Bus
    input   [31:0]  mem_addr,   //address for load or store
    output  [31:0]  mem_ldata,  //load data  
    input   [31:0]  mem_sdata,  //store data
    input   	    mem_lenable,//high when CPU wants to load data
    input   [ 3:0]  mem_mask    //Choose byte (1 for write)
);
`define ENABLE_WRITE_DATA_MEM;
`define ENABLE_READ_DATA_MEM;



    (* ram_style = "block" *) reg [31:0] MEM [0:SIZE-1];            //4096 * 4 = 16Kb
    wire    [31:0]  addr_word;   assign addr_word = mem_addr[31:2]; //because MEM 32bit

    //read instr
    reg     [31: 0] rdata = 32'b0;  assign mem_ldata = rdata;
`ifdef ENABLE_READ_DATA_MEM
    always @(posedge clk) begin
        if(mem_lenable)
            rdata <= MEM[addr_word];   
    end
`endif

    //write instr
`ifdef ENABLE_WRITE_DATA_MEM
    always @(posedge clk) begin
        if(mem_mask[0]) MEM[addr_word][ 7:0 ] <= mem_sdata[ 7:0 ];
        if(mem_mask[1]) MEM[addr_word][15:8 ] <= mem_sdata[15:8 ];
        if(mem_mask[2]) MEM[addr_word][23:16] <= mem_sdata[23:16];
        if(mem_mask[3]) MEM[addr_word][31:24] <= mem_sdata[31:24];
    end
`endif

endmodule