module Device_select (
    input [31:0] addr,
    output s0_sel_mem   //MEMORY_DATA
);
  wire mem_space = (addr[31:28] != 4'h0);

  assign s0_sel_mem  = mem_space;
endmodule
