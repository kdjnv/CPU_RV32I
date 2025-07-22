module Device_select (
    input   [31:0]  addr,
    output          s0_sel_mem,     //MEMORY_DATA
    output          s3_sel_uart      //UART 
);
    wire mem_space = (addr[31:28] == 4'h2);
    wire uart_space = (addr[31:28] == 4'h6);

    assign s0_sel_mem  = mem_space;
    assign s3_sel_uart = uart_space;
endmodule
