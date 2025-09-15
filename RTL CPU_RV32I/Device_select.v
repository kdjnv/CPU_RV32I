module Device_select (
    input   [31:0]  addr,
    output          s0_sel_plic,
//    output          s0_sel_mem,     //MEMORY_DATA
    output          s1_sel_gpio,
    output          s3_sel_uart      //UART 

);
   
//    wire mem_space = (addr[31:28] == 4'h2);
    wire plic_space = (addr[31:28] == 4'h3);
    wire gpio_space = (addr[31:28] == 4'h4);
    wire uart_space = (addr[31:28] == 4'h6);

    assign s0_sel_plic = plic_space;
    assign s1_sel_gpio = gpio_space;
//    assign s0_sel_mem  = mem_space;
    assign s3_sel_uart = uart_space;
endmodule
