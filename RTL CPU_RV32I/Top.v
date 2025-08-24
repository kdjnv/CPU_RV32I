module Top_module(
    input           clk,
    input           rst,
    
    input           uart_rx,
    output          uart_tx
);


wire            clksys = clk;
wire    [31:0 ] peri_addr;
wire    [31:0 ] peri_wdata;
wire    [ 3:0 ] peri_wmask;
reg     [31:0 ] peri_rdata;
wire            peri_wen;
wire            peri_ren;
wire            s3_sel_uart;
wire    [31:0 ] rdata_uart;

    always @(*) begin
        peri_rdata = rdata_uart;
    end

RV32I_core Core_CPU(
    .clk            (clksys),    
    .rst            (rst),
    .peri_addr      (peri_addr),
    .peri_wdata     (peri_wdata),
    .peri_wmask     (peri_wmask),
    .peri_rdata     (peri_rdata),
    .peri_wen       (peri_wen),
    .peri_ren       (peri_ren),

    .s3_sel_uart    (s3_sel_uart)
);

uart_ip uart_unit(
    .clk        (clksys),
    .rst        (rst),

    .waddr      ({4'h0, peri_addr[27:0]}),
    .wdata      (peri_wdata),
    .wen        (s3_sel_uart & (|peri_wmask) & peri_wen),
    .wstrb      (peri_wmask),
    .wready     (),
    .raddr      ({4'h0, peri_addr[27:0]}),
    .ren        (s3_sel_uart & peri_ren),
    .rdata      (rdata_uart),
    .rvalid     (),

    .o_uart_tx  (uart_tx),
    .i_uart_rx  (uart_rx)
);

endmodule
