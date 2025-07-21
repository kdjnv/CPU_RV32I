/*   

    DTP D21DT166
   module uart ip
kết nối các module 
với thanh ghi uart


*/

//module uart_ip #(
//    parameter ADDR_W = 32,
//    parameter DATA_W = 32,
//    parameter STRB_W = DATA_W / 8
//)(

     
//    input clk,
//    input rst,


//    input  [ADDR_W-1:0] waddr,
//    input  [DATA_W-1:0] wdata,
//    input               wen,
//    input  [STRB_W-1:0] wstrb,
//    output              wready,
//    input  [ADDR_W-1:0] raddr,
//    input               ren,
//    output [DATA_W-1:0] rdata,
//    output              rvalid,

     

//    output  o_uart_tx


//);

//    reg o_ready_reg;

//    

//    wire [7:0] csr_u_data_data_out;
//    wire       o_ready;
//    wire       csr_u_ctrl_start_out;


//    regs_uart  uart_regs_interface_u0(

//        .clk(clk),
//        .rst(rst),
//        .csr_u_data_data_out(csr_u_data_data_out),
//        .csr_u_stat_ready_in(o_ready),
//        .csr_u_stat_tx_done_in( !o_ready_reg & o_ready  ),
//        .csr_u_ctrl_start_out(csr_u_ctrl_start_out),
//        .waddr(waddr),
//        .wdata(wdata),
//        .wen(wen),
//        .wstrb(wstrb),
//        .wready(wready),
//        .raddr(raddr),
//        .ren(ren),
//        .rdata(rdata),
//        .rvalid(rvalid)

//    );


//    uart_tx uart_transmitter_u1 (

//        .i_clk(clk),
//        .i_rst(rst),
//        .i_data(csr_u_data_data_out),
//        .i_valid(csr_u_ctrl_start_out),
//        .o_ready(o_ready),
//        .o_uart_tx(o_uart_tx)

//    );


//    always @(posedge clk) begin
//        o_ready_reg <= o_ready;
//    end



//endmodule



/*   
    DTP D21DT166
    module uart ip
    kết nối các module 
    với thanh ghi uart
*/

module uart_ip #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input                   clk,
    input                   rst,

    // Local Bus
    input  [ADDR_W-1:0]     waddr,
    input  [DATA_W-1:0]     wdata,
    input                   wen,
    input  [STRB_W-1:0]     wstrb,
    output                  wready,
    input  [ADDR_W-1:0]     raddr,
    input                   ren,
    output [DATA_W-1:0]     rdata,
    output                  rvalid,

    // UART TX RX
    output                  o_uart_tx,
    input                   i_uart_rx
);

    // --------------------------------------------------
    // Wires between regs_UART and UART
    // --------------------------------------------------
    wire        csr_en;
    wire        csr_strtx;
    wire [3:0]  csr_br;
    wire [7:0]  csr_clk_cfg;
    wire [7:0]  csr_tx_data;

    wire        uart_busy;
    wire        uart_rxne;    
    wire [7:0]  uart_rx_data; 

    // --------------------------------------------------
    // Instantiation: register interface
    // --------------------------------------------------
    regs_UART #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .STRB_W (STRB_W)
    ) regs_if (
        .clk                    (clk),
        .rst                    (!rst),

        // CTRL outputs
        .csr_u_ctrl_en_out      (csr_en),
        .csr_u_ctrl_strtx_out   (csr_strtx),
        .csr_u_ctrl_br_out      (csr_br),
        .csr_u_ctrl_clk_out     (csr_clk_cfg),

        // STAT inputs
        .csr_u_stat_tbusy_in    (uart_busy),
        .csr_u_stat_rxne_in     (uart_rxne),

        // TXDATA
        .csr_u_txdata_data_out  (csr_tx_data),

        // RXDATA 
        .csr_u_rxdata_data_in   (uart_rx_data),

        // Local bus
        .waddr                  (waddr),
        .wdata                  (wdata),
        .wen                    (wen),
        .wstrb                  (wstrb),
        .wready                 (wready),
        .raddr                  (raddr),
        .ren                    (ren),
        .rdata                  (rdata),
        .rvalid                 (rvalid)
    );

    // --------------------------------------------------
    // Instantiation: UART transmitter
    // --------------------------------------------------
    UART #(
        .CLOCK    (3_375_000),  
        .BAUD_RATE(115_200)
    ) uart_tx_inst (
        .i_clk      (clk),
        .i_rst      (rst),
        .i_en       (csr_en),
        .i_str_tx   (csr_strtx),
        .i_data_tx  (csr_tx_data),
        .i_br       (csr_br),
        .i_clk_dec  (csr_clk_cfg),

        // RX không dùng
        .i_RX       (i_uart_rx),
        .o_TX       (o_uart_tx),

        .o_busy_tx  (uart_busy),
        .o_RXNE     (uart_rxne),
        .o_data_rx  (uart_rx_data)
    );

endmodule
