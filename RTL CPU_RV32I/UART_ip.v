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
    input           PCLK,
    input           PRESETn,

    // APB
    input   [31:0]  PADDR,
    input           PWRITE,
    input   [31:0]  PWDATA,
    input   [ 3:0]  PSTRB,
    input           PSEL,
    input           PENABLE,
    output  [31:0]  PRDATA,
    output          PREADY,
    output          PSLVERR,

    // UART TX RX
    output          o_uart_tx,
    input           i_uart_rx
);

    // --------------------------------------------------
    // Wires between regs_UART and UART
    // --------------------------------------------------
    wire        csr_en;
    wire        csr_strtx;
    wire [3:0]  csr_br;
    wire [7:0]  csr_PCLK_cfg;
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
    ) regs_uart (
        .clk                    (PCLK),
        .rst                    (!PRESETn),

        // CTRL outputs
        .csr_u_ctrl_en_out      (csr_en),
        .csr_u_ctrl_strtx_out   (csr_strtx),
        .csr_u_ctrl_br_out      (csr_br),
        .csr_u_ctrl_clk_out     (csr_PCLK_cfg),

        // STAT inputs
        .csr_u_stat_tbusy_in    (uart_busy),
        .csr_u_stat_rxne_in     (uart_rxne),

        // TXDATA
        .csr_u_txdata_data_out  (csr_tx_data),

        // RXDATA 
        .csr_u_rxdata_data_in   (uart_rx_data),

        // APB
        .psel           (PSEL),
        .paddr          (PADDR),
        .penable        (PENABLE),
        .pwrite         (PWRITE),
        .pwdata         (PWDATA),
        .pstrb          (PSTRB),
        .prdata         (PRDATA),
        .pready         (PREADY),
        .pslverr        (PSLVERR)
    );

    // --------------------------------------------------
    // Instantiation: UART transmitter
    // --------------------------------------------------
    UART #(
        .CLOCK    (27_000_000),  
        .BAUD_RATE(115_200)
    ) uart_tx_inst (
        .i_clk      (PCLK),
        .i_rst      (PRESETn),
        .i_en       (csr_en),
        .i_str_tx   (csr_strtx),
        .i_data_tx  (csr_tx_data),
        .i_br       (csr_br),
        .i_clk_dec  (csr_PCLK_cfg),

        // RX không dùng
        .i_RX       (i_uart_rx),
        .o_TX       (o_uart_tx),

        .o_busy_tx  (uart_busy),
        .o_RXNE     (uart_rxne),
        .o_data_rx  (uart_rx_data)
    );

endmodule
