module Top_module(
    input           clk,
    input           rst,
    
    input           uart_rx,
    output          uart_tx
);


wire    [31:0 ] peri_addr;
wire    [31:0 ] peri_wdata;
wire    [ 3:0 ] peri_wmask;
wire    [31:0 ] peri_rdata;
wire            peri_wen;
wire            peri_ren;

wire            s3_sel_uart;
wire    [31:0 ] rdata_uart;
wire            pready_uart;
wire            pslverr_uart;

wire    [ 2:0 ] peri_burst;
wire    [ 1:0 ] peri_htrans;
wire            peri_err;
wire            peri_rvalid;
wire            peri_wdone;

wire    [ 3:0 ] HWSTRB;
wire            HCLK = clk;
wire    [31:0 ] HADDR;
wire    [ 1:0 ] HTRANS;
wire            HWRITE;
wire    [ 2:0 ] HSIZE;
wire    [ 2:0 ] HBURST;
wire    [ 3:0 ] HPROT;
wire            HMASTLOCK;
wire    [31:0 ] HWDATA;
wire    [31:0 ] HRDATA;
wire            HREADY;
wire            HRESP;

wire            PCLK;
wire            PSEL;
wire    [31:0 ] PADDR;
wire            PWRITE;
wire    [31:0 ] PWDATA;
wire    [31:0 ] PWDATAT;
wire    [ 3:0 ] PSTRB;
wire            PENABLE;
reg     [31:0 ] PRDATA; 
reg             PREADY;
reg             PSLVERR;


    always @(*) begin
        case({s3_sel_uart})
            1'b1: PRDATA = rdata_uart;
            default: PRDATA = rdata_uart;
        endcase
    end

    always @(*) begin
        case({s3_sel_uart})
            1'b1: PREADY = pready_uart;
            default: PREADY = pready_uart;
        endcase
    end

    always @(*) begin
        case({s3_sel_uart})
            1'b1: PSLVERR = pslverr_uart;
            default: PSLVERR = pslverr_uart;
        endcase
    end


//////////////////////////////////////////////////////////////////////////////////////////////////
Device_select DS(
        .addr(PADDR),
        .s3_sel_uart(s3_sel_uart)
);


/////////////////////////////////////////////////////////////////////////////////////////////////
RV32I_core Core_CPU(
        .clk            (clk),    
        .rst            (rst),
        .peri_addr      (peri_addr),
        .peri_wdata     (peri_wdata),
        .peri_wmask     (peri_wmask),
        .peri_rdata     (peri_rdata),
        .peri_wen       (peri_wen),
        .peri_ren       (peri_ren),
        .peri_burst     (peri_burst),
        .peri_htrans    (peri_htrans),

        .peri_rvalid    (peri_rvalid),
        .peri_wdone     (peri_wdone),
        .peri_err       (peri_err)
);



///////////////////////////////////////////////////////////////////////////////////////////////////////////
ahb3lite_master_adapter CPU_to_AHB(
        .HCLK           (HCLK),
        .HRESETn        (rst),

        // Simple CPU/DMAC request interface
        .peri_addr      (peri_addr),
        .peri_wdata     (peri_wdata),
        .peri_wmask     (peri_wmask),     // 0000=read; otherwise write strobes
        .peri_wen       (peri_wen),
        .peri_ren       (peri_ren),
        .peri_burst     (peri_burst),
        .peri_htrans    (peri_htrans),

        // Read return (per beat)
        .peri_rvalid    (peri_rvalid),
        .peri_wdone     (peri_wdone),
        .peri_rdata     (peri_rdata),
        .peri_err       (peri_err),

        .PWDATAT        (PWDATAT),
        // AHB‑Lite master bus
        .HWSTRB         (HWSTRB),
        .HADDR          (HADDR),
        .HTRANS         (HTRANS),
        .HWRITE         (HWRITE),
        .HSIZE          (HSIZE),
        .HBURST         (HBURST),
//        .HPROT          (HPROT),           //Chưa dùng
//        .HMASTLOCK      (HMASTLOCK),       //Chưa dùng
        .HWDATA         (HWDATA),
        .HRDATA         (HRDATA),
        .HREADY         (HREADY),
        .HRESP          (HRESP)
);



////////////////////////////////////////////////////////////////////////////////////////////
ahb3lite_to_apb_bridge AHB_to_APB (
        .HCLK           (HCLK),
        .HRESETn        (rst),

        .PWDATAT        (PWDATAT),
        // AHB-Lite slave interface
        .HWSTRB         (HWSTRB),
        .HADDR          (HADDR),
        .HTRANS         (HTRANS),
        .HWRITE         (HWRITE),
        .HBURST         (HBURST),
        .HSIZE          (HSIZE),
        .HWDATA         (HWDATA),
        .HRDATA         (HRDATA),
        .HREADY         (HREADY),
        .HRESP          (HRESP),

        // APB master interface
        .PCLK           (PCLK),
        .PADDR          (PADDR),
        .PWRITE         (PWRITE),
        .PWDATA         (PWDATA),
        .PSTRB          (PSTRB),
        .PSEL           (PSEL),
        .PENABLE        (PENABLE),
        .PRDATA         (PRDATA),
        .PREADY         (PREADY),
        .PSLVERR        (PSLVERR)
);



/////////////////////////////////////////////////////////////////////////////////////////////
uart_ip uart_unit(
        .PCLK           (PCLK),
        .PRESETn        (rst),

        // APB
        .PSEL           (s3_sel_uart && PSEL),
        .PADDR          ({4'h0, PADDR[27:0]}),
        .PENABLE        (PENABLE),
        .PWRITE         (PWRITE),
        .PWDATA         (PWDATA),
        .PSTRB          (PSTRB),
        .PRDATA         (rdata_uart),
        .PREADY         (pready_uart),
        .PSLVERR        (pslverr_uart),

        .o_uart_tx  (uart_tx),
        .i_uart_rx  (uart_rx)
);

endmodule
