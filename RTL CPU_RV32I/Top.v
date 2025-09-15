module Top_module(
    input           clk,
    input           rst,

    inout   [15:0 ] GPIO_x,

    input           uart_rx,
    output          uart_tx
);

wire clksys = clk;
//    Gowin_CLKDIV your_instance_name(
//        .clkout(clksys), //output clkout
//        .hclkin(clk), //input hclkin
//        .resetn(rst) //input resetn
//    );

wire    [31:0 ] peri_addr;
wire    [31:0 ] peri_wdata;
wire    [ 3:0 ] peri_wmask;
wire    [31:0 ] peri_rdata;
wire            peri_wen;
wire            peri_ren;

wire            s3_sel_uart;
wire            s1_sel_gpio;
wire            s0_sel_plic;
wire    [31:0 ] rdata_uart;
wire            pready_uart;
wire            pslverr_uart;

wire    [ 2:0 ] peri_burst;
wire    [ 1:0 ] peri_htrans;
wire            peri_err;
wire            peri_rvalid;
wire            peri_wdone;

wire    [ 3:0 ] HWSTRB;
wire            HCLK = clksys;
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

wire            irq_out;
wire    [ 5:0 ] claim_id;
wire            irq_external_pending;
wire            trap_en;

wire    [31:0 ] rdata_plic;
wire            pready_plic;
wire            pslverr_plic; 

wire    [31:0 ] rdata_gpio;
wire            pready_gpio;
wire            pslverr_gpio;

wire            irqs1_rxuart;  
wire            irqs2_txuart;  
wire    [15:0 ] irqsx_gpio_pedge;
wire    [15:0 ] irqsx_gpio_nedge;   


    always @(*) begin
        case({s3_sel_uart, s1_sel_gpio, s0_sel_plic})
            3'b001: PRDATA = rdata_plic;
            3'b010: PRDATA = rdata_gpio;
            3'b100: PRDATA = rdata_uart;
            default: PRDATA = 32'h0;
        endcase
    end

    always @(*) begin
        case({s3_sel_uart, s1_sel_gpio, s0_sel_plic})
            3'b001: PREADY = pready_plic;
            3'b010: PREADY = pready_gpio;
            3'b100: PREADY = pready_uart;
            default: PREADY = 1'h0;
        endcase
    end

    always @(*) begin
        case({s3_sel_uart, s1_sel_gpio, s0_sel_plic})
            3'b001: PSLVERR = pslverr_plic;
            3'b010: PSLVERR = pslverr_gpio;
            3'b100: PSLVERR = pslverr_uart;
            default: PSLVERR = 1'h0;
        endcase
    end


//////////////////////////////////////////////////////////////////////////////////////////////////
Device_select DS(
        .addr(PADDR),
        .s0_sel_plic(s0_sel_plic),
        .s1_sel_gpio(s1_sel_gpio),
        .s3_sel_uart(s3_sel_uart)
);


/////////////////////////////////////////////////////////////////////////////////////////////////
RV32I_core Core_CPU(
        .clk            (clksys),    
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
        .peri_err       (peri_err),
    
        .irq_flag       (irq_out),
        .irq_external_pending(irq_external_pending),
        .trap_en        (trap_en)
//        .irq_claim_id   (claim_id)
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

        .irqs1_rxuart   (irqs1_rxuart),
        .irqs2_txuart   (irqs2_txuart),

        .o_uart_tx  (uart_tx),
        .i_uart_rx  (uart_rx)
);



/////////////////////////////////////////////////////////////////////////////////////////////
PLIC_ip PLIC_unit(
        .PCLK           (PCLK),
        .PRESETn        (rst),

        .irqs1_rxuart   (irqs1_rxuart),             //Ngắt của UART
        .irqs2_txuart   (irqs2_txuart),

        .irqs3_pgpio0   (irqsx_gpio_pedge[0]),      //Ngắt của GPIO
        .irqs4_ngpio0   (irqsx_gpio_nedge[0]),

        .irqs5_pgpio1   (irqsx_gpio_pedge[1]),
        .irqs6_ngpio1   (irqsx_gpio_nedge[1]),

        .irqs7_pgpio2   (irqsx_gpio_pedge[2]),
        .irqs8_ngpio2   (irqsx_gpio_nedge[2]),

        .irqs9_pgpio3   (irqsx_gpio_pedge[3]),
        .irqs10_ngpio3  (irqsx_gpio_nedge[3]),

        .irqs11_pgpio4  (irqsx_gpio_pedge[4]),
        .irqs12_ngpio4  (irqsx_gpio_nedge[4]),

        .irqs13_pgpio5  (irqsx_gpio_pedge[5]),
        .irqs14_ngpio5  (irqsx_gpio_nedge[5]),

        .irqs15_pgpio6  (irqsx_gpio_pedge[6]),
        .irqs16_ngpio6  (irqsx_gpio_nedge[6]),

        .irqs17_pgpio7  (irqsx_gpio_pedge[7]),
        .irqs18_ngpio7  (irqsx_gpio_nedge[7]),

        .irqs19_pgpio8  (irqsx_gpio_pedge[8]),
        .irqs20_ngpio8  (irqsx_gpio_nedge[8]),

        .irqs21_pgpio9  (irqsx_gpio_pedge[9]),
        .irqs22_ngpio9  (irqsx_gpio_nedge[9]),

        .irqs23_pgpio10 (irqsx_gpio_pedge[10]),
        .irqs24_ngpio10 (irqsx_gpio_nedge[10]),

        .irqs25_pgpio11 (irqsx_gpio_pedge[11]),
        .irqs26_ngpio11 (irqsx_gpio_nedge[11]),

        .irqs27_pgpio12 (irqsx_gpio_pedge[12]),
        .irqs28_ngpio12 (irqsx_gpio_nedge[12]),

        .irqs29_pgpio13 (irqsx_gpio_pedge[13]),
        .irqs30_ngpio13 (irqsx_gpio_nedge[13]),

        .irqs31_pgpio14 (irqsx_gpio_pedge[14]),
        .irqs32_ngpio14 (irqsx_gpio_nedge[14]),

        .irqs33_pgpio15 (irqsx_gpio_pedge[15]),
        .irqs34_ngpio15 (irqsx_gpio_nedge[15]),
        

        .irq_out        (irq_out),
        .irq_external_pending(irq_external_pending),
        .trap_en        (trap_en),
//        .claim_id       (claim_id),

        .PSEL           (s0_sel_plic && PSEL),
        .PADDR          ({4'h0, PADDR[27:0]}),
        .PENABLE        (PENABLE),
        .PWRITE         (PWRITE),
        .PWDATA         (PWDATA),
        .PSTRB          (PSTRB),
        .PRDATA         (rdata_plic),
        .PREADY         (pready_plic),
        .PSLVERR        (pslverr_plic)
);


//////////////////////////////////////////////////////////////////////////////////////////////
gpio_ip gpio_unit(
        .PCLK           (PCLK),
        .PRESETn        (rst),

        // APB
        .PSEL           (s1_sel_gpio && PSEL),
        .PADDR          ({4'h0, PADDR[27:0]}),
        .PENABLE        (PENABLE),
        .PWRITE         (PWRITE),
        .PWDATA         (PWDATA),
        .PSTRB          (PSTRB),
        .PRDATA         (rdata_gpio),
        .PREADY         (pready_gpio),
        .PSLVERR        (pslverr_gpio),

        .irqsx_gpio_pedge(irqsx_gpio_pedge),
        .irqsx_gpio_nedge(irqsx_gpio_nedge),

        .GPIO_x         (GPIO_x)
);

endmodule
