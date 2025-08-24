`define HTRANS_IDLE   2'b00
`define HTRANS_BUSY   2'b01
`define HTRANS_NONSEQ 2'b10
`define HTRANS_SEQ    2'b11


// -------------------------------------------------------------
// Master Adapter (burst capable)
//  * Accepts simple request interface + length (beats)
//  * Generates NONSEQ + SEQ with HBURST = SINGLE/INCR4/8/16
//  * Sustains 1 beat/clk when HREADY=1
// -------------------------------------------------------------
module ahb3lite_master_adapter (
    input  wire        HCLK,
    input  wire        HRESETn,

    // Simple CPU/DMAC request interface
    input  wire [31:0]  peri_addr,
    input  wire [31:0]  peri_wdata,
    input  wire  [3:0]  peri_wmask,     // 0000=read; otherwise write strobes
    input               peri_wen,
    input               peri_ren,
    input       [ 2:0 ] peri_burst,
    input       [ 1:0 ] peri_htrans,

    // Read return (per beat)
    output reg          peri_rvalid,
    output reg          peri_wdone,
    output reg  [31:0]  peri_rdata,
    output              peri_err,

    // AHB‑Lite master bus
    output reg  [31:0] HADDR,
    output reg   [1:0] HTRANS,
    output reg         HWRITE,
    output reg   [2:0] HSIZE,
    output       [2:0] HBURST,
    output reg   [3:0] HPROT,           //Chưa dùng
    output reg         HMASTLOCK,       //Chưa dùng
    output reg  [31:0] HWDATA,
    input  wire [31:0] HRDATA,
    input  wire        HREADY,
    input  wire        HRESP
);
    localparam S_IDLE = 2'd0,
               S_ADDR = 2'd1,
               S_DATA = 2'd2;

    reg [1:0] state;
    reg [4:0] count_burst;
    reg [4:0] cnt_burst_max;
    reg       burst_done;
    reg [2:0] plus;

//    wire[6:0] block_size = f_plus_from_wstrb(peri_wmask) * (f_count_from_burst(peri_burst));
//    wire[6:0] block_mask = block_size - 1;
//    wire[31:0]block_base = peri_addr & ~block_mask;
//    wire[31:0]addr_max   = block_base + block_size - f_plus_from_wstrb(peri_wmask);
//    wire[4:0] count_wrap_max = (addr_max - peri_addr)/f_plus_from_wstrb(peri_wmask);

    function [2:0] f_plus_from_wstrb;
        input [3:0] wstrb; begin
            case (wstrb)
                4'b0001,4'b0010,4'b0100,4'b1000: f_plus_from_wstrb = 3'd1; // byte
                4'b0011,4'b1100:                 f_plus_from_wstrb = 3'd2; // half
                4'b1111:                         f_plus_from_wstrb = 3'd4; // word
                default:                         f_plus_from_wstrb = 3'd4;
            endcase
        end
    endfunction

    function [2:0] f_hsize_from_wstrb;
        input [3:0] wstrb; begin
            case (wstrb)
                4'b0001,4'b0010,4'b0100,4'b1000: f_hsize_from_wstrb = 3'b000; // byte
                4'b0011,4'b1100:                 f_hsize_from_wstrb = 3'b001; // half
                4'b1111:                         f_hsize_from_wstrb = 3'b010; // word
                default:                         f_hsize_from_wstrb = 3'b010;
            endcase
        end
    endfunction

    function [4:0] f_count_from_burst;
        input [2:0] burst; begin
            case (burst)
                3'b000:                             f_count_from_burst = 5'd1; // Single transfer
                3'b001:                             f_count_from_burst = 5'd0; // Incrementing burst of undefined length
                3'b010, 3'b011:                     f_count_from_burst = 5'd4; // 4-beat burst
                3'b100, 3'b101:                     f_count_from_burst = 5'd8; // 8-beat burst 
                3'b110, 3'b111:                     f_count_from_burst = 5'd16; // 16-beat burst               
                default:                            f_count_from_burst = 5'b1;
            endcase
        end
    endfunction

    

    assign  HBURST      =   peri_burst;
    assign  peri_rdata  =   HRDATA;
    assign  peri_rvalid =   HREADY && (peri_htrans && burst_done);
    assign  peri_wdone  =   HREADY && (peri_htrans && burst_done);
    assign  peri_err    =   HRESP;
    
    


    always @(posedge HCLK) begin
        if (!HRESETn) begin
            state <= S_IDLE;
            count_burst <= 0;
            beats_total <= 4'd0; beats_left <= 4'd0;
        end else begin            

            case (peri_htrans)
                HTRANS_IDLE: begin
                    HTRANS <= 2'b00;        //IDLE
                end
                HTRANS_BUSY: begin        
                    HTRANS <= 2'b01;        //BUSY
                    HWDATA <= HWDATA;   HWRITE <= HWRITE;
                    HSIZE  <= HSIZE;    HADDR  <= HADDR;
                end
                HTRANS_NONSEQ: begin        
                    HTRANS <= 2'b10;        //NONSEQ 
                    HADDR  <= peri_addr;
                    HWRITE <= |peri_wmask && peri_wen;
                    HSIZE  <= f_hsize_from_wstrb(peri_wmask);
                    HWDATA <= peri_wdata;
                end
                HTRANS_SEQ: begin 
                    HTRANS <= 2'b11;        //SEQ
                    HADDR <= peri_addr + f_plus_from_wstrb(peri_wmask)*count_burst;//cần bổ sung khi wrap
                    HWRITE <= |peri_wmask && peri_wen;
                    HSIZE  <= f_hsize_from_wstrb(peri_wmask);
                    HWDATA <= peri_wdata;
                    count_burst <= count_burst + 1;

                    case(state) 
                        2'b00: begin

                            case (f_count_from_burst(peri_burst)) 
                                4'd0: state <= 2'b10;                                   //n (với mọi n > 0)
                                4'd1: begin
                                    count_burst <= count_burst;
                                    state <= 2'b11;
                                end
                                default: begin
                                    cnt_burst_max <= f_count_from_burst(peri_burst)-1;  //do bắt đầu của SEQ bao giờ cũng là NONSEQ
                                    state <= 2'b01;                                     //4 8 16
                                end
                            
                        end
                        2'b01: begin
                            if(count_burst == cnt_burst_max - 1) state <= 2'b11;
                        end
                        2'b10: begin
                            //
                        end
                        2'b11: begin
                            count_burst <= count_burst;
                            HWDATA <= HWDATA;   HWRITE <= HWRITE;
                            HSIZE  <= HSIZE;    HADDR  <= HADDR;
                            HTRANS <= HTRANS;
                        end
                    endcase
                end
            endcase
        end
    end
endmodule






