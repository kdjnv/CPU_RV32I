module CSR_unit(
    input           clk,
    input           rst,

    // CSR interface
    input   [11:0]  csr_addrr,      // địa chỉ đọc CSR (12 bit)
    input   [11:0]  csr_addrw,      // địa chỉ ghi CSR (12 bit)
    input   [31:0]  csr_wdata,      // dữ liệu ghi
    input           csr_we,         // enable ghi
    output  reg [31:0] csr_rdata,   // dữ liệu đọc

    // Trap/interrupt input
    input           trap_taken,     // khi có trap/interrupt
    input   [31:0]  trap_vector,    // địa chỉ handler
    input   [31:0]  trap_pc,        // PC gây ra trap
    input   [31:0]  trap_cause,     // cause code
    input   [31:0]  trap_tval,      // badaddr (load/store/fetch)

    // External inputs cho time và instret
    input   [63:0]  real_mtime,
    input   [63:0]  csr_instret
);

    // CSR registers
    reg [31:0] mstatus;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;
    reg [31:0] mip;

    // Counters
    reg [63:0] cycle;
    reg [63:0] instret;
    reg [63:0] mtime;

    // -------------------
    // Counter, ins, time update
    // -------------------
    always @(posedge clk) begin
        if (!rst) begin
            cycle   <= 64'd0;
            instret <= 64'd0;
        end else begin
            cycle   <= cycle + 1;    // tăng mỗi cycle
            instret <= csr_instret;
            mtime <= real_mtime;
        end
    end

    // -------------------
    // CSR write logic
    // -------------------
    always @(posedge clk) begin
        if (!rst) begin
            mstatus <= 32'h0;
            mie     <= 32'h0;
            mtvec   <= 32'h0;
            mepc    <= 32'h0;
            mcause  <= 32'h0;
            mtval   <= 32'h0;
            mip     <= 32'h0;
        end else begin
            if (csr_we) begin
                case (csr_addrw)
                    12'h300: mstatus <= csr_wdata;  // mstatus
                    12'h304: mie     <= csr_wdata;  // mie
                    12'h305: mtvec   <= csr_wdata;  // mtvec
                    12'h341: mepc    <= csr_wdata;  // mepc
                    12'h342: mcause  <= csr_wdata;  // mcause
                    12'h343: mtval   <= csr_wdata;  // mtval
                    12'h344: mip     <= csr_wdata;  // mip
                    default: ;
                endcase
            end

            // khi có trap → cập nhật trap CSR
            if (trap_taken) begin
                mepc   <= trap_pc;
                mcause <= trap_cause;
                mtval  <= trap_tval;
            end
        end
    end

    // -------------------
    // CSR read logic
    // -------------------
    always @(*) begin
        case (csr_addrr)
            // machine-level CSRs
            12'h300: csr_rdata = mstatus;
            12'h304: csr_rdata = mie;
            12'h305: csr_rdata = mtvec;
            12'h341: csr_rdata = mepc;
            12'h342: csr_rdata = mcause;
            12'h343: csr_rdata = mtval;
            12'h344: csr_rdata = mip;

            // counters (low/high)
            12'hC00: csr_rdata = cycle[31:0];    // cycle
            12'hC80: csr_rdata = cycle[63:32];   // cycleh
            12'hC01: csr_rdata = mtime[31:0];     // mtime (giả lập = real_mtime)
            12'hC81: csr_rdata = mtime[63:32];    // mtimeh
            12'hC02: csr_rdata = instret[31:0];  // instret
            12'hC82: csr_rdata = instret[63:32]; // instreth

            default: csr_rdata = 32'h0;
        endcase
    end

endmodule
