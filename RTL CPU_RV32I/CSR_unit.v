/*
CSR mie(Machine Interrupt Enable) sd MEIE, MTIE, MSIE
    Mode        index       name        Description
    ------------------------------------------------------------------------
    Machine     11          MEIE        Machine external interrupt enable
    Supervisor  9           SEIE        Supervisor external interrupt enable
    User        8           UEIE        User External Interrupt Enable
    
    Machine     7           MTIE        Machine Timer Interrupt Enable
    Supervisor  5           STIE        Supervisor Timer Interrupt Enable
    User        4           UTIE        User Timer Interrupt Enable

    Machine     3           MSIE        Machine Software Interrupt Enable
    Supervisor  1           SSIE        Supervisor Software Interrupt Enable
    User        0           USIE        User Software Interrupt Enable  

CSR mip(Machine Interrupt Pending)  sd MEIP, MTIP, MSIP
    Mode        index       name        Description
    ------------------------------------------------------------------------
    Machine     11          MEIP        Machine external interrupt Pending
    Supervisor  9           SEIP        Supervisor external interrupt Pending
    User        8           UEIP        User External Interrupt Pending
    
    Machine     7           MTIP        Machine Timer Interrupt Pending
    Supervisor  5           STIP        Supervisor Timer Interrupt Pending
    User        4           UTIP        User Timer Interrupt Pending

    Machine     3           MSIP        Machine Software Interrupt Pending
    Supervisor  1           SSIP        Supervisor Software Interrupt Pending
    User        0           USIP        User Software Interrupt Pending
 
CSR mstatus(Machine status) sd MIE, MPIE, MPP
    Mode        index       name        Description
    ------------------------------------------------------------------------
                31          SD          State Dirty nếu FS ≠ 00 hoặc XS ≠ 00. Nếu CPU không có FPU/vector thì luôn 0.
                22          TSR         Trap SRET: nếu =1, SRET trong S-mode sẽ trap vào M-mode. (chỉ khi có S-mode).
                21          TW          Timeout Wait: WFI trong S-mode gây trap vào M-mode (nếu =1).
                20          TVM         Trap Virtual Memory: truy cập SATP trong S-mode trap vào M-mode (nếu =1).
                19          MXR         Make eXecutable Readable: cho phép load từ page execute-only (S-mode).
                18          SUM         Permit Supervisor User Memory access: cho phép S-mode load/store U-mode memory
                17          MPRV        Modify PRiVilege: load/store sử dụng privilege ở MPP thay vì current privilege.
                16:15       XS          Extension Status: trạng thái dirty/clean cho extension (VD: vector).
                14:13       FS          Floating-point Status: 00=Off, 01=Initial, 10=Clean, 11=Dirty. Nếu không có FPU thì =00

    Machine     12:11       MPP         Machine Previous Privilege: lưu mode trước khi trap (M=11, S=01, U=00).
                10:9        VS          Vector Status (nếu có V extension). Nếu không → 00.
    Supervisor  8           SPP         Supervisor Previous Privilege: lưu mode trước khi trap vào S-mode.

    Machine     7           MPIE        Machine Previous Interrupt Enable: lưu MIE khi trap.
    User        6           UBE         Endianness: điều khiển endian của U-mode. Nếu CPU little-endian thì thường hardwired 0.
    Supervisor  5           SPIE        Supervisor Previous Interrupt Enable.

    Machine     3           MIE         Machine Interrupt Enable (global IE cho M-mode).
    Supervisor  1           SIE         Supervisor Interrupt Enable.

CSR mstatush(Machine status high)
    Mode        index       name        Description
    ------------------------------------------------------------------------
    Machine     5           MBE         Endianness: điều khiển endian của M-mode. Nếu CPU little-endian thì thường hardwired 0.
    Supervisor  4           SBE         Endianness: điều khiển endian của S-mode. Nếu CPU little-endian thì thường hardwired 0.

CSR mtvec(Machine Trap-Vector Base Address) sd BASE, MODE
    Mode        index       name        Description
    ------------------------------------------------------------------------
                31:2        BASE        Địa chỉ cơ sở của trap handler (phải aligned theo 4 byte). WARL
                1:0         MODE        Kiểu định địa chỉ trap vector (00: direct / 01: vectored).
                                        Direct: All exceptions set pc to BASE.
                                        Vectored: Asynchronous interrupts set pc to BASE+4×cause.

CSR mepc(Machine Exception Program Counter) sd all
    Mode        index       name        Description
    ------------------------------------------------------------------------
                31:0        mepc        Machine Exception Program Counter

CSR mcause(Machine Cause) sd all
    index       name                    Description
    ------------------------------------------------------------------------
    31          Interrupt               =1 nếu trap do interrupt (async), =0 nếu do exception (sync).
    30:0        Exception Code          Mã số cho biết loại interrupt/exception cụ thể.

    Interrupt   Exception Code          Description
    ------------------------------------------------------------------------
    1           0                       User software interrupt
    1           1                       Supervisor software interrupt
    1           3                       Machine software interrupt

    1           4                       User timer interrupt
    1           5                       Supervisor timer interrupt
    1           7                       Machine timer interrupt
    
    1           8                       User external interrupt
    1           9                       Supervisor external interrupt
    1           11                      Machine external interrupt
    1           >=16                    Designated for platform use

    0           X                       Read The RISC-V Instruction Set Manual
                                        Volume II: Privileged Archite
                                        Document Version 20211203
                                        Table 3.6 page 39

CSR mtval(Machine Trap Value)
*/



module CSR_unit(
    input           clk,
    input           rst,             // active-high reset

    // CSR interface
    input       [11:0]  csr_addrr,       // địa chỉ đọc CSR (12 bit)
    input       [11:0]  csr_addrw,       // địa chỉ ghi CSR (12 bit)
    input       [31:0]  csr_wdata,       // dữ liệu ghi
    input               csr_we,          // enable ghi
    output  reg [31:0]  csr_rdata,    // dữ liệu đọc

    // Trap/interrupt input
    input               trap_taken,      // khi có trap/interrut
    input               trap_complete,   // Hoàn thành xử lý trap
    input               updateMEPC,
    input       [31:0]  trap_pc,         // PC gây ra trap
    output      [31:0]  trap_addr,       // Đĩa chỉ nhảy tới khi trap
    output      [31:0]  trap_rpc,        // Return PC
    input       [31:0]  trap_cause,      // cause code
//    input   [31:0]  trap_tval,       // badaddr (load/store/fetch)

    // External inputs cho time và instret
    input       [63:0]  real_mtime,
    input       [63:0]  csr_instret,

    // Interrupt enables/pending 
    input               irq_software, 
    input               irq_timer, 
    input               irq_external, 
    output              global_ie
);

    // CSR registers
    reg [31:0] mstatus;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mepc;
    reg [31:0] mcause;
//    reg [31:0] mtval;
    reg [31:0] mip;

    // Counters
    reg [63:0] cycle;
    reg [63:0] instret;
    reg [63:0] mtime;

    // Global interrupt enable 
    assign global_ie = mstatus[3] && mie[11]; // MIE & MEIE
    assign trap_rpc  = mepc;
    assign trap_addr = {{mtvec[31:2],2'b00}};

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
            mtime   <= real_mtime;
        end
    end

    // -------------------
    // CSR write logic & trap update
    // -------------------
reg mask = 1'b0;
    always @(posedge clk) begin
        if (!rst) begin
            mstatus <= 32'h0;
            mie     <= 32'h0;
            mtvec   <= 32'h0;
            mepc    <= 32'h0;
            mcause  <= 32'h0;
            mask    <= 1'b0;
//            mtval   <= 32'h0;
            mip     <= 32'h0;
        end else begin

            // Update pending bits (phần cứng)
            mip[3]  <= irq_software; // MSIP
            mip[7]  <= irq_timer;    // MTIP
            mip[11] <= irq_external; // MEIP

            // CSR write từ phần mềm
            if (csr_we) begin
                case (csr_addrw)
                    12'h300: mstatus <= csr_wdata; 
                    12'h304: mie     <= csr_wdata; 
                    12'h305: mtvec   <= csr_wdata;  
                    12'h341: mepc    <= csr_wdata;  
                    12'h342: mcause  <= csr_wdata; 
//                    12'h343: mtval   <= csr_wdata;  
                    12'h344: mip  <= csr_wdata; 
                    default: ;
                endcase
            end

            // Khi có trap → cập nhật trap CSR
            if (trap_taken && !mask) begin
                mask   <= 1'b1;
                mepc   <= trap_pc;
                mcause <= trap_cause;
//                mtval  <= trap_tval;

                // Update mstatus theo spec
                mstatus[7]     <= mstatus[3];  // MPIE <- MIE
                mstatus[3]     <= 0;           // MIE <- 0
                mstatus[12:11] <= 2'b11;       // MPP <- M-mode
            end
    
            if (updateMEPC) mepc   <= trap_pc; 

            if (trap_complete) begin
                mask <= 1'b0;
                mstatus[3] <= mstatus[7]; 
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
//            12'h343: csr_rdata = mtval;
            12'h344: csr_rdata = mip;

            // counters (low/high)
            12'hC00: csr_rdata = cycle[31:0];    // cycle
            12'hC80: csr_rdata = cycle[63:32];   // cycleh
            12'hC01: csr_rdata = mtime[31:0];    // mtime
            12'hC81: csr_rdata = mtime[63:32];   // mtimeh
            12'hC02: csr_rdata = instret[31:0];  // instret
            12'hC82: csr_rdata = instret[63:32]; // instreth

            default: csr_rdata = 32'h0;
        endcase
    end

endmodule
