module RV32I_core(
    input           clk,    
    input           rst,

    input           uart_rx,
    output          uart_tx
);
/*===========================================================================
1. Basic Mode (No Pipeline – 5 Cycles per Instruction)
When FIVE_STATE_EN is defined, the CPU operates in its most basic form without pipelining.  
Each instruction goes through 5 sequential stages (such as Fetch, Decode, Execute, Memory,
and Writeback) but only one stage is processed at a time. As a result, one instruction re-
-quires 5 clock cycles to complete. This mode prioritizes simplicity and ease of verifica-
-tion over speed.
                            `define FIVE_STATE_EN
===========================================================================*/
//`define FIVE_STATE_EN


/*===========================================================================
2. Fast Mode (No Pipeline – 3 Cycles per Instruction)
When THREE_STATE_EN is defined, the CPU still operates without pipelining but optimizes t-
-he execution flow to merge or shorten certain stages, allowing each instruction to compl-
-ete in just 3 clock cycles. While this is faster than the basic mode, it is still sequen-
-tial and does not overlap instructions like a true pipeline.
                            `define THREE_STATE_EN
===========================================================================*/
//`define THREE_STATE_EN


/*===========================================================================
3. Advanced Mode (Pipelined – 1 Cycle per Instruction)
When FIVES_PIPELINE_EN is defined, the CPU uses pipeline techniques to overlap multiple i-
-nstruction stages (e.g., while one instruction is in Decode, another can be in Execute).
With this approach, once the pipeline is filled, the CPU can achieve a throughput of 1 in-
-struction per clock cycle, significantly improving performance compared to non-pipelined
modes.
                            `define FIVES_PIPELINE_EN
===========================================================================*/
`define FIVES_PIPELINE_EN
`define DEBUG_EN



`ifdef FIVE_STATE_EN
    `undef THREE_STATE_EN
    `undef FIVES_PIPELINE_EN
`endif
`ifdef THREE_STATE_EN
    `undef FIVE_STATE_EN
    `undef FIVES_PIPELINE_EN
`endif
`ifdef FIVES_PIPELINE_EN
    `undef THREE_STATE_EN
    `undef FIVE_STATE_EN
`endif

//`define CLKDIV8

localparam  INSTR_FIRST     =   32'h00000000;
localparam  INSTR_NOP       =   32'h00000013;
localparam  VALUE_RESET32   =   32'h00000000;
localparam  VALUE_RESET     =   0;


//for CPU 5 instruction cycles
localparam  IF              =   0;
localparam  ID              =   1;
localparam  EX              =   2;
localparam  MEM             =   3;
localparam  WB              =   4;

localparam  Fetchoh         =   5'b00001 <<  IF;        //Fetch
localparam  Decodeoh        =   5'b00001 <<  ID;        //Decode
localparam  Executeoh       =   5'b00001 <<  EX;        //Execute
localparam  MemoryAoh       =   5'b00001 <<  MEM;       //Memory access
localparam  WriteBoh        =   5'b00001 <<  WB;        //Write back


//for CPU 3 instruction cycles
localparam  FD              =   0;                      //Fetch and decodes
localparam  EM              =   1;                      //Execute and access memory
localparam  WM              =   2;                      //Write back reg or mem

localparam  FetADec         =   5'b00001 <<  FD;
localparam  ExeAMem         =   5'b00001 <<  EM;
localparam  WRegOMem        =   5'b00001 <<  WM;

wire    [31:0 ] memd_ldata, memins_rdata;

reg     [31:0 ] processor_data;
wire    [31:0 ] rdata_uart;
wire            s0_sel_mem;
wire            s3_sel_uart;
wire clksys;

`ifdef FIVE_STATE_EN
    always @(*) begin
        case({s3_sel_uartshift1, s0_sel_memshift1})
            2'b01: processor_data = memd_ldata;
            2'b10: processor_data = rdata_uart;
            default: processor_data = 32'h00000000;
        endcase
    end
`endif
`ifdef THREE_STATE_EN
    always @(*) begin
        case({s3_sel_uartshift1, s0_sel_memshift1})
            2'b01: processor_data = memd_ldata;
            2'b10: processor_data = rdata_uart;
            default: processor_data = 32'h00000000;
        endcase
    end
`endif
`ifdef FIVES_PIPELINE_EN
    reg s0_sel_memshift1;
    reg s3_sel_uartshift1;

    always @(posedge clksys) begin
        s0_sel_memshift1 <= s0_sel_mem;
        s3_sel_uartshift1 <= s3_sel_uart;
    end

    always @(*) begin
        case({s3_sel_uartshift1, s0_sel_memshift1})
            2'b01: processor_data = memd_ldata;
            2'b10: processor_data = rdata_uart;
            default: processor_data = 32'h00000000;
        endcase
    end
`endif

    
`ifndef CLKDIV8
    assign clksys = clk;
`endif
`ifdef CLKDIV8
    Gowin_CLKDIV your_instance_name(
        .clkout(clksys), //output clkout
        .hclkin(clk), //input hclkin
        .resetn(rst) //input resetn
    );
`endif


/*-------------------------------------------------
The device_select module decodes the upper 4 bits of a 32-bit address to generate select signals for different peripherals in an RV32I-based system. 
Each peripheral (memory/*, GPIO, I2C, UART, PWM, timer, SPI ...//) is mapped to a unique address region. 
When the address matches a region, the corresponding select signal is asserted high.
--------------------------------------------------*/
reg     [31:0 ] mem_addr;
wire    [31:0 ] memins_addr;
Device_select DS(
    .addr(mem_addr),
    .s0_sel_mem(s0_sel_mem),
    .s3_sel_uart(s3_sel_uart)
);



/*-------------------------------------------------
This module decodes 32-bit RV32I instructions by extracting fields such as opcode, rd, rs1, rs2, funct3, funct7, and immediate values. 
It identifies the instruction format (R, I, S, B, U, J) and generates control signals for execution units.
-------------------------------------------------*/
reg     [31:0 ] instr_data;
wire    [31:0 ] Immediate;
wire            insALUImm, insALUReg, insLUI,
                insAUIPC, insJAL, insJALR,
                insBRA, insLOAD, insSTORE,
                insSYS, insFENCE;
wire    [ 2:0 ] funct3;
wire    [ 7:0 ] funct3oh;
wire    [ 6:0 ] funct7;
wire    [ 4:0 ] regrs2;
wire    [ 4:0 ] regrs1;
wire    [ 4:0 ] regrd;
wire            en_rs1;
wire            en_rs2;
wire            en_rd;
wire    [ 3:0 ] pred;
wire    [ 3:0 ] succ;
wire    [11:0 ] indcsr;


RV32_Decoder Maindecoder(
    .instr_data         (instr_data),
    .Immediate          (Immediate),
    .insALUImm          (insALUImm),
    .insALUReg          (insALUReg),
    .insLUI             (insLUI),
    .insAUIPC           (insAUIPC),
    .insJAL             (insJAL),
    .insJALR            (insJALR),
    .insBRA             (insBRA),
    .insLOAD            (insLOAD),
    .insSTORE           (insSTORE),
    .insSYS             (insSYS),
    .insFENCE           (insFENCE),
    .funct3             (funct3),
    .funct3oh           (funct3oh),
    .funct7             (funct7),
    .regrs2             (regrs2),
    .regrs1             (regrs1),
    .regrd              (regrd),
    .en_rs1             (en_rs1),
    .en_rs2             (en_rs2),
    .en_rd              (en_rd),
    .pred               (pred),
    .succ               (succ),
    .indcsr             (indcsr)
);



/*---------------------------------------------------
Instruction Memory:
A read-only memory that stores machine instructions. 
The CPU provides an address and receives the corresponding 32-bit instruction. 
It is used during the instruction fetch stage and does not support write operations.

Data Memory:
A read-write memory used to store program data. 
The CPU can read from or write to a specified address. 
It is accessed during the load/store stage of instruction execution.
---------------------------------------------------*/
//reg     [31:0 ] mem_addr, memint_addr;
reg     [31:0 ] memd_sdata;
wire    [31:0 ] memd_sdatafn;       assign  memd_sdatafn    =   memd_sdata << (mem_addr[1:0]*8);    //Shift data for store byte or haftw
reg             memd_lready; 
wire            memins_read;
wire    [ 3:0 ] memd_mask;
reg             memd_senable = 1'b0;

wire            flag_branch;
reg             predict_taken2  =   VALUE_RESET;

reg     [31:0 ] PCnext          =   INSTR_FIRST;
reg     [31:0 ] PCnext_pred     =   INSTR_FIRST;
wire            wrong_predfast; 
assign          wrong_predfast  =   insBRA & (flag_branch != predict_taken2);
wire    [31:0 ] memins_rdata_pred;

Instruction_memory #(
//    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/FW RV32I/firmware/firmware.hex"),
    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware_instr.hex"),
    .SIZE       (4096)  //4Kb
) ins_mem(
    .clk            (clksys),
    .mem_addr       (memins_addr),
    .mem_addrpred   (PCnext),
    .mem_rdata      (memins_rdata),
    .mem_rdata_pred (memins_rdata_pred),
    .mem_renable    (memins_read)
);

wire    [ 3:0 ] memd_maskfn_mem;
wire            memd_lenfn_mem;
wire    [ 3:0 ] memd_maskfn_uart;
wire            memd_lenfn_uart;
`ifdef THREE_STATE_EN
    assign memd_maskfn_mem  =   memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTORE}};
    assign memd_lenfn_mem   =   memd_lready & insLOAD;
`endif
`ifdef FIVE_STATE_EN
    assign memd_maskfn_mem  =   memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTORE}};
    assign memd_lenfn_mem   =   memd_lready & insLOAD;
`endif
`ifdef FIVES_PIPELINE_EN
reg insSTOREshift1;
reg insLOADshift1;
    assign memd_maskfn_mem  =   memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTOREshift1}};
    assign memd_lenfn_mem   =   memd_lready & insLOADshift1;

    assign memd_maskfn_uart =   memd_mask & {4{memd_senable}} & {4{s3_sel_uart}} & {4{insSTOREshift1}};
    assign memd_lenfn_uart  =   memd_lready & insLOADshift1;
`endif

Data_memory #(
    //.MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/FW RV32I/firmware/firmware.hex"),
    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware_data.hex"),
    .SIZE       (4096)  //16Kb
) data_mem(
    .clk        (clksys),
    .mem_addr   ({4'h0, mem_addr[27:0]}),
    .mem_ldata  (memd_ldata),
    .mem_sdata  (memd_sdatafn),    
    .mem_lenable(memd_lenfn_mem),
    .mem_mask   (memd_maskfn_mem)
);

wire    [31:0 ] mem_addrforl; 
//load, store 8bit, 16bit or 32bit
wire    lsByte ;// =   (funct3[1:0] == 2'b00);
wire    lsHaftW;// =   (funct3[0]);
wire    lsWord ;// =   (funct3[1]);
wire    lsSign ;// =   (!funct3[2]);
`ifdef THREE_STATE_EN
    assign lsByte           =   (funct3[1:0] == 2'b00);
    assign lsHaftW          =   (funct3[0]);
    assign lsWord           =   (funct3[1]);
    assign lsSign           =   (!funct3[2]);

    assign mem_addrforl     =   mem_addr;
`endif
`ifdef FIVE_STATE_EN
    assign lsByte           =   (funct3[1:0] == 2'b00);
    assign lsHaftW          =   (funct3[0]);
    assign lsWord           =   (funct3[1]);
    assign lsSign           =   (!funct3[2]);

    assign mem_addrforl     =   mem_addr;
`endif
`ifdef FIVES_PIPELINE_EN
reg     [ 2:0 ] funct3shift1;
reg     [ 2:0 ] funct3shift2;
reg     [31:0 ] mem_addrshift1;
always @(posedge clksys) mem_addrshift1 <= mem_addr;
    assign lsByte           =   (funct3shift1[1:0] == 2'b00);
    assign lsHaftW          =   (funct3shift1[0]);
    assign lsWord           =   (funct3shift1[1]);
    assign lsSign           =   (!funct3shift1[2]);

    assign mem_addrforl     =   mem_addrshift1;
`endif

//gen mem_mask and make load data 8bit, 16bit or 32bit
assign          memd_mask   = 
                    lsWord  ?   4'b1111                         :
                    lsHaftW ?   (mem_addr[1]?4'b1100:4'b0011)  :
                    lsByte  ?   (mem_addr[1]?(mem_addr[0]?4'b1000:4'b0100):
                                              (mem_addr[0]?4'b0010:4'b0001)):
                    4'b0;

wire    [31:0]  byte_data;
wire    [31:0]  halfw_data;

assign byte_data = (mem_addrforl[1:0] == 2'b00) ? processor_data[7:0]   :
                   (mem_addrforl[1:0] == 2'b01) ? processor_data[15:8]  :
                   (mem_addrforl[1:0] == 2'b10) ? processor_data[23:16] :
                                              processor_data[31:24] ;
assign halfw_data = mem_addrforl[1] ? processor_data[31:16] : processor_data[15:0];


wire    [31:0]  mem_ldmask;     //Mem load data(mask)

assign mem_ldmask = lsWord  ? processor_data :
                    lsHaftW ? (lsSign ? {{16{halfw_data[15]}}, halfw_data[15:0]} :
                                        {16'b0, halfw_data[15:0]}) :
                    lsByte  ? (lsSign ? {{24{byte_data[7]}}, byte_data[7:0]} :
                                        {24'b0, byte_data[7:0]}) :
                    32'b0;

/*---------------------------------------------------------
The Arithmetic Logic Unit performs integer operations such as addition, subtraction, bitwise logic, and shifts. 
It also supports comparison operations used for branch instructions, including signed and unsigned comparisons. 
The ALU receives two operands and an operation code, and outputs the result along with a branch condition cr if needed.
---------------------------------------------------------*/
wire    [31:0 ] result_ALU;
//wire            flag_branch;
wire    [31:0 ] data_rs1;
wire    [31:0 ] data_rs1fn;
wire    [31:0 ] data_rs2;   
wire    [31:0 ] data_rs2fn;
wire    [31:0 ] temphz;

//Hazard RAW
reg     [ 4:0 ] regrd_shifthz1  =   5'h00;          //Du lai rd 1 chu ki truoc de so sanh. Phục vụ phát hiện hazard 1 chu kì
reg     [ 4:0 ] regrd_shifthz2  =   5'h00;          //Du lai rd 2 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì
wire            isRAW_Hazardrs1_1cyc_forWB    ;    //phat hien hazard 1 chu kì
wire            isRAW_Hazardrs2_1cyc_forWB;    //phat hien hazard 1 chu kì
wire            isRAW_Hazardrs1_2cyc_forWB;    //phat hien hazard 1 chu kì
wire            isRAW_Hazardrs2_2cyc_forWB;    //phat hien hazard 1 chu kì
reg             en_rdhz1        =   VALUE_RESET;
reg             en_rdhz2        =   VALUE_RESET;
//Result of ins
reg     [31:0 ] result;
wire    [31:0 ] resulthz_1cyc;
wire    [31:0 ] resulthz_2cyc;

`ifdef THREE_STATE_EN
    assign data_rs2fn   =   (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;
    assign data_rs1fn   =   data_rs1;
`endif
`ifdef FIVE_STATE_EN
    assign data_rs2fn   =   (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;
    assign data_rs1fn   =   data_rs1;
`endif
`ifdef FIVES_PIPELINE_EN
wire    [31:0 ] data_rs1hz_1cyc;
wire    [31:0 ] data_rs2hz_1cyc;
wire    [31:0 ] temp_hz;

wire    [31:0 ] data_rs1hz_2cyc;
wire    [31:0 ] data_rs2hz_2cyc;
    assign temp_hz          =   (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;
    assign data_rs2hz_1cyc  =   (isRAW_Hazardrs2_1cyc_forWB)?resulthz_1cyc:temp_hz;
    assign data_rs1hz_1cyc  =   (isRAW_Hazardrs1_1cyc_forWB)?resulthz_1cyc:data_rs1;

    assign data_rs2hz_2cyc  =   (isRAW_Hazardrs2_2cyc_forWB)?resulthz_2cyc:temp_hz;
    assign data_rs1hz_2cyc  =   (isRAW_Hazardrs1_2cyc_forWB)?resulthz_2cyc:data_rs1;

    assign data_rs2fn       =   (isRAW_Hazardrs2_1cyc_forWB)?data_rs2hz_1cyc:data_rs2hz_2cyc;
    assign data_rs1fn       =   (isRAW_Hazardrs1_1cyc_forWB)?data_rs1hz_1cyc:data_rs1hz_2cyc;
`endif




ALU_unit ALU(
    .isALUimm   (insALUImm),
    .isALUreg   (insALUReg),
    .isBranch   (insBRA),
    .funct3oh   (funct3oh),
    .funct7     (funct7),
    .rs1        (data_rs1fn),
    .rs2        (data_rs2fn),
    .result     (result_ALU),
    .correct    (flag_branch)
);



/*---------------------------------------------------------
This block contains 32 general-purpose 32-bit registers (x0–x31).
Register x0 is hardwired to zero and cannot be modified.
The register file supports two read ports and one write port, allowing simultaneous access to rs1, rs2, and rd operands. 
It is used for storing intermediate and final results during instruction execution.
---------------------------------------------------------*/
wire    [31:0 ] data_desfn;
wire    [ 4:0 ] regrdfn;
//reg     [31:0 ] result;
reg     [31:0 ] data_des;
reg     [ 4:0 ] regrd_shiftpl   =   5'h00;
`ifdef THREE_STATE_EN
    assign data_desfn = (insLOAD)?mem_ldmask:result;
    assign regrdfn = regrd;
`endif
`ifdef FIVE_STATE_EN
    assign data_desfn = data_des;
    assign regrdfn = regrd;
`endif
`ifdef FIVES_PIPELINE_EN
    assign data_desfn = data_des;
    assign regrdfn = regrd_shiftpl;
`endif


//wire    [31:0 ] data_rs1;
//wire    [31:0 ] data_rs2;
wire    [31:0 ] data_rs1pred;
wire    [ 4:0 ] regrs1pred;
reg             data_valid;

reg     [ 4:0 ] rd_lpl;
reg     [31:0 ] data_des_lpl;
reg             data_validlpl;


Registers_unit Regunit(
    .clk            (clksys), 
    .rs1            (regrs1),
    .rs1pred        (regrs1pred),
    .rs2            (regrs2),
    .rd             (regrdfn),
    .data_des       (data_desfn),
    .data_valid     (data_valid),
    .data_rs1       (data_rs1),
    .data_rs2       (data_rs2),
    .data_rs1pred   (data_rs1pred)
);



/*---------------------------------------------------------
The Universal Asynchronous Receiver Transmitter (UART) enables asynchronous serial communication by transmitting and receiving data over tx and rx lines.
It supports configurable baud rate and basic control features, providing status flags to indicate transmission and reception states.
---------------------------------------------------------*/
uart_ip uart_unit(
    .clk        (clksys),
    .rst        (rst),

    .waddr      ({4'h0, mem_addr[27:0]}),
    .wdata      (memd_sdata),
    .wen        (s3_sel_uart & (|memd_mask) & insSTOREshift1),///////////////////////////
    .wstrb      (memd_mask),
    .wready     (),
    .raddr      ({4'h0, mem_addr[27:0]}),
    .ren        (s3_sel_uart & memd_lready & insLOADshift1),/////////////////////////////
    .rdata      (rdata_uart),
    .rvalid     (),

    .o_uart_tx  (uart_tx),
    .i_uart_rx  (uart_rx)
);


reg     [31:0 ] PC      =   INSTR_FIRST;    //The program counter (PC) is a 32-bit register that holds the address of the current instruction being executed. 
                                            //After each instruction, the PC is typically incremented by 4 to point to the next instruction. 
//reg     [31:0 ] PCnext  =   INSTR_FIRST;
reg     [ 4:0 ] state   =   5'b00001;
reg     [31:0 ] load_data;
wire            insALU  =   insALUImm || insALUReg;


/*------------------------------------------------------------------------------------------
/*----------------------------------------------------------
This processor is organized into five stages:
    1. IF (Instruction Fetch): Fetches the next instruction from instruction memory.
    2. ID (Instruction Decode): Decodes the instruction, reads registers, and generates control signals.
    3. EX (Execute): Performs arithmetic or logic operations in the ALU, or computes branch targets.
    4. MEM (Memory Access): Accesses data memory for load/store instructions.
    5. WB (Write Back): Writes results back to the register file.
No pileline
------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
`ifdef FIVE_STATE_EN
    assign memins_read = state[IF]; //only Instruction Fetch
    assign memins_addr = PC;

    always @(posedge clksys) begin
        if(!rst) begin
            PC <= INSTR_FIRST;
            PCnext <= INSTR_FIRST;
            data_valid <= VALUE_RESET; 
            memd_lready <= VALUE_RESET;
            memd_senable <= VALUE_RESET;
            data_des <= VALUE_RESET32;
            memd_sdata <= VALUE_RESET32;
            state <= Fetchoh;
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                state[IF]   : begin
                    data_valid <= 1'b0;
                    //PC <= PCnext;
                    state <= Decodeoh;    //ID
                end
                state[ID]   : begin
                    instr_data <= memins_rdata;
                    state <= Executeoh;   //EX
                end
                state[EX]   : begin
                    (*parallel_case*)
                    case(1'b1)
                        insBRA: begin
                            PCnext <= flag_branch ? PC + Immediate : PC+4;
                        end
                        insLOAD: begin
                            memd_lready <= 1'b1;
                            mem_addr <= data_rs1 + Immediate;
                        end
                        insSTORE: begin
                            mem_addr <= data_rs1 + Immediate;
                        end
                        insALU: begin
                            result <= result_ALU;
                        end
                        insLUI: begin
                            result <= Immediate;
                        end
                        insAUIPC: begin 
                            result <= PC + Immediate;
                        end
                        insJAL: begin
                            result <= PC + 4;
                            PCnext <= PC + Immediate;
                        end
                        insJALR: begin
                            result <= PC + 4;
                            PCnext <= data_rs1 + Immediate;
                        end
                        default: state <= Fetchoh;
                    endcase
                    state <= (insLOAD || insSTORE) ? MemoryAoh : WriteBoh;
                    if (!(insBRA||insJAL||insJALR)) PCnext <= PC + 4;
                end
                state[MEM]  : begin
                    if(insLOAD) begin
                        memd_lready <= 1'b0;
                        load_data <= mem_ldmask;
                    end
                    else begin //insSTORE
                        memd_sdata <= data_rs2;
                        memd_senable <= 1'b1;
                    end
                    state <= WriteBoh;
                    
                end
                state[WB]   : begin
                    memd_senable <= 1'b0;
                    case(1'b1)
                        insLOAD: begin
                            data_des <= mem_ldmask;
                            data_valid <= 1'b1;
                        end
                        insALU | insLUI | insAUIPC | insJAL | insJALR: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                    endcase
                    state <= Fetchoh;
                    PC <= PCnext;               //thay doi PC de fetch
                end
            endcase
        end
    end
`endif


/*------------------------------------------------------------------------------------------
/*----------------------------------------------------------
This processor is organized into three states:
    1. FD (Fetch & Decode): Fetches the instruction, decodes it, and reads registers.
    2. EM (Execute & Memory): Executes ALU operations or accesses data memory.
    3. WM (Write Back): Writes results back to the register file or memory.
No pipeline
------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
`ifdef THREE_STATE_EN
    assign memins_read = 1'b1; //always read
    assign memins_addr = PC;

    always @(*) begin
        instr_data = memins_rdata;
    end

    always @(posedge clksys) begin
        if(!rst) begin
            PC <= INSTR_FIRST;
            PCnext <= INSTR_FIRST;
            data_valid <= VALUE_RESET; 
            memd_lready <= VALUE_RESET;
            memd_senable <= VALUE_RESET;
            data_des <= VALUE_RESET32;
            memd_sdata <= VALUE_RESET32;
            state <= FetADec;
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                state[EM]: begin
                    (*parallel_case*)
                    case(1'b1)
                        insBRA: begin
                            PCnext <= flag_branch ? PC + Immediate : PC+4;
                        end
                        insLOAD: begin
                            memd_lready <= 1'b1;
                            mem_addr <= data_rs1 + Immediate;
                        end
                        insSTORE: begin
                            mem_addr <= data_rs1 + Immediate;
                            memd_sdata <= data_rs2;
                            memd_senable <= 1'b1;
                        end
                        insALU: begin
                            result <= result_ALU;
                            data_valid <= 1'b1;
                        end
                        insLUI: begin
                            result <= Immediate;
                            data_valid <= 1'b1;
                        end
                        insAUIPC: begin 
                            result <= PC + Immediate;
                            data_valid <= 1'b1;
                        end
                        insJAL: begin
                            result <= PC + 4;
                            PCnext <= PC + Immediate;
                            data_valid <= 1'b1;
                        end
                        insJALR: begin
                            data_valid <= 1'b1;
                            result <= PC + 4;
                            PCnext <= data_rs1 + Immediate;
                        end
                    endcase    
                    if (!(insBRA||insJAL||insJALR)) PCnext <= PC + 4;
                    state <= WRegOMem;
                end
                state[WM]: begin
                    case(1'b1)
                        insLOAD: begin
                            memd_lready <= 1'b0;
                            data_valid <= 1'b1;
                        end
                        insSTORE: begin
//                            memd_sdata <= data_rs2;
                            memd_senable <= 1'b0;
                        end
                        insALU | insLUI | insAUIPC | insJAL | insJALR: begin
                            //data_des <= result;
                            data_valid <= 1'b0;
                        end
                    endcase 
                    state <= FetADec;
                    PC <= PCnext;
                end
                state[FD]: begin
                    data_valid <= 1'b0;
                    state <= ExeAMem;
                end
            endcase   
        end
    end
`endif


/*------------------------------------------------------------------------------------------
/*----------------------------------------------------------
This processor is organized into five pipeline stages:
    1. IF (Instruction Fetch): Fetches the instruction from instruction memory.
    2. ID (Instruction Decode): Decodes the instruction and reads operands from registers.
    3. EX (Execute): Performs ALU operations or computes branch targets.
    4. MEM (Memory Access): Accesses data memory for load/store instructions.
    5. WB (Write Back): Writes results back to the register file.
Pipelined architecture enables instruction-level parallelism.
Each instruction passes through these stages in successive clock cycles.
After the pipeline is filled, one instruction is completed per clock cycle.
------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
`ifdef FIVES_PIPELINE_EN
reg     IDen                    =   VALUE_RESET;
reg     EXen                    =   VALUE_RESET;
reg     MEMen                   =   VALUE_RESET;
reg     WBen                    =   VALUE_RESET;
reg     Fetchena                =   VALUE_RESET;    
reg     [31:0 ] PCBraoJum       =   VALUE_RESET32;
reg     [31:0 ] temp            =   VALUE_RESET32;

reg     [ 2:0 ] CntfMem         =   3'd0;           //counter for memory access
reg     [31:0 ] Store_datapl1   =   VALUE_RESET32;
reg     [31:0 ] Store_datapl2   =   VALUE_RESET32;

reg     [31:0 ] mem_addrup      =   VALUE_RESET32;  //địa chỉ tới trước 1 chu kì cho memory access
reg     [31:0 ] result_shiftpl  =   VALUE_RESET32;
reg     [31:0 ] return_addrpred =   VALUE_RESET32;
reg     [31:0 ] choose1_1       =   VALUE_RESET32;
reg     [31:0 ] choose1_2       =   VALUE_RESET32;
reg     [31:0 ] choose1_3       =   VALUE_RESET32;
reg     [31:0 ] choose2_1       =   VALUE_RESET32;
reg     [31:0 ] choose2_2       =   VALUE_RESET32;
reg     [31:0 ] choose2_3       =   VALUE_RESET32;

reg             ck              =   VALUE_RESET;
reg             ck1             =   VALUE_RESET;
reg             ck2             =   VALUE_RESET;
reg             wrong_pred      =   VALUE_RESET;
reg             predict_taken1  =   VALUE_RESET;
//reg             predict_taken2  =   VALUE_RESET;
reg             predict_taken3  =   VALUE_RESET;


reg     [ 4:0 ] regrd_shiftpl1  =   5'h00;
reg     [ 4:0 ] regrd_shiftpl2  =   5'h00;
reg     [ 4:0 ] regrd_shiftpl3  =   5'h00;
//reg     [ 4:0 ] regrd_shiftpl   =   5'h00;

reg             insALUImmshift1, insALURegshift1, insLUIshift1,
                insAUIPCshift1, insJALshift1, insJALRshift1,
                insBRAshift1, //insLOADshift1, insSTOREshift1,
                insSYSshift1, insFENCEshift1;
reg             insALUImmshift2, insALURegshift2, insLUIshift2,
                insAUIPCshift2, insJALshift2, insJALRshift2,
                insBRAshift2, insLOADshift2, insSTOREshift2,
                insSYSshift2, insFENCEshift2;

reg     [31:0 ] PCshift1        =   VALUE_RESET;


//reg     [ 4:0 ] regrd_shifthz1  =   5'h00;          //Du lai rd 1 chu ki truoc de so sanh. Phục vụ phát hiện hazard 1 chu kì
//reg     [ 4:0 ] regrd_shifthz2  =   5'h00;          //Du lai rd 2 chu ki truoc de so sanh. Phục vụ phát hiện hazard 2 chu kì
//reg             isRAW_Hazardrs1_1cyc_forWB =   VALUE_RESET;    //phat hien hazard 1 chu kì
//reg             isRAW_Hazardrs2_1cyc_forWB =   VALUE_RESET;    //phat hien hazard 1 chu kì
//reg             isRAW_Hazardrs1_2cyc_forWB =   VALUE_RESET;    //phat hien hazard 2 chu kì
//reg             isRAW_Hazardrs2_2cyc_forWB =   VALUE_RESET;    //phat hien hazard 2 chu kì

wire            isRAW_Hazardrs1_1cyc_forSTORE;  //Phat hien hazard 1 chu kì cho store
wire            isRAW_Hazardrs2_1cyc_forSTORE;  //Phat hien hazard 1 chu kì cho store
wire            isRAW_Hazardrs1_2cyc_forSTORE;  //Phat hien hazard 2 chu kì cho store
wire            isRAW_Hazardrs2_2cyc_forSTORE;  //Phat hien hazard 2 chu kì cho store


`ifdef DEBUG_EN
    
`endif



/*-------------------------------------------------------------------------
This module implements a 2-bit bimodal branch predictor using a 256-entry table of saturating counters.
It predicts branch direction based on the most significant bit (MSB) of each counter. On each clock cy-
-cle, the predictor updates its state according to the actual branch outcome if update_en is high.
-------------------------------------------------------------------------*/
reg             update_BHT      =   VALUE_RESET;
reg             actual_taken    =   VALUE_RESET;
wire            predict_taken;  
wire            insBRApred; 
reg     [31:0 ] pc_pred_in      =   VALUE_RESET32;
reg             enUpdate        =   VALUE_RESET;
branch_predictor BRA_PRED(
    .clk             (clksys),
    .rst             (rst), 
    .insBRA          (insBRApred),
    .pc_pred_in      (pc_pred_in),           //Dư đoán lệnh kế tiếp      
    .update_en       (update_BHT),    
    .actual_taken    (actual_taken),  
    .predict_taken   (predict_taken)   //0: not take, 1: take
);

/*-------------
Decoder for predictor
-------------*/
wire            insJALpred;
wire            insJALRpred;

wire    [31:0 ] Immediatepred;
RV32_Decoder Decoder_for_pred(
    .instr_data         (memins_rdata_pred),
    .Immediate          (Immediatepred),
    .insJAL             (insJALpred),
    .insJALR            (insJALRpred),
    .insBRA             (insBRApred),
    .regrs1             (regrs1pred),
    .regrd              (regrdpred)
);
 
//ALU_unit ALU(
//    .isALUimm   (insALUImmshift1),
//    .isALUreg   (insALURegshift1),
//    .isBranch   (insBRAshift1),
//    .funct3oh   (funct3oh),
//    .funct7     (funct7),
//    .rs1        (data_rs1fn),
//    .rs2        (data_rs2fn),
//    .result     (result_ALU),
//    .correct    (flag_branch)
//);


/*----------------------------------------------------------
    1. IF (Instruction Fetch): Fetches the instruction from instruction memory.
    2. ID (Instruction Decode): Decodes the instruction and reads operands from registers.
    3. EX (Execute): Performs ALU operations or computes branch targets.
    4. MEM (Memory Access): Accesses data memory for load/store instructions.
    5. WB (Write Back): Writes results back to the register file.
----------------------------------------------------------*/
    //Fetch
    
    assign memins_addr = PC;
    assign memins_read = 1'b1;          //Luôn đọc
    always @(posedge clksys) begin
        if(!rst) begin 
            PC <= INSTR_FIRST;
            PCnext <= INSTR_FIRST;
            Fetchena <= VALUE_RESET;
            pc_pred_in <= VALUE_RESET32;
            ck <= VALUE_RESET;
            IDen <= VALUE_RESET;
            update_BHT <= VALUE_RESET;
            predict_taken1 <= VALUE_RESET;
            predict_taken2 <= VALUE_RESET;
        end
        else begin
            PC <= PCnext;   PCshift1 <= PC;
            Fetchena <= 1'b1;
            PCnext <= PCnext + 4;
            PCnext_pred <= (predict_taken1)?choose2_1+4:choose1_1;
            pc_pred_in <= PCnext + 4;
            update_BHT <= 1'b0;
            ck <= 1'b0;

            (*parallel_case*)
            case(1'b1) 
                insJALpred: begin
                    PCnext <= PCnext + Immediatepred;
                    //return_addrpred <= PC + 4;
                end
                insJALRpred: begin
                    PCnext <= data_rs1pred + Immediatepred;
                    //return_addrpred <= PC + 4;
                end
                insBRApred: begin
                    PCnext <= (predict_taken)?PCnext + Immediatepred:PCnext+4;
                    ck <= 1'b1;
                end
            endcase

            IDen <= 1'b1;
            if(wrong_predfast) begin
                IDen <= 1'b0;                                       //Lập lại pipeline, xóa kết quả cũ
                //PC <= (predict_taken3)?choose2_3+4:choose1_3;         //Phục hồi nhánh đúng
                //PCnext <= (predict_taken3)?choose2_3+8:choose1_3+4;
            end 

            if(wrong_predfast) begin
                PCnext <= PCnext_pred;
            end

            if(enUpdate) begin
                //Update BHT
                update_BHT <= 1'b1;
                actual_taken <= (wrong_pred)?!predict_taken3:predict_taken3;
                pc_pred_in <= choose2_3;                          //Địa chỉ lệnh branch
            end

            choose1_1 <= PCnext + Immediatepred;    choose1_2 <= choose1_1; choose1_3 <= choose1_2;
            choose2_1 <= PCnext;                choose2_2 <= choose2_1; choose2_3 <= choose2_2;

            predict_taken1 <= predict_taken;    
            predict_taken2 <= predict_taken1;
            predict_taken3 <= predict_taken2;

            ck1 <= ck;       
            ck2 <= ck1;
        end
    end


    //Decode
    always @(*) begin
        instr_data = memins_rdata;
    end
    
    assign isRAW_Hazardrs1_1cyc_forWB   =   !insLOAD & insLOADshift1 & ((en_rdhz1&&en_rs1)?(regrs1 == regrd_shifthz1):1'b0);
    assign isRAW_Hazardrs2_1cyc_forWB   =   !insLOAD & insLOADshift1 & ((en_rdhz1&&en_rs2)?(regrs2 == regrd_shifthz1):1'b0);
    assign isRAW_Hazardrs1_2cyc_forWB   =   (en_rdhz2&&en_rs1)?(regrs1 == regrd_shifthz2):1'b0;
    assign isRAW_Hazardrs2_2cyc_forWB   =   (en_rdhz2&&en_rs2)?(regrs2 == regrd_shifthz2):1'b0;

    assign isRAW_Hazardrs1_1cyc_forSTORE=   isRAW_Hazardrs1_1cyc_forWB;//(en_rdhz1&&en_rs1)?(regrs1 == regrd_shifthz1):1'b0;
    assign isRAW_Hazardrs2_1cyc_forSTORE=   isRAW_Hazardrs2_1cyc_forWB;//(en_rdhz1&&en_rs2)?(regrs2 == regrd_shifthz1):1'b0;
    assign isRAW_Hazardrs1_2cyc_forSTORE=   isRAW_Hazardrs1_2cyc_forWB;//(en_rdhz2&&en_rs1)?(regrs1 == regrd_shifthz2):1'b0;
    assign isRAW_Hazardrs2_2cyc_forSTORE=   isRAW_Hazardrs2_2cyc_forWB;//(en_rdhz2&&en_rs2)?(regrs2 == regrd_shifthz2):1'b0;

    assign resulthz_1cyc                =   result;
    assign resulthz_2cyc                =   result_shiftpl;

    always @(posedge clksys or negedge IDen) begin
        if (!rst || !IDen) begin
            EXen <= 1'b0;

            insALUImmshift1 <= 1'b0;    insALURegshift1 <= 1'b0;    insLUIshift1    <= 1'b0;
            insAUIPCshift1  <= 1'b0;    insJALshift1    <= 1'b0;    insJALRshift1   <= 1'b0;
            insBRAshift1    <= 1'b0;    insLOADshift1   <= 1'b0;    insSTOREshift1  <= 1'b0;
            insSYSshift1    <= 1'b0;    insFENCEshift1  <= 1'b0;

            insALUImmshift2 <= 1'b0;    insALURegshift2 <= 1'b0;    insLUIshift2    <= 1'b0;
            insAUIPCshift2  <= 1'b0;    insJALshift2    <= 1'b0;    insJALRshift2   <= 1'b0;
            insBRAshift2    <= 1'b0;    insLOADshift2   <= 1'b0;    insSTOREshift2  <= 1'b0;
            insSYSshift2    <= 1'b0;    insFENCEshift2  <= 1'b0;
        end
        else begin
            //instr_data <= memins_rdata;
            EXen <= 1'b1;
//            Store_datapl1 <= data_rs2;          //Lưu trễ 1 chu kì
//            Store_datapl2 <= Store_datapl1;     //Lưu trễ 2 chu kì để dành cho memory access

            regrd_shiftpl1 <= regrd;            //Lưu trễ 1 chu kì
            regrd_shiftpl2 <= regrd_shiftpl1;   //Lưu trẽ 2 chu kì
            regrd_shiftpl3 <= regrd_shiftpl2;   //Lưu trễ 3 chu kì

            //Hazard RAW(Read after write)
            regrd_shifthz1 <= regrd;
            regrd_shifthz2 <= regrd_shifthz1;
            en_rdhz1 <= en_rd;
            en_rdhz2 <= en_rdhz1;
//            isRAW_Hazardrs1_1cyc_forWB <= (en_rdhz1&&en_rs1)?(regrs1 == regrd_shifthz1):1'b0;
//            isRAW_Hazardrs2_1cyc_forWB <= (en_rdhz1&&en_rs2)?(regrs2 == regrd_shifthz1):1'b0;
//            isRAW_Hazardrs1_2cyc_forWB <= (en_rdhz2&&en_rs1)?(regrs1 == regrd_shifthz2):1'b0;
//            isRAW_Hazardrs2_2cyc_forWB <= (en_rdhz2&&en_rs2)?(regrs2 == regrd_shifthz2):1'b0;
            

            // Lưu trễ 1 chu kỳ 
            insALUImmshift1  <= insALUImm;      insALURegshift1  <= insALUReg;
            insLUIshift1     <= insLUI;         insAUIPCshift1   <= insAUIPC;
            insJALshift1     <= insJAL;         insJALRshift1    <= insJALR;
            insBRAshift1     <= insBRA;         insLOADshift1    <= insLOAD;
            insSTOREshift1   <= insSTORE;       insSYSshift1     <= insSYS;
            insFENCEshift1   <= insFENCE;

            funct3shift1     <= funct3;         funct3shift2     <= funct3shift1;

            // Lưu trễ 2 chu kỳ 
            insALUImmshift2  <= insALUImmshift1;insALURegshift2  <= insALURegshift1;
            insLUIshift2     <= insLUIshift1;   insAUIPCshift2   <= insAUIPCshift1;
            insJALshift2     <= insJALshift1;   insJALRshift2    <= insJALRshift1;
            insBRAshift2     <= insBRAshift1;   insLOADshift2    <= insLOADshift1;
            insSTOREshift2   <= insSTOREshift1; insSYSshift2     <= insSYSshift1;
            insFENCEshift2   <= insFENCEshift1;

            
        end
    end


    //Execute
    always @(*) begin//-> giúp load sớm hơn 1 chu kì còn store cứ dữ nguyên
        if(!rst || !EXen) begin
        end
        else begin
            (*parallel_case*)
            case(1'b1)
                insLOAD: begin
                    mem_addr =      (isRAW_Hazardrs1_1cyc_forSTORE)?result + Immediate:
                                    (isRAW_Hazardrs1_2cyc_forSTORE)?result_shiftpl + Immediate:
                                                                    data_rs1 + Immediate;
                    memd_lready = 1'b1;
                end
                insSTOREshift1: begin
                    mem_addr =      (isRAW_Hazardrs1_1cyc_forSTORE)?result + Immediate:
                                    (isRAW_Hazardrs1_2cyc_forSTORE)?result_shiftpl + Immediate:
                                                                    data_rs1 + Immediate;
                    memd_sdata =    (isRAW_Hazardrs2_1cyc_forSTORE)?result:
                                    (isRAW_Hazardrs2_2cyc_forSTORE)?result_shiftpl:
                                                                    data_rs2;
                    memd_senable = 1'b1;
                end
                default: begin
                    memd_lready = 1'b0;
                    memd_senable = 1'b0;
                end
            endcase
        end
    end

    always @(posedge clksys or negedge EXen) begin
        if(!rst || !EXen) begin
            MEMen <= 1'b0;
            wrong_pred <= VALUE_RESET;
            PCBraoJum <= VALUE_RESET;
            mem_addrup <= VALUE_RESET32;
            result <= VALUE_RESET32;
        end
        else begin
            wrong_pred <= 1'b0; enUpdate <= 1'b0;
//            memd_lready <= 1'b0;
//            memd_senable <= 1'b0;
            (*parallel_case*)
            case(1'b1)
                insBRA: begin
                    //PCnext <= flag_branch ? PCshift1 + Immediate : PCshift1+4; //was predicted
                    if(flag_branch != predict_taken2) begin
                        wrong_pred <= 1'b1;
                    end
                    enUpdate <= 1'b1;
                end
                insLOAD: begin
//                    mem_addr <= (isRAW_Hazardrs1_1cyc_forSTORE)?result + Immediate:
//                                  (isRAW_Hazardrs1_2cyc_forSTORE)?result_shiftpl + Immediate:
//                                                                  data_rs1 + Immediate;
//                    memd_lready <= 1'b1;
                end
                insSTORE: begin
//                    mem_addr <=   (isRAW_Hazardrs1_1cyc_forSTORE)?result + Immediate:
//                                  (isRAW_Hazardrs1_2cyc_forSTORE)?result_shiftpl + Immediate:
//                                                                  data_rs1 + Immediate;
//                    memd_sdata <=    (isRAW_Hazardrs2_1cyc_forSTORE)?result:
//                                     (isRAW_Hazardrs2_2cyc_forSTORE)?result_shiftpl:
//                                                                     data_rs2;
//                    memd_senable <= 1'b1;
                end
                insALU: begin
                    result <= result_ALU;
                end
                insLUI: begin
                    result <= Immediate;
                end
                insAUIPC: begin 
                    result <= PCshift1 + Immediate;
                end
                insJAL: begin
                    result <= PCshift1 + 4;                 
                    //PCnext <= PCshift1 + Immediate;         //was predicted
                end
                insJALR: begin
                    result <= PCshift1 + 4;                
                    //PCnext <= data_rs1 + Immediate;   //was predicted
                end
            endcase

            result_shiftpl <= result;
            MEMen <= 1'b1;
        end
    end


    //Memory access
    always @(posedge clksys or negedge MEMen) begin
        if(!rst || !MEMen) begin
            WBen <= VALUE_RESET;
//            load_data <= VALUE_RESET32;
//            memd_sdata <= VALUE_RESET32;
        end
        else begin
            if(insLOADshift1) begin
                //load_data <= mem_ldmask;
                //mem_addr <= mem_addrup;
                //memd_lready <= 1'b1;
            end
            else if(insSTOREshift1) begin
                //memd_sdata <= Store_datapl1;
                //mem_addr <= mem_addrup;
                //memd_senable <= 1'b1;
            end
            else begin
//                memd_lready <= 1'b0;
//                memd_senable <= 1'b0;
            end
            WBen <= 1'b1;
        end
    end


    //Write back
    always @(posedge clksys or negedge WBen) begin
        if(!rst || !WBen) begin
            data_des <= VALUE_RESET32;
            regrd_shiftpl <= 5'h00;
            data_valid <= VALUE_RESET;
        end
        else begin
            case(1'b1)
                insLOADshift2: begin
                    data_des <= mem_ldmask;
                    regrd_shiftpl <= regrd_shiftpl2;
                    data_valid <= 1'b1;
                end
                insALUImmshift2 | insALURegshift2 | insLUIshift2 | 
                insAUIPCshift2  | insJALshift2    | insJALRshift2   : begin
                    data_des <= result_shiftpl;
                    regrd_shiftpl <= regrd_shiftpl2;
                    data_valid <= 1'b1;
                end
                default: data_valid <= 1'b0;
            endcase
        end
    end
`endif

endmodule