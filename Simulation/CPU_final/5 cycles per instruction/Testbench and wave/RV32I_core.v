module RV32I_core(
    input           clk,    
    input           rst,

    input           uart_rx,
    output          uart_tx
);

`define FIVE_STATE_EN

localparam  INSTR_FIRST     =   32'h00000000;
localparam  VALUE_RESET32   =   32'h00000000;
localparam  VALUE_RESET     =   0;

localparam  IF              =   0;
localparam  ID              =   1;
localparam  EX              =   2;
localparam  MEM             =   3;
localparam  WB              =   4;

localparam  Fetchoh            =   5'b0001 <<  IF;
localparam  Decodeoh            =   5'b0001 <<  ID;
localparam  Executeoh            =   5'b0001 <<  EX;
localparam  MemoryAoh           =   5'b0001 <<  MEM;
localparam  WriteBoh            =   5'b0001 <<  WB;

wire    [31:0 ] memd_ldata, memins_rdata;

reg     [31:0 ] processor_data;
wire    [31:0 ] rdata_uart;
wire            s0_sel_mem;
wire            s3_sel_uart;
always @(*) begin
    case({s3_sel_uart, s0_sel_mem})
        2'b01: processor_data = memd_ldata;
        2'b10: processor_data = rdata_uart;
        default: processor_data = 32'h00000000;
    endcase
end

wire clksys;
    Gowin_CLKDIV your_instance_name(
        .clkout(clksys), //output clkout
        .hclkin(clk), //input hclkin
        .resetn(rst) //input resetn
    );


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
reg     [31:0 ] memd_sdata, memins_wdata;
wire    [31:0 ] memd_sdatafn;               assign  memd_sdatafn    =   memd_sdata << (mem_addr[1:0]*8);
reg             memd_lready; 
wire            memins_read;
wire    [ 3:0 ] memd_mask, memins_mask;
reg             memd_senable = 1'b0;

Instruction_memory #(
//    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/FW RV32I/firmware/firmware.hex"),
    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware_instr.hex"),
    .SIZE       (4096)  //4Kb
) ins_mem(
    .clk        (clksys),
    .mem_addr   (memins_addr),
    .mem_rdata  (memins_rdata),
    .mem_wdata  (memins_wdata <<mem_addr[1:0]*8), //don't use
    .mem_renable(memins_read),
    .mem_mask   (memins_mask)   //don't use
);

Data_memory #(
    //.MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/FW RV32I/firmware/firmware.hex"),
    .MEM_FILE   ("C:/Users/PHONG/OneDrive - ptit.edu.vn/Desktop/Project_I2C/firmware/firmware_data.hex"),
    .SIZE       (4096)  //16Kb
) data_mem(
    .clk        (clksys),
    .mem_addr   ({4'h0, mem_addr[27:0]}),
    .mem_ldata  (memd_ldata),
    .mem_sdata  (memd_sdatafn),    //Shift data for store byte or haftw
    .mem_lenable(memd_lready & insLOAD),
    .mem_mask   (memd_mask & {4{memd_senable}} & {4{s0_sel_mem}} & {4{insSTORE}})
);

//load, store 8bit, 16bit or 32bit
wire    lsByte  =   (funct3[1:0] == 2'b00);
wire    lsHaftW =   (funct3[0]);
wire    lsWord  =   (funct3[1]);
wire    lsSign  =   (!funct3[2]);
//gen mem_mask and make load data 8bit, 16bit or 32bit
assign          memd_mask   = 
                    lsWord  ?   4'b1111                         :
                    lsHaftW ?   (mem_addr[1]?4'b1100:4'b0011)  :
                    lsByte  ?   (mem_addr[1]?(mem_addr[0]?4'b1000:4'b0100):
                                              (mem_addr[0]?4'b0010:4'b0001)):
                    4'b0;

wire    [31:0]  byte_data;
wire    [31:0]  halfw_data;

assign byte_data = (mem_addr[1:0] == 2'b00) ? processor_data[7:0]   :
                   (mem_addr[1:0] == 2'b01) ? processor_data[15:8]  :
                   (mem_addr[1:0] == 2'b10) ? processor_data[23:16] :
                                               processor_data[31:24];
assign halfw_data = mem_addr[1] ? processor_data[31:16] : processor_data[15:0];


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
wire            flag_branch;
wire    [31:0 ] data_rs1;
wire    [31:0 ] data_rs2;   
wire    [31:0 ] data_rs2fn; assign  data_rs2fn  =  (insALUImm)?(((!funct3[1])&funct3[0])?regrs2:Immediate):data_rs2;

ALU_unit ALU(
    .isALUimm   (insALUImm),
    .isALUreg   (insALUReg),
    .isBranch   (insBRA),
    .funct3oh   (funct3oh),
    .funct7     (funct7),
    .rs1        (data_rs1),
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
reg     [31:0 ] data_des;
//wire    [31:0 ] data_rs1;
//wire    [31:0 ] data_rs2;
reg             data_valid;

Registers_unit Regunit(
    .clk        (clksys), 
    .rs1        (regrs1),
    .rs2        (regrs2),
    .rd         (regrd),
    .data_des   (data_des),
    .data_valid (data_valid),   
    .data_rs1   (data_rs1),
    .data_rs2   (data_rs2)
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
    .wen        (s3_sel_uart & (|memd_mask) & insSTORE),
    .wstrb      (memd_mask),
    .wready     (),
    .raddr      ({4'h0, mem_addr[27:0]}),
    .ren        (s3_sel_uart & memd_lready & insLOAD),
    .rdata      (rdata_uart),
    .rvalid     (),

    .o_uart_tx  (uart_tx),
    .i_uart_rx  (uart_rx)
);


/*----------------------------------------------------------
This processor is organized into five stages:
    1. IF (Instruction Fetch): Fetches the next instruction from instruction memory.
    2. ID (Instruction Decode): Decodes the instruction, reads registers, and generates control signals.
    3. EX (Execute): Performs arithmetic or logic operations in the ALU, or computes branch targets.
    4. MEM (Memory Access): Accesses data memory for load/store instructions.
    5. WB (Write Back): Writes results back to the register file.
This pipelined structure improves instruction throughput by allowing multiple instructions to be processed in parallel, each at a different stage.
----------------------------------------------------------*/
reg     [31:0 ] PC      =   INSTR_FIRST;    //The program counter (PC) is a 32-bit register that holds the address of the current instruction being executed. 
                                            //After each instruction, the PC is typically incremented by 4 to point to the next instruction. 
reg     [31:0 ] PCnext  =   INSTR_FIRST;
reg     [ 4:0 ] state   =   5'b00001;
reg     [31:0 ] load_data;
reg     [31:0 ] result;
wire            insALU  =   insALUImm || insALUReg;

//assign          mem_addr = data_rs1 + Immediate;

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
                        insALU: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                        insLUI: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                        insAUIPC: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                        insJAL: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                        insJALR: begin
                            data_des <= result;
                            data_valid <= 1'b1;
                        end
                    endcase
                    state <= Fetchoh;
                    PC <= PCnext;               //thay doi PC de fetch
                end
//                default: 
            endcase
        end
    end
`endif


`ifdef PIPELINE_EN
    //PIPELINE HERE
`else //SIMPLE_EFFECTIVE_EN
    //SIMPLE_EFFECTIVE_EN
`endif


endmodule