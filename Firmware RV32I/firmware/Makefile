# Toolchain settings
RISCV_TOOLCHAIN_DIR = /home/do_phong/opt/riscv32/bin
GCC         = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-gcc
OBJDUMP     = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-objdump
OBJCOPY     = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-objcopy

# Project structure
SRC_DIR     = .
BUILD_DIR   = .
INCLUDE_DIR = inc
INSTR_MEM_SIZE = 1024   # 1KB
DATA_MEM_SIZE  = 4096   # 4KB

# Source files
C_SRC       = $(SRC_DIR)/main.c
ASM_SRC     = $(SRC_DIR)/start.S
LDSCRIPT    = $(SRC_DIR)/sections.lds

# Object and output files
C_OBJ       = $(BUILD_DIR)/main.o
ASM_OBJ     = $(BUILD_DIR)/start.o
ELF_FILE    = $(BUILD_DIR)/firmware.elf
MAP_FILE    = $(BUILD_DIR)/firmware.map
DUMP_FILE   = $(BUILD_DIR)/dumpfile

# New binaries and HEX for instruction and data
INSTR_BIN   = $(BUILD_DIR)/firmware_instr.bin
DATA_BIN    = $(BUILD_DIR)/firmware_data.bin
INSTR_HEX   = $(BUILD_DIR)/firmware_instr.hex
DATA_HEX    = $(BUILD_DIR)/firmware_data.hex

# Build rules  
all: $(INSTR_HEX) $(DATA_HEX) $(DUMP_FILE)


        # .bin -> .hex
$(INSTR_HEX): $(INSTR_BIN) makehex.py
	python3 makehex.py $< $(INSTR_MEM_SIZE) > $@

$(DATA_HEX): $(DATA_BIN) makehex.py
	python3 makehex.py $< $(DATA_MEM_SIZE) > $@

        # Extract binary sections
$(INSTR_BIN): $(ELF_FILE)
	$(OBJCOPY) -O binary -j .text $< $@

$(DATA_BIN): $(ELF_FILE)
	$(OBJCOPY) -O binary -j .rodata -j .data $< $@

        # Dump file
$(DUMP_FILE): $(ELF_FILE)
	$(OBJDUMP) -d $< > $@

        # Build ELF
$(ELF_FILE): $(C_OBJ) $(ASM_OBJ) $(LDSCRIPT)
	$(GCC) -Og -mabi=ilp32 -march=rv32i -ffreestanding -nostdlib -o $@ \
		-Wl,--build-id=none,-Bstatic,-T,$(LDSCRIPT),-Map,$(MAP_FILE),--strip-debug \
		$(ASM_OBJ) $(C_OBJ) -lgcc

        # Compile C and ASM
$(C_OBJ): $(C_SRC)
	$(GCC) -c -I$(INCLUDE_DIR)/ -mabi=ilp32 -march=rv32i -Og --std=c99 -ffreestanding -nostdlib -o $@ $<
$(ASM_OBJ): $(ASM_SRC)
	$(GCC) -c -mabi=ilp32 -march=rv32i -o $@ $<

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.elf \
	      $(BUILD_DIR)/*.bin $(BUILD_DIR)/*.hex \
	      $(BUILD_DIR)/*.map $(DUMP_FILE)
