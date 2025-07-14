# Toolchain settings
RISCV_TOOLCHAIN_DIR = /home/do_phong/opt/riscv32/bin
GCC         = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-gcc
OBJDUMP     = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-objdump
OBJCOPY     = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-objcopy

# Project structure
SRC_DIR     = .
BUILD_DIR   = .
INCLUDE_DIR = inc
MEM_SIZE    = 1024

# Source files
C_SRC       = $(SRC_DIR)/main.c
ASM_SRC     = $(SRC_DIR)/start.S
LDSCRIPT    = $(SRC_DIR)/sections.lds

# Object and output files
C_OBJ       = $(BUILD_DIR)/main.o
ASM_OBJ     = $(BUILD_DIR)/start.o
ELF_FILE    = $(BUILD_DIR)/firmware.elf
BIN_FILE    = $(BUILD_DIR)/firmware.bin
HEX_FILE    = $(BUILD_DIR)/firmware.hex
MAP_FILE    = $(BUILD_DIR)/firmware.map
DUMP_FILE   = $(BUILD_DIR)/dumpfile

# Build rules
all: $(HEX_FILE) $(DUMP_FILE)

$(HEX_FILE): $(BIN_FILE) makehex.py
	python3 makehex.py $< $(MEM_SIZE) > $@

$(BIN_FILE): $(ELF_FILE)
	$(OBJCOPY) -O binary $< $@

$(DUMP_FILE): $(ELF_FILE)
	$(OBJDUMP) -d $< > $@

$(ELF_FILE): $(C_OBJ) $(ASM_OBJ) $(LDSCRIPT)
	$(GCC) -Og -mabi=ilp32 -march=rv32i -ffreestanding -nostdlib -o $@ \
		-Wl,--build-id=none,-Bstatic,-T,$(LDSCRIPT),-Map,$(MAP_FILE),--strip-debug \
		$(ASM_OBJ) $(C_OBJ) -lgcc

$(C_OBJ): $(C_SRC)
	$(GCC) -c -I$(INCLUDE_DIR)/ -mabi=ilp32 -march=rv32i -Og --std=c99 -ffreestanding -nostdlib -o $@ $<

$(ASM_OBJ): $(ASM_SRC)
	$(GCC) -c -mabi=ilp32 -march=rv32i -o $@ $<

clean:
	rm -f $(BUILD_DIR)/*.o $(BUILD_DIR)/*.elf $(BUILD_DIR)/*.bin \
	      $(BUILD_DIR)/*.hex $(BUILD_DIR)/*.map $(BUILD_DIR)/dumpfile
