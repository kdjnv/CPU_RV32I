/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.
*/
/*
MEMORY {
	
	mem : ORIGIN = 0x00000000, LENGTH = 1k
}

SECTIONS {
	.memory : {
		. = 0x000000;
		start*(.text);
		*(.text);
		*(*);
		end = .;
		. = ALIGN(4);
	} > mem
}
*/
MEMORY {
    instr_mem (rx) : ORIGIN = 0x00000000, LENGTH = 1K
    data_mem  (rw) : ORIGIN = 0x20000000, LENGTH = 4K
}

SECTIONS {
    .text : {
        _start = .;
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > instr_mem

    .data : {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > data_mem

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > data_mem
}
