//.equ MEM_SIZE, 0x20001000  
.section .text
.globl _start
_start:
	# lui	a5,0x40000
	la   sp, __stack_top

    # Clear .bss
    la   t1, _sbss
    la   t2, _ebss
    li   t3, 0
1:  beq  t1, t2, 2f
    sw   t3, 0(t1)
    addi t1, t1, 4
    j    1b
2:

	call main
	ebreak
	