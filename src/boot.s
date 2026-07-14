.section .flash.appdesc, "a"
.global esp_app_desc
.align 4
esp_app_desc:
    .word  0xABCD5432       
    .word  0                
    .word  0, 0             
    .ascii "1.0.0"          
    .zero  27
    .ascii "trobbio"        
    .zero  25
    .zero  16               
    .zero  16               
    .ascii "v5.1"           
    .zero  28
    .zero  32               
    .hword 0                
    .hword 0xFFFF           
    .byte  0                
    .zero  3                
    .zero  72               

.section .text.init, "ax"
.global _start
_start:
    li sp, 0x40860000

    csrw mie, zero      
    csrw mstatus, 0x8   

    la t0, _vector_table
    ori t0, t0, 1
    csrw mtvec, t0

    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    la a0, _sidata
    la a1, _sdata
    la a2, _edata
    bgeu a1, a2, 2f
1:
    lw t0, 0(a0)
    sw t0, 0(a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b
2:
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 4f
3:
    sw zero, 0(a0)
    addi a0, a0, 4
    bltu a0, a1, 3b
4:
    call main

.section .text.trap, "ax"
.global _vector_table
.global _trap_handler

.balign 256
_vector_table:
    .option push
    .option norvc
    /* Generate 32 explicit jumps for all RISC-V Traps (0-31) */
    .rept 32
    j _trap_handler
    .endr
    .option pop

.balign 4
_trap_handler:
    addi sp, sp, -64
    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t2, 12(sp)
    sw a0, 16(sp)
    sw a1, 20(sp)
    sw a2, 24(sp)
    sw a3, 28(sp)
    sw a4, 32(sp)
    sw a5, 36(sp)
    sw a6, 40(sp)
    sw a7, 44(sp)
    sw t3, 48(sp)
    sw t4, 52(sp)
    sw t5, 56(sp)
    sw t6, 60(sp)

    call rust_trap_handler

    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t2, 12(sp)
    lw a0, 16(sp)
    lw a1, 20(sp)
    lw a2, 24(sp)
    lw a3, 28(sp)
    lw a4, 32(sp)
    lw a5, 36(sp)
    lw a6, 40(sp)
    lw a7, 44(sp)
    lw t3, 48(sp)
    lw t4, 52(sp)
    lw t5, 56(sp)
    lw t6, 60(sp)
    addi sp, sp, 64
    mret
