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
    # Frame layout, 128 bytes total (16-byte aligned), one word each:
    #  0: ra    4: t0    8: t1   12: t2
    # 16: a0   20: a1   24: a2   28: a3
    # 32: a4   36: a5   40: a6   44: a7
    # 48: t3   52: t4   56: t5   60: t6
    # 64: s0   68: s1   72: s2   76: s3
    # 80: s4   84: s5   88: s6   92: s7
    # 96: s8  100: s9  104: s10 108: s11
    # 112: mepc                 116-124: unused padding
    #
    # Extended from the caller-saved-only frame the cooperative design
    # needed: preemption can happen at ANY instruction, not just a
    # voluntary call boundary, so EVERY register plus the resume address
    # (mepc) has to be captured -- not just what the ABI's callee-saved
    # convention would otherwise protect for free on a normal return.
    addi sp, sp, -128
    sw ra,  0(sp)
    sw t0,  4(sp)
    sw t1,  8(sp)
    sw t2,  12(sp)
    sw a0,  16(sp)
    sw a1,  20(sp)
    sw a2,  24(sp)
    sw a3,  28(sp)
    sw a4,  32(sp)
    sw a5,  36(sp)
    sw a6,  40(sp)
    sw a7,  44(sp)
    sw t3,  48(sp)
    sw t4,  52(sp)
    sw t5,  56(sp)
    sw t6,  60(sp)
    sw s0,  64(sp)
    sw s1,  68(sp)
    sw s2,  72(sp)
    sw s3,  76(sp)
    sw s4,  80(sp)
    sw s5,  84(sp)
    sw s6,  88(sp)
    sw s7,  92(sp)
    sw s8,  96(sp)
    sw s9,  100(sp)
    sw s10, 104(sp)
    sw s11, 108(sp)
    csrr t0, mepc
    sw t0, 112(sp)

    # rust_trap_handler(current_sp: usize) -> usize (next_sp).
    # Stage 1: always returns the same sp it was given (see arch::sched),
    # so a0 == sp here and the following line is a no-op in practice --
    # kept in place now so Stage 2 needs zero asm changes, only the
    # Rust-side scheduling decision changes.
    mv a0, sp
    call rust_trap_handler
    mv sp, a0

    lw t0, 112(sp)
    csrw mepc, t0
    lw ra,  0(sp)
    lw t0,  4(sp)
    lw t1,  8(sp)
    lw t2,  12(sp)
    lw a0,  16(sp)
    lw a1,  20(sp)
    lw a2,  24(sp)
    lw a3,  28(sp)
    lw a4,  32(sp)
    lw a5,  36(sp)
    lw a6,  40(sp)
    lw a7,  44(sp)
    lw t3,  48(sp)
    lw t4,  52(sp)
    lw t5,  56(sp)
    lw t6,  60(sp)
    lw s0,  64(sp)
    lw s1,  68(sp)
    lw s2,  72(sp)
    lw s3,  76(sp)
    lw s4,  80(sp)
    lw s5,  84(sp)
    lw s6,  88(sp)
    lw s7,  92(sp)
    lw s8,  96(sp)
    lw s9,  100(sp)
    lw s10, 104(sp)
    lw s11, 108(sp)
    addi sp, sp, 128
    mret
