ENTRY(_start);

PHDRS {
    text_seg   PT_LOAD FLAGS(5); /* PF_R|PF_X: Code */
    rodata_seg PT_LOAD FLAGS(4); /* PF_R: Read-only data */
    dram_seg   PT_LOAD FLAGS(6); /* PF_R|PF_W: RAM */
}

MEMORY {
    FLASH    (rx) : ORIGIN = 0x42000020, LENGTH = 0x400000 - 0x20
    RAM     (rwx) : ORIGIN = 0x40800000, LENGTH = 512K
}

_stack_size = DEFINED(_stack_size) ? _stack_size : 64K;

SECTIONS {
    .rodata_desc : {
        KEEP(*(.flash.appdesc))
        . = ALIGN(256);
    } > FLASH : text_seg

    .trap : ALIGN(256) {
        KEEP(*(.text.trap))
    } > FLASH : text_seg

    .text : ALIGN(4) {
        KEEP(*(.text.init))
        *(.text .text.*)
    } > FLASH : text_seg

    /* Aligned to 64KB to prevent MMU physical/virtual overlap */
    .rodata : ALIGN(0x10000) {
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
    } > FLASH : rodata_seg

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.data .data.*);
        *(.sdata .sdata.*);
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > RAM AT > FLASH : dram_seg

    PROVIDE(_sidata = LOADADDR(.data));

    .bss (NOLOAD) : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > RAM : dram_seg

    /DISCARD/ : {
        *(.eh_frame .eh_frame_hdr)
        *(.riscv.attributes)
        *(.comment)
        *(.note.*)
    }
}


/* --- Safety Assertions --- */

/* 1. Ensure the Stack doesn't overflow into BSS/Data */
ASSERT(_ebss + _stack_size <= ORIGIN(RAM) + LENGTH(RAM), "RAM OVERFLOW: Not enough RAM for the requested stack size");

/* 2. RISC-V requires 4-byte word alignment for variables. */
ASSERT((_sdata % 4) == 0, "BUG: .data start is not 4-byte aligned");
ASSERT((_edata % 4) == 0, "BUG: .data end is not 4-byte aligned");
ASSERT((_sbss % 4) == 0, "BUG: .bss start is not 4-byte aligned");
ASSERT((_ebss % 4) == 0, "BUG: .bss end is not 4-byte aligned");

/* 3. Ensure the RISC-V global pointer is defined for relative addressing */
ASSERT(DEFINED(__global_pointer$), "BUG: RISC-V __global_pointer$ is missing");

/* 4. main's stack is hardcoded in boot.s as `li sp, 0x40860000`, NOT derived
      from _stack_size/RAM above — this assert is the only thing standing
      between task .bss growth (STACKS array etc.) and silently colliding
      with that hardcoded boot stack, since check #1 only protects against
      running off the *end* of RAM, not against this separate fixed address. */
ASSERT(_ebss <= 0x40860000, "BUG: .bss has grown into the hardcoded boot stack region (0x40860000)");
