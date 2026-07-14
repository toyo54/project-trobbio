ENTRY(_start);

PHDRS {
    flash_seg PT_LOAD FLAGS(5); /* PF_R|PF_X: descriptor, rodata, and code */
    dram_seg  PT_LOAD FLAGS(6); /* PF_R|PF_W: initialized data -> RAM */
}

MEMORY {
    APP_DESC (r)  : ORIGIN = 0x42000020, LENGTH = 0x1E0
    FLASH    (rx) : ORIGIN = 0x42000200, LENGTH = 0x400000 - 0x200
    RAM     (rwx) : ORIGIN = 0x40800000, LENGTH = 512K
}

_stack_size = DEFINED(_stack_size) ? _stack_size : 64K;

SECTIONS {
    .rodata_desc : {
        KEEP(*(.flash.appdesc))
    } > APP_DESC : flash_seg

    .trap : ALIGN(256) {
        KEEP(*(.text.trap))
    } > FLASH : flash_seg

    .text : ALIGN(4) {
        KEEP(*(.text.init))
        *(.text .text.*)
    } > FLASH : flash_seg

    .rodata : ALIGN(4) {
        *(.rodata .rodata.*)
    } > FLASH : flash_seg

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
}
