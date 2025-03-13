OUTPUT_ARCH("riscv")

SECTIONS
{
    /* Entry point */
    . = 0x70000000;

    /* Code and Read-Only Data */
    .text :
    {
        *(.text.entry)   /* Entry point */
        *(.text .text.*) /* Main code */
    } 

    /* Read-Only Data */
    .rodata :
    {
        *(.rodata .rodata.*)
    }

    /* Initialized Data */
    .data :
    {
        *(.data .data.*)
    }

    /* BSS (Uninitialized Data) */
    .bss :
    {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    }

    /* Define _end symbol */
    _end = .;

    /* Global pointer for small data */
    __global_pointer$ = . + 0x800; /* Typical offset for global pointer */


    . = 0x80000000; /* Address for model binary data */

    .model_data :
    {
        model_data_start = .; /* Symbol marking the start of the data */
        KEEP(*(.model_data))  /* Keep section even with optimizations */
        model_data_end = .;   /* Symbol marking the end of the data */
    }


	. = 0x90000000; /* Address for tokenizer binary data */

    .tokenizer_data :
    {
        tokenizer_data_start = .; /* Symbol marking the start of the second data */
        KEEP(*(.tokenizer_data))  /* Keep section even with optimizations */
        tokenizer_data_end = .;   /* Symbol marking the end of the second data */
    }
}
