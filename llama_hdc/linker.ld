SECTIONS
{
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
