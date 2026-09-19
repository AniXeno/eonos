/* EonOS linker script — higher-half kernel, Limine boot protocol */

OUTPUT_FORMAT(elf64-x86-64)
ENTRY(_start)

PHDRS
{
    text    PT_LOAD    FLAGS((1 << 0) | (1 << 2)) ; /* Execute + Read */
    rodata  PT_LOAD    FLAGS((1 << 2)) ;             /* Read only */
    data    PT_LOAD    FLAGS((1 << 1) | (1 << 2)) ;  /* Write + Read */
    dynamic PT_DYNAMIC FLAGS((1 << 1) | (1 << 2)) ;
}

SECTIONS
{
    /* Kernel lives in the top 2GiB of the address space so it can use
       the kernel code model and be mapped alongside a higher-half direct map. */
    . = 0xffffffff80000000;

    /* Section-boundary symbols the VMM uses to map the kernel image with
       per-section permissions (text RX, rodata RO, data/bss RW, all NX
       except text). Addresses here are link-time *virtual* addresses;
       the VMM turns them into physical ones via KernelAddressRequest. */
    PROVIDE(__kernel_text_start = .);
    .text : {
        *(.text .text.*)
    } :text
    PROVIDE(__kernel_text_end = .);

    . += CONSTANT(MAXPAGESIZE);

    PROVIDE(__kernel_rodata_start = .);
    .rodata : {
        *(.rodata .rodata.*)

        /* Limine boot-protocol requests must sit between the start and end
           markers, in that order, and must survive --gc-sections. */
        . = ALIGN(8);
        KEEP(*(.requests_start_marker))
        KEEP(*(.requests))
        . = ALIGN(8);
        KEEP(*(.requests_end_marker))
    } :rodata
    PROVIDE(__kernel_rodata_end = .);

    . += CONSTANT(MAXPAGESIZE);

    PROVIDE(__kernel_data_start = .);
    .data : {
        *(.data .data.*)
    } :data

    .dynamic : {
        *(.dynamic)
    } :data :dynamic

    .bss : {
        *(COMMON)
        *(.bss .bss.*)
    } :data
    PROVIDE(__kernel_data_end = .);

    /DISCARD/ : {
        *(.eh_frame)
        *(.note .note.*)
        *(.comment)
    }
}
