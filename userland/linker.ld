/* Linker script for EonOS user programs: a static, non-PIE executable
   at 0x400000 with page-aligned, separately-protected segments (code is
   R+X, read-only data is R, data/bss is R+W), which is what the kernel's
   ELF loader expects. */

ENTRY(_start)

PHDRS
{
    text   PT_LOAD FLAGS(5);   /* R + X */
    rodata PT_LOAD FLAGS(4);   /* R     */
    data   PT_LOAD FLAGS(6);   /* R + W */
}

SECTIONS
{
    . = 0x400000;

    .text : {
        *(.text .text.*)
    } :text

    . = ALIGN(4096);
    .rodata : {
        *(.rodata .rodata.*)
    } :rodata

    . = ALIGN(4096);
    .data : {
        *(.data .data.*)
    } :data

    .bss : {
        *(.bss .bss.* COMMON)
    } :data

    /DISCARD/ : {
        *(.note .note.* .comment .eh_frame)
    }
}
