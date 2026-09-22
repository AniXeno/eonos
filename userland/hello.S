# Tiny standalone user program for exercising `exec /hello`.

.intel_syntax noprefix

.section .rodata
message: .ascii "hello from /hello (a separate ELF process)\n"
.equ message_len, . - message

.section .text
.global _start
_start:
    mov edi, 1                  # stdout
    lea rsi, [rip + message]
    mov edx, message_len
    mov eax, 1                  # write
    syscall

    xor edi, edi                # exit status 0
    mov eax, 60                 # exit
    syscall
    ud2                         # exit must not return
