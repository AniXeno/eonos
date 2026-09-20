# EonOS /init -- the first user-mode program.
#
# Freestanding x86-64 assembly, no libc. It talks to the kernel with the
# Linux-style syscall convention EonOS uses (number in RAX, arguments in
# RDI/RSI/RDX, result in RAX):
#
#     1  write(fd, buf, len)
#    24  sched_yield()
#    39  getpid()
#    60  exit(status)
#
# On entry RSP points at the System V initial stack: argc, then the argv
# pointers.

.intel_syntax noprefix

.section .rodata
msg_hello:  .ascii "init: hello from user space (ring 3)!\n"
.equ msg_hello_len, . - msg_hello
msg_argv0:  .ascii "init: argv[0] = "
.equ msg_argv0_len, . - msg_argv0
msg_pid:    .ascii "init: my pid is "
.equ msg_pid_len, . - msg_pid
msg_yield:  .ascii "init: yielding the CPU\n"
.equ msg_yield_len, . - msg_yield
msg_bye:    .ascii "init: done, exiting with status 0\n"
.equ msg_bye_len, . - msg_bye
newline:    .ascii "\n"

.section .text
.global _start
_start:
    mov  rbx, rsp                       # keep the initial stack pointer

    lea  rsi, [rip + msg_hello]
    mov  edx, msg_hello_len
    call print

    # argv[0] is at [rsp + 8]; print it followed by a newline.
    lea  rsi, [rip + msg_argv0]
    mov  edx, msg_argv0_len
    call print
    mov  rsi, [rbx + 8]
    xor  edx, edx
1:  cmp  byte ptr [rsi + rdx], 0
    je   2f
    inc  rdx
    jmp  1b
2:  call print
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print

    # getpid(), printed in decimal.
    lea  rsi, [rip + msg_pid]
    mov  edx, msg_pid_len
    call print
    mov  eax, 39
    syscall
    call print_u64
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print

    # Give the CPU away a few times.
    mov  r12d, 3
3:  lea  rsi, [rip + msg_yield]
    mov  edx, msg_yield_len
    call print
    mov  eax, 24
    syscall
    dec  r12d
    jnz  3b

    lea  rsi, [rip + msg_bye]
    mov  edx, msg_bye_len
    call print

    xor  edi, edi                       # status 0
    mov  eax, 60
    syscall
    ud2                                 # exit() never returns

# print: write RDX bytes at RSI to stdout.
print:
    mov  edi, 1
    mov  eax, 1
    syscall
    ret

# print_u64: write RAX to stdout in decimal. Preserves RBX.
print_u64:
    push rbx
    sub  rsp, 32
    lea  rbx, [rsp + 32]                # digits are written backwards from here
    mov  ecx, 10
1:  xor  edx, edx
    div  rcx                            # rax = rax / 10, rdx = rax % 10
    add  dl, '0'
    dec  rbx
    mov  [rbx], dl
    test rax, rax
    jnz  1b
    lea  rdx, [rsp + 32]
    sub  rdx, rbx                       # number of digits
    mov  rsi, rbx
    call print
    add  rsp, 32
    pop  rbx
    ret