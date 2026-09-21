# EonOS /init -- now a small interactive shell ("eonsh").
#
# Freestanding x86-64 assembly, no libc. It talks to the kernel with the
# Linux-style syscall convention EonOS uses (number in RAX, arguments in
# RDI/RSI/RDX/R10, result in RAX):
#
#     0  read(fd, buf, len)          -- fd 0 only; one line, canonical
#                                       mode, echoed by the kernel, from
#                                       the PS/2 keyboard and/or COM1.
#     1  write(fd, buf, len)
#    24  sched_yield()
#    39  getpid()
#    60  exit(status)
#  9001  eonos_list_files(buf, len)  -- newline-separated initramfs
#                                       listing. Stopgap for `ls` until
#                                       there's a real filesystem/fd
#                                       table.
#  9002  eonos_read_file(name, name_len, buf, buf_len)
#                                     -- whole-file read by path.
#                                       Stopgap for `cat`.
#  9003  eonos_uptime_ms()           -- milliseconds since boot.
#
# Commands: ls, cat <file>, echo <text>, clear, uptime, pid, help, exit.
# No pipes, no scripting -- just enough to poke around from a prompt.

.intel_syntax noprefix

.section .bss
.balign 8
line_buf:   .skip 128
LINE_CAP = 128
file_buf:   .skip 4096
FILE_CAP = 4096
numbuf:     .skip 24                # enough for any 64-bit value in decimal

.section .rodata
msg_banner:   .ascii "EonOS shell -- type 'help' for commands\n"
.equ msg_banner_len, . - msg_banner
prompt:       .ascii "eonsh> "
.equ prompt_len, . - prompt
msg_help:     .ascii "commands: ls, cat <file>, echo <text>, clear, uptime, pid, help, exit\n"
.equ msg_help_len, . - msg_help
msg_unknown:  .ascii "unknown command: "
.equ msg_unknown_len, . - msg_unknown
msg_notfound: .ascii "cat: no such file\n"
.equ msg_notfound_len, . - msg_notfound
msg_toobig:   .ascii "cat: file too large to display\n"
.equ msg_toobig_len, . - msg_toobig
msg_uptime_suffix: .ascii " ms\n"
.equ msg_uptime_suffix_len, . - msg_uptime_suffix
newline:      .ascii "\n"
clear_seq:    .ascii "\x1b[2J\x1b[H"    # CSI 2J (clear) + CSI H (home)
.equ clear_seq_len, . - clear_seq
cmd_ls:       .ascii "ls"
cmd_help:     .ascii "help"
cmd_exit:     .ascii "exit"
cmd_clear:    .ascii "clear"
cmd_uptime:   .ascii "uptime"
cmd_pid:      .ascii "pid"
cmd_cat:      .ascii "cat "   # note trailing space: "cat" alone is not this
cmd_echo:     .ascii "echo "  # same convention; bare "echo" prints nothing

.section .text
.global _start
_start:
    lea  rsi, [rip + msg_banner]
    mov  edx, msg_banner_len
    call print

.Lloop:
    lea  rsi, [rip + prompt]
    mov  edx, prompt_len
    call print

    lea  rsi, [rip + line_buf]
    mov  edx, LINE_CAP
    call read_line                      # -> rax = length, incl. trailing \n

    test rax, rax
    jle  .Lloop                         # error or empty read: try again

    # Trim a trailing '\n', if present.
    mov  rcx, rax                       # rcx = length (line_buf[0..rcx))
    lea  rsi, [rip + line_buf]
    cmp  byte ptr [rsi + rcx - 1], 10
    jne  1f
    dec  rcx
1:
    test rcx, rcx
    jz   .Lloop                         # blank line: reprompt

    # --- "ls" ------------------------------------------------------
    cmp  rcx, 2
    jne  .Ltry_help
    lea  rdi, [rip + cmd_ls]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Ltry_help
    call do_ls
    jmp  .Lloop

.Ltry_help:
    cmp  rcx, 4
    jne  .Ltry_exit
    lea  rdi, [rip + cmd_help]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Ltry_exit
    lea  rsi, [rip + msg_help]
    mov  edx, msg_help_len
    call print
    jmp  .Lloop

.Ltry_exit:
    cmp  rcx, 4
    jne  .Ltry_cat
    lea  rdi, [rip + cmd_exit]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Ltry_cat
    xor  edi, edi
    mov  eax, 60                        # exit(0)
    syscall

.Ltry_cat:
    cmp  rcx, 4
    jle  .Ltry_echo                      # need at least "cat " + 1 char
    lea  rdi, [rip + cmd_cat]
    mov  r8, 4
    call strprefix
    test rax, rax
    jz   .Ltry_echo
    lea  rsi, [rip + line_buf]
    lea  rsi, [rsi + 4]                 # filename starts after "cat "
    mov  rdx, rcx
    sub  rdx, 4                         # filename length
    call do_cat
    jmp  .Lloop

.Ltry_echo:
    cmp  rcx, 5
    jle  .Ltry_clear                     # need at least "echo " + 1 char
    lea  rdi, [rip + cmd_echo]
    mov  r8, 5
    call strprefix
    test rax, rax
    jz   .Ltry_clear
    lea  rsi, [rip + line_buf]
    lea  rsi, [rsi + 5]                 # text starts after "echo "
    mov  rdx, rcx
    sub  rdx, 5
    call print
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print
    jmp  .Lloop

.Ltry_clear:
    cmp  rcx, 5
    jne  .Ltry_uptime
    lea  rdi, [rip + cmd_clear]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Ltry_uptime
    lea  rsi, [rip + clear_seq]
    mov  edx, clear_seq_len
    call print
    jmp  .Lloop

.Ltry_uptime:
    cmp  rcx, 6
    jne  .Ltry_pid
    lea  rdi, [rip + cmd_uptime]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Ltry_pid
    call do_uptime
    jmp  .Lloop

.Ltry_pid:
    cmp  rcx, 3
    jne  .Lunknown
    lea  rdi, [rip + cmd_pid]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Lunknown
    call do_pid
    jmp  .Lloop

.Lunknown:
    lea  rsi, [rip + msg_unknown]
    mov  edx, msg_unknown_len
    call print
    lea  rsi, [rip + line_buf]
    mov  rdx, rcx
    call print
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print
    jmp  .Lloop

# ---------------------------------------------------------------------
# do_ls: list every file in the initramfs.
# ---------------------------------------------------------------------
do_ls:
    push rbx
    lea  rdi, [rip + file_buf]
    mov  esi, FILE_CAP
    mov  eax, 9001                      # eonos_list_files
    syscall
    test rax, rax
    js   1f                             # negative: shouldn't happen here
    test rax, rax
    jz   2f                             # no files
    mov  rbx, rax
    lea  rsi, [rip + file_buf]
    mov  rdx, rbx
    call print
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print
2:  pop  rbx
    ret
1:  pop  rbx
    ret

# ---------------------------------------------------------------------
# do_cat: rsi = filename ptr, rdx = filename length.
# ---------------------------------------------------------------------
do_cat:
    push rbx
    push r12
    push r13
    mov  r12, rsi                       # save name ptr
    mov  r13, rdx                       # save name len
    mov  rdi, r12
    mov  rsi, r13
    lea  rdx, [rip + file_buf]
    mov  r10d, FILE_CAP
    mov  eax, 9002                      # eonos_read_file
    syscall
    cmp  rax, 0
    jge  1f
    cmp  rax, -34                       # -ERANGE
    jne  2f
    lea  rsi, [rip + msg_toobig]
    mov  edx, msg_toobig_len
    call print
    jmp  3f
2:  lea  rsi, [rip + msg_notfound]
    mov  edx, msg_notfound_len
    call print
    jmp  3f
1:  mov  rbx, rax
    lea  rsi, [rip + file_buf]
    mov  rdx, rbx
    call print
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print
3:  pop  r13
    pop  r12
    pop  rbx
    ret

# ---------------------------------------------------------------------
# do_uptime: print milliseconds since boot.
# ---------------------------------------------------------------------
do_uptime:
    push rbx
    mov  eax, 9003                      # eonos_uptime_ms
    syscall
    mov  rbx, rax
    call print_u64_decimal
    lea  rsi, [rip + msg_uptime_suffix]
    mov  edx, msg_uptime_suffix_len
    call print
    pop  rbx
    ret

# ---------------------------------------------------------------------
# do_pid: print this process's pid.
# ---------------------------------------------------------------------
do_pid:
    push rbx
    mov  eax, 39                        # getpid
    syscall
    mov  rbx, rax
    call print_u64_decimal
    lea  rsi, [rip + newline]
    mov  edx, 1
    call print
    pop  rbx
    ret

# ---------------------------------------------------------------------
# print_u64_decimal: print rbx as unsigned decimal, no newline. Builds
# digits least-significant-first into numbuf from the end, then prints
# the used portion in one write -- simplest way to get the digits out
# in the right order without a second reversal pass over the buffer.
# Clobbers rax, rcx, rdx, rsi, rdi, r8, r9.
# ---------------------------------------------------------------------
print_u64_decimal:
    lea  r8, [rip + numbuf]
    add  r8, 24                         # r9 will fill backward from here
    mov  r9, r8
    mov  rax, rbx
    mov  rcx, 10
.Ldigit_loop:
    xor  rdx, rdx
    div  rcx                            # rax /= 10, rdx = rax % 10
    add  dl, '0'
    dec  r9
    mov  [r9], dl
    test rax, rax
    jnz  .Ldigit_loop
    mov  rsi, r9
    mov  rdx, r8
    sub  rdx, r9                        # length = end - start
    call print
    ret

# ---------------------------------------------------------------------
# streq: compare rcx bytes at [rsi] against r8 bytes at [rdi].
# Returns rax = 1 if equal length and content, else 0. Clobbers rcx.
# ---------------------------------------------------------------------
streq:
    cmp  rcx, r8
    jne  .Lstreq_no
    push rsi
    push rdi
    push rcx
    repe cmpsb
    pop  rcx
    pop  rdi
    pop  rsi
    jne  .Lstreq_no
    mov  eax, 1
    ret
.Lstreq_no:
    xor  eax, eax
    ret

# ---------------------------------------------------------------------
# strprefix: does the r8-byte string at [rdi] match the first r8 bytes
# at [rsi]? (rcx = total available bytes at [rsi]; only a length check,
# not part of the comparison itself.) Returns rax = 1/0. Clobbers rcx.
# ---------------------------------------------------------------------
strprefix:
    cmp  rcx, r8
    jl   .Lprefix_no
    push rsi
    push rdi
    push rcx
    mov  rcx, r8
    repe cmpsb
    pop  rcx
    pop  rdi
    pop  rsi
    jne  .Lprefix_no
    mov  eax, 1
    ret
.Lprefix_no:
    xor  eax, eax
    ret

# ---------------------------------------------------------------------
# read_line: rsi = buffer, edx = capacity. Returns rax = bytes read
# (>= 0, may be 0), or a negative errno. rsi/rdx already line up with
# the syscall's own arg2/arg3 -- only rdi (fd) and rax (syscall number)
# need setting.
# ---------------------------------------------------------------------
read_line:
    xor  edi, edi                       # fd = 0 (stdin)
    xor  eax, eax                       # SYS_READ = 0
    syscall
    ret

# ---------------------------------------------------------------------
# print: write RDX bytes at RSI to stdout.
# ---------------------------------------------------------------------
print:
    mov  edi, 1
    mov  eax, 1
    syscall
    ret