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
#     2  open(path, path_len, flags)
#     3  close(fd)
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
#  9006  eonos_list_processes(buf, len)
#
# Commands: ls, cat <file>, exec <file>, write <file> <text>,
# echo <text>, ps, clear,
# uptime, pid, help, exit.
# No pipes, no scripting -- just enough to poke around from a prompt.

.intel_syntax noprefix

.section .bss
.balign 8
line_buf:   .skip 128
LINE_CAP = 128
file_buf:   .skip 4096
FILE_CAP = 4096
numbuf:     .skip 24                # enough for any 64-bit value in decimal
wait_status: .skip 4
edit_path:   .skip 256
edit_path_len: .skip 8
edit_size:   .skip 8

.section .rodata
msg_banner:   .ascii "EonOS shell -- type 'help' for commands\n"
.equ msg_banner_len, . - msg_banner
prompt:       .ascii "eonsh> "
.equ prompt_len, . - prompt
msg_help:     .ascii "commands: ls, cat, exec, write, mkdir, rm, touch, cp, edit, echo, ps, clear, uptime, pid, help, exit\n"
.equ msg_help_len, . - msg_help
msg_unknown:  .ascii "unknown command: "
.equ msg_unknown_len, . - msg_unknown
msg_notfound: .ascii "cat: no such file\n"
.equ msg_notfound_len, . - msg_notfound
msg_exec_error: .ascii "exec: unable to launch file\n"
.equ msg_exec_error_len, . - msg_exec_error
msg_ps_error: .ascii "ps: unable to list processes\n"
.equ msg_ps_error_len, . - msg_ps_error
msg_write_ok: .ascii "write: saved\n"
.equ msg_write_ok_len, . - msg_write_ok
msg_write_error: .ascii "write: failed (use a valid FAT32 path under /disk)\n"
.equ msg_write_error_len, . - msg_write_error
msg_toobig:   .ascii "cat: file too large to display\n"
.equ msg_toobig_len, . - msg_toobig
msg_mkdir_error: .ascii "mkdir: failed\n"
.equ msg_mkdir_error_len, . - msg_mkdir_error
msg_rm_error: .ascii "rm: failed\n"
.equ msg_rm_error_len, . - msg_rm_error
msg_touch_error: .ascii "touch: failed\n"
.equ msg_touch_error_len, . - msg_touch_error
msg_edit_header: .ascii " EonOS nano -- append lines, .save writes, .quit exits\n"
.equ msg_edit_header_len, . - msg_edit_header
msg_edit_prompt: .ascii "nano> "
.equ msg_edit_prompt_len, . - msg_edit_prompt
msg_edit_saved: .ascii "[ saved ]\n"
.equ msg_edit_saved_len, . - msg_edit_saved
msg_edit_error: .ascii "edit: failed (file or 4 KiB buffer limit)\n"
.equ msg_edit_error_len, . - msg_edit_error
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
cmd_ps:       .ascii "ps"
cmd_cat:      .ascii "cat "   # note trailing space: "cat" alone is not this
cmd_exec:     .ascii "exec "
cmd_write:    .ascii "write "
cmd_echo:     .ascii "echo "  # same convention; bare "echo" prints nothing
cmd_mkdir:    .ascii "mkdir "
cmd_rm:       .ascii "rm "
cmd_touch:    .ascii "touch "
cmd_cp:       .ascii "cp "
cmd_edit:     .ascii "edit "
cmd_save:     .ascii ".save"
cmd_quit:     .ascii ".quit"

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
    jle  .Ltry_exec                      # need at least "cat " + 1 char
    lea  rdi, [rip + cmd_cat]
    mov  r8, 4
    call strprefix
    test rax, rax
    jz   .Ltry_exec
    lea  rsi, [rip + line_buf]
    lea  rsi, [rsi + 4]                 # filename starts after "cat "
    mov  rdx, rcx
    sub  rdx, 4                         # filename length
    call do_cat
    jmp  .Lloop

.Ltry_exec:
    cmp  rcx, 6
    jle  .Ltry_write
    lea  rdi, [rip + cmd_exec]
    mov  r8, 5
    call strprefix
    test rax, rax
    jz   .Ltry_write
    lea  rdi, [rip + line_buf + 5]      # path pointer (arg 1)
    mov  rsi, rcx                       # path length (arg 2)
    sub  rsi, 5
    mov  eax, 9005
    syscall
    test rax, rax
    js   .Lexec_error
    mov  rdi, rax
    lea  rdx, [rip + wait_status]
    mov  eax, 61
    syscall
    jmp  .Lloop
.Lexec_error:
    lea  rsi, [rip + msg_exec_error]
    mov  edx, msg_exec_error_len
    call print
    jmp  .Lloop

.Ltry_write:
    cmp  rcx, 8
    jle  .Ltry_mkdir
    lea  rdi, [rip + cmd_write]
    mov  r8, 6
    call strprefix
    test rax, rax
    jz   .Ltry_mkdir
    lea  r12, [rip + line_buf + 6]
    mov  r13, rcx
    sub  r13, 6
    xor  r14d, r14d
.Lwrite_find_space:
    cmp  r14, r13
    je   .Lwrite_error
    cmp  byte ptr [r12 + r14], ' '
    je   .Lwrite_path_found
    inc  r14
    jmp  .Lwrite_find_space
.Lwrite_path_found:
    lea  r15, [r12 + r14 + 1]
    sub  r13, r14
    dec  r13                            # data length after the separator
    mov  rdi, r12                       # path
    mov  rsi, r14                       # path length
    mov  edx, 577                       # O_WRONLY | O_CREAT | O_TRUNC
    mov  eax, 2                         # open
    syscall
    test rax, rax
    js   .Lwrite_error
    mov  rbx, rax                       # fd
    mov  rdi, rbx
    mov  rsi, r15                       # text
    mov  rdx, r13                       # text length
    mov  eax, 1                         # write
    syscall
    test rax, rax
    js   .Lwrite_close_error
    mov  rdi, rbx
    mov  eax, 3                         # close
    syscall
    lea  rsi, [rip + msg_write_ok]
    mov  edx, msg_write_ok_len
    call print
    jmp  .Lloop
.Lwrite_close_error:
    mov  rdi, rbx
    mov  eax, 3
    syscall
.Lwrite_error:
    lea  rsi, [rip + msg_write_error]
    mov  edx, msg_write_error_len
    call print
    jmp  .Lloop

.Ltry_mkdir:
    cmp  rcx, 7
    jle  .Ltry_rm
    lea  rdi, [rip + cmd_mkdir]
    mov  r8, 6
    call strprefix
    test rax, rax
    jz   .Ltry_rm
    lea  rdi, [rip + line_buf + 6]
    mov  rsi, rcx
    sub  rsi, 6
    mov  eax, 9007
    syscall
    test rax, rax
    js   .Lmkdir_error
    jmp  .Lloop
.Lmkdir_error:
    lea  rsi, [rip + msg_mkdir_error]
    mov  edx, msg_mkdir_error_len
    call print
    jmp  .Lloop

.Ltry_rm:
    cmp  rcx, 4
    jle  .Ltry_touch
    lea  rdi, [rip + cmd_rm]
    mov  r8, 3
    call strprefix
    test rax, rax
    jz   .Ltry_touch
    lea  rdi, [rip + line_buf + 3]
    mov  rsi, rcx
    sub  rsi, 3
    mov  eax, 9008
    syscall
    test rax, rax
    js   .Lrm_error
    jmp  .Lloop
.Lrm_error:
    lea  rsi, [rip + msg_rm_error]
    mov  edx, msg_rm_error_len
    call print
    jmp  .Lloop

.Ltry_touch:
    cmp  rcx, 7
    jle  .Ltry_cp
    lea  rdi, [rip + cmd_touch]
    mov  r8, 6
    call strprefix
    test rax, rax
    jz   .Ltry_cp
    lea  rdi, [rip + line_buf + 6]
    mov  rsi, rcx
    sub  rsi, 6
    mov  edx, 65                       # O_WRONLY | O_CREAT
    mov  eax, 2
    syscall
    test rax, rax
    js   .Ltouch_error
    mov  rdi, rax
    mov  eax, 3
    syscall
    jmp  .Lloop
.Ltouch_error:
    lea  rsi, [rip + msg_touch_error]
    mov  edx, msg_touch_error_len
    call print
    jmp  .Lloop

.Ltry_cp:
    cmp  rcx, 6
    jle  .Ltry_edit
    lea  rdi, [rip + cmd_cp]
    mov  r8, 3
    call strprefix
    test rax, rax
    jz   .Ltry_edit
    lea  r12, [rip + line_buf + 3]
    mov  r13, rcx
    sub  r13, 3
    xor  r14d, r14d
.Lcp_find_space:
    cmp  r14, r13
    je   .Ltry_edit
    cmp  byte ptr [r12 + r14], ' '
    je   .Lcp_path_found
    inc  r14
    jmp  .Lcp_find_space
.Lcp_path_found:
    lea  r15, [r12 + r14 + 1]
    mov  rbx, r13
    sub  rbx, r14
    dec  rbx
    mov  rdi, r12
    mov  rsi, r14
    xor  edx, edx                     # O_RDONLY
    mov  eax, 2
    syscall
    test rax, rax
    js   .Lcopy_error
    mov  r12, rax                     # source fd
    mov  rdi, r15
    mov  rsi, rbx
    mov  edx, 577                     # O_WRONLY|O_CREAT|O_TRUNC
    mov  eax, 2
    syscall
    test rax, rax
    js   .Lcopy_close_source
    mov  r13, rax                     # destination fd
.Lcopy_loop:
    mov  rdi, r12
    lea  rsi, [rip + file_buf]
    mov  edx, FILE_CAP
    xor  eax, eax
    syscall
    test rax, rax
    js   .Lcopy_close_both
    jz   .Lcopy_done
    mov  rdx, rax
    mov  rdi, r13
    lea  rsi, [rip + file_buf]
    mov  eax, 1
    syscall
    cmp  rax, rdx
    jne  .Lcopy_close_both
    jmp  .Lcopy_loop
.Lcopy_done:
    mov  rdi, r12
    mov  eax, 3
    syscall
    mov  rdi, r13
    mov  eax, 3
    syscall
    jmp  .Lloop
.Lcopy_close_both:
    mov  rdi, r13
    mov  eax, 3
    syscall
.Lcopy_close_source:
    mov  rdi, r12
    mov  eax, 3
    syscall
.Lcopy_error:
    lea  rsi, [rip + msg_write_error]
    mov  edx, msg_write_error_len
    call print
    jmp  .Lloop

.Ltry_edit:
    cmp  rcx, 6
    jle  .Ltry_echo
    lea  rdi, [rip + cmd_edit]
    mov  r8, 5
    call strprefix
    test rax, rax
    jz   .Ltry_echo
    lea  rsi, [rip + line_buf + 5]
    mov  rdx, rcx
    sub  rdx, 5
    call do_edit
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
    jne  .Ltry_ps
    lea  rdi, [rip + cmd_pid]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Ltry_ps
    call do_pid
    jmp  .Lloop

.Ltry_ps:
    cmp  rcx, 2
    jne  .Lunknown
    lea  rdi, [rip + cmd_ps]
    mov  r8, rcx
    call streq
    test rax, rax
    jz   .Lunknown
    call do_ps
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
# do_ps: display the kernel's tracked user process records.
# ---------------------------------------------------------------------
do_ps:
    push rbx
    lea  rdi, [rip + file_buf]
    mov  esi, FILE_CAP
    mov  eax, 9006                      # eonos_list_processes
    syscall
    test rax, rax
    js   1f
    test rax, rax
    jz   2f
    mov  rbx, rax
    lea  rsi, [rip + file_buf]
    mov  rdx, rbx
    call print
    jmp  2f
1:  lea  rsi, [rip + msg_ps_error]
    mov  edx, msg_ps_error_len
    call print
2:  pop  rbx
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
# do_edit: a tiny append-only, line-oriented editor. The canonical input
# path provides line editing; .save writes the buffer and .quit returns.
# ---------------------------------------------------------------------
do_edit:
    push rbx
    push r12
    push r13
    push r14
    push r15
    mov  r12, rsi                      # path pointer
    mov  r13, rdx                      # path length
    lea  rdi, [rip + edit_path]
    mov  rsi, r12
    mov  rcx, r13
    rep  movsb
    mov  qword ptr [rip + edit_path_len], r13

    mov  rdi, r12
    mov  rsi, r13
    lea  rdx, [rip + file_buf]
    mov  r10d, FILE_CAP
    mov  eax, 9002                     # load existing file if present
    syscall
    test rax, rax
    js   .Ledit_new_file
    mov  r14, rax                      # current buffer size
    jmp  .Ledit_screen
.Ledit_new_file:
    xor  r14d, r14d
.Ledit_screen:
    lea  rsi, [rip + clear_seq]
    mov  edx, clear_seq_len
    call print
    lea  rsi, [rip + msg_edit_header]
    mov  edx, msg_edit_header_len
    call print
    lea  rsi, [rip + file_buf]
    mov  rdx, r14
    call print
.Ledit_loop:
    lea  rsi, [rip + msg_edit_prompt]
    mov  edx, msg_edit_prompt_len
    call print
    lea  rsi, [rip + line_buf]
    mov  edx, LINE_CAP
    call read_line
    test rax, rax
    jle  .Ledit_loop
    mov  rbx, rax                      # complete entered line length
    mov  r15, rax                      # command comparison length
    lea  rsi, [rip + line_buf]
    cmp  byte ptr [rsi + r15 - 1], 10
    jne  .Ledit_compare
    dec  r15
.Ledit_compare:
    cmp  r15, 5
    jne  .Ledit_check_quit
    lea  rdi, [rip + cmd_save]
    mov  rcx, r15
    mov  r8, 5
    call streq
    test rax, rax
    jnz  .Ledit_save
.Ledit_check_quit:
    cmp  r15, 5
    jne  .Ledit_append
    lea  rdi, [rip + cmd_quit]
    lea  rsi, [rip + line_buf]
    mov  rcx, r15
    mov  r8, 5
    call streq
    test rax, rax
    jnz  .Ledit_return
.Ledit_append:
    mov  rax, rbx                       # append line including its newline
    cmp  r14, FILE_CAP
    jae  .Ledit_error
    lea  rdi, [rip + file_buf]
    add  rdi, r14
    lea  rsi, [rip + line_buf]
    mov  rcx, rax
    mov  rdx, r14
    add  rdx, rcx
    cmp  rdx, FILE_CAP
    ja   .Ledit_error
    rep  movsb
    add  r14, rax
    jmp  .Ledit_loop
.Ledit_save:
    lea  rdi, [rip + edit_path]
    mov  rsi, qword ptr [rip + edit_path_len]
    mov  edx, 577                      # O_WRONLY | O_CREAT | O_TRUNC
    mov  eax, 2
    syscall
    test rax, rax
    js   .Ledit_error
    mov  r12, rax
    mov  rdi, r12
    lea  rsi, [rip + file_buf]
    mov  rdx, r14
    mov  eax, 1
    syscall
    cmp  rax, r14
    jne  .Ledit_write_failed
    mov  rdi, r12
    mov  eax, 3
    syscall
    lea  rsi, [rip + msg_edit_saved]
    mov  edx, msg_edit_saved_len
    call print
    jmp  .Ledit_loop
.Ledit_write_failed:
    mov  rdi, r12
    mov  eax, 3
    syscall
    jmp  .Ledit_error
.Ledit_error:
    lea  rsi, [rip + msg_edit_error]
    mov  edx, msg_edit_error_len
    call print
    jmp  .Ledit_return
.Ledit_return:
    pop  r15
    pop  r14
    pop  r13
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
