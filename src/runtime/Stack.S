/******************************************************************************
    Copyright © 2012-2015 Martin Karsten

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
#if defined(__x86_64__)
.include "generic/regsave.h"

.text

.align 8
.globl stackDirect
stackDirect:                # stack pointer, func, arg1, arg2, arg3, arg4
	movq $0,   %rbp           # set %rbp to 0 
	movq %rdi, %rsp           # set stack pointer
	movq %rsi, %rax           # save 'func' in %rax
	movq %rdx, %rdi           # shift arg1 in place
	movq %rcx, %rsi           # shift arg2 in place
	movq %r8,  %rdx           # shift arg3 in place
	movq %r9,  %rcx           # shift arg4 in place
	pushq %rbp                # previous %rip = 0 (fake stack frame)
	pushq %rbp                # previous %rbp = 0
	movq %rsp, %rbp           # set base pointer
	jmp *%rax                 # invoke 'func'

.align 8
.globl stackInit
stackInit:                  # stack, nextMemCtx, func, arg1, arg2, arg3 -> new stack
	movq $0,   -64(%rdi)      # indirectly set %rbp to 0
	movq %rsi, -56(%rdi)      # store 'nextMemCtx' for stub function (via %rbx)
	movq %rdx, -48(%rdi)      # store 'func' for stub function (via %r12)
	movq %rcx, -40(%rdi)      # store 'arg1' for stub function (via %r13)
	movq %r8,  -32(%rdi)      # store 'arg2' for stub function (via %r14)
	movq %r9,  -24(%rdi)      # store 'arg3' for stub function (via %r15)
	movq $stubInit, -16(%rdi) # push stub function as return address
;	leaq stubInit(%rip), %rax # set stub function using RIP-relative addressing
;	movq %rax, -16(%rdi)      # push stub function as return address
	leaq -64(%rdi), %rax      # return stack address, size 64: cf. STACK_PUSH
	ret

.align 8
stubInit:                   # stub needed to pass arguments to 'invokeThread'
	movq %rax, %rdi           # 'prevThread' as returned by 'postSwitch'
	movq %rbx, %rsi           # 'nextMemCtx'
	movq %r12, %rdx           # 'func'
	movq %r13, %rcx           # 'arg1'
	movq %r14, %r8            # 'arg2'
	movq %r15, %r9            # 'arg3'
	pushq %rbp                # previous %rip = 0 (fake stack frame)
	pushq %rbp                # previous %rbp = 0
	movq %rsp, %rbp           # set base pointer
	jmp invokeThread

.align 8
.globl stackSwitch
stackSwitch:                # (currT, target, &currSP, nextSP)
	STACK_PUSH                # save register context
	mov %rsp, (%rdx)          # save current stack pointer
	mov %rcx, %rsp            # load next stack pointer
	STACK_POP                 # restore register context
	jmp postSwitch            # %rdi/%rsi set -> return directly from postSwitch

#else
#error unsupported architecture: only __x86_64__ supported at this time
#endif
