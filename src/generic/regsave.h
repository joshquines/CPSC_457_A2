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
#ifndef _regsave_h_
#define _regsave_h_ 1

#if defined(__x86_64__)

# GCC: arguments in %rdi, %rsi, %rdx, %rcx , %r8, %r9
# see http://en.wikipedia.org/wiki/X86_calling_conventions

# caller- vs. callee-owned registers
# see http://x86-64.org/documentation/abi.pdf, Sec 3.2.1

# save callee-owned registers during asynchronous interrupt
# caller-owned regs automatically saved by compiler code upon routine calls

.set ISRFRAME, 72

.macro ISR_PUSH                       /* ISRFRAME bytes pushed */
	pushq %rax
	pushq %rcx
	pushq %rdx
	pushq %rdi
	pushq %rsi
	pushq %r8
	pushq %r9
	pushq %r10
	pushq %r11
.endm

.macro ISR_POP                        /* ISRFRAME bytes popped */
	popq %r11
	popq %r10
	popq %r9
	popq %r8
	popq %rsi
	popq %rdi
	popq %rdx
	popq %rcx
	popq %rax
.endm

# save caller-owned registers during synchronous stack switch
# callee-owned regs automatically savedby compiler code before routine calls

.macro STACK_PUSH
  pushq %r15
  pushq %r14
  pushq %r13
  pushq %r12
  pushq %rbx
  pushq %rbp
.endm

.macro STACK_POP
  popq %rbp
  popq %rbx
  popq %r12
  popq %r13
  popq %r14
  popq %r15
.endm

#else
#error unsupported architecture: only __x86_64__ supported at this time
#endif

#endif /* _regsave_h_ */
