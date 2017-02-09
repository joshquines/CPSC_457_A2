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
.include "machine/asmshare.h"

.text

.align   8
.globl   syscall_wrapper
syscall_wrapper:
	swapgs
	movq %rsp, %rax       /* save user stack pointer */
	movq %gs:TSSRSP, %rsp /* switch to kernel stack (from TSS), cf. Processor.h */
	sti                   /* re-enable interrupts (disabled during syscall) */
	pushq %rax            /* push user stack pointer on kernel stack */
	pushq %r11            /* also save %r11 (rflags) */
	pushq %rcx            /* also save %rcx (return address) */
	movq %r10, %rcx       /* restore 4th argument, see syscallStub.S */
	call syscall_handler  /* args in: rdi/rsi/rdx/rcx/r8/r9; retcode: rax */
	popq %rcx             /* restore %rcx (return address) */
	popq %r11             /* restore %r11 (rflags) - sysretq enables interrupts */
	popq %r10             /* get user stack pointer */
	cli                   /* disable interrupts before swapgs and setting stack */
	movq %rsp, %gs:TSSRSP /* store kernel stack pointer in TSS, cf. Processor.h */
	movq %r10, %rsp       /* switch back to user stack */
	swapgs
	sysretq               /* TODO: security - verify that %rcx address is canonical! */

.align   8
.globl   startUserCode
startUserCode:          /* arg1(rdi), arg2(rsi), kstack(rdx), func(rcx), ustack(r8) */
	movq $0x200,%r11      /* enable interrupts during sysretq */
	cli                   /* disable interrupts before swapgs and setting stack */
	movq %rdx, %gs:TSSRSP /* store kernel stack pointer in TSS, cf. Processor.h */
	movq %r8, %rsp        /* set user stack pointer */
	swapgs
	sysretq
