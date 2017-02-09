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
.include "generic/regsave.h"

.text

.align 8
.globl   _KOS_sigwrapper
_KOS_sigwrapper:
	pushfq
	ISR_PUSH
	movq (ISRFRAME+8)(%rsp), %rdi /* signal number before register frame */
	call _KOS_sighandler
	ISR_POP
	popfq
	popq dummy                    /* skip signal number without modifying rflags */
	retq

.bss

dummy:
	.quad 0x0000000000000000
