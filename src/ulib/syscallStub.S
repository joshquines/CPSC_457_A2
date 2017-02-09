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

.align   8
.globl syscallStub
syscallStub:      /* store/restore caller-owned registers */
  STACK_PUSH
  movq %rcx, %r10 /* save 4th argument, %rcx overwritten by syscall */
  syscall
  STACK_POP
  retq
