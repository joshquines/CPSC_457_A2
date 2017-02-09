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
#ifndef _stack_h_
#define _stack_h_ 1

#include "runtime/Runtime.h"

// stack.S
class Scheduler;
class Thread;
// initialize stack and switch directly to 'func(arg1,arg2,arg3,arg4)'
extern "C" mword stackDirect(vaddr stack, ptr_t func, ptr_t arg1, ptr_t arg2, ptr_t arg3, ptr_t arg4);
// initialize stack for indirect invocation of 'invokeThread(prevThread,nextMemCtx,func,arg1,arg2,arg3)'
extern "C" mword stackInit(vaddr stack, Runtime::MemoryContext* nextMemCtx, ptr_t func, ptr_t arg1, ptr_t arg2, ptr_t arg3);
// save stack to 'currSP', switch to stack in 'nextSP', then call 'postSwitch(currThread, target)'
extern "C" Thread* stackSwitch(Thread* currThread, Scheduler* target, vaddr* currSP, vaddr nextSP);

#endif /* _stack_h_ */
