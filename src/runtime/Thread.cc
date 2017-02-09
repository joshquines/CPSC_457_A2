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
#include "runtime/BlockingSync.h"
#include "runtime/RuntimeImpl.h"
#include "runtime/Scheduler.h"
#include "runtime/Stack.h"
#include "runtime/Thread.h"
#include "kernel/Output.h"

Thread* Thread::create(vaddr mem, size_t ss) {
  vaddr This = mem + ss - sizeof(Thread);
  Runtime::debugT("Thread create: ", FmtHex(mem), '/', FmtHex(ss), '/', FmtHex(This));
  return new (ptr_t(This)) Thread(mem, ss);
}

Thread* Thread::create(size_t ss) {
  vaddr mem = Runtime::allocThreadStack(ss);
  return create(mem, ss);
}

Thread* Thread::create() {
  return create(defaultStack);
}

void Thread::destroy() {
  GENASSERT1(state == Finishing, state);
  Runtime::debugT("Thread destroy: ", FmtHex(stackBottom), '/', FmtHex(stackSize), '/', FmtHex(this));
  Runtime::releaseThreadStack(stackBottom, stackSize);
}

void Thread::start(ptr_t func, ptr_t p1, ptr_t p2, ptr_t p3) {
  stackPointer = stackInit(stackPointer, &Runtime::getMemoryContext(), func, p1, p2, p3);
  Scheduler::resume(*this);
}

void Thread::direct(ptr_t func, ptr_t p1, ptr_t p2, ptr_t p3, ptr_t p4) {
  stackDirect(stackPointer, func, p1, p2, p3, p4);
}

void Thread::cancel() {
  GENASSERT1(this != Runtime::getCurrThread(), Runtime::getCurrThread());
  if (__atomic_exchange_n(&state, Cancelled, __ATOMIC_RELAXED) == Blocked) {
    unblockInfo->cancelTimeout();
    unblockInfo->cancelBlocking(*this);
    Scheduler::resume(*this);
  }
}
