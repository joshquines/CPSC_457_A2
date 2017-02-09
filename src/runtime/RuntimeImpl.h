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
#ifndef _RuntimeImpl_h_
#define _RuntimeImpl_h_ 1

#if defined(__KOS__)

#include "runtime/Runtime.h"
#include "runtime/Scheduler.h"
#include "kernel/Process.h"

namespace Runtime {
  static void idleLoop(Scheduler* s) {
    for (;;) {
      // disable PreemptIPI - might need IPI renumbering
      mword halt = s->preemption + 3;
      while (!s->readyCount && sword(halt - s->preemption) > 0) CPU::Pause();
      halt = s->resumption;
      LocalProcessor::lock(true);
      s->preempt();
      LocalProcessor::unlock(true);
      while (halt == s->resumption) CPU::Halt();
    }
    unreachable();
  }

  static void postResume(bool invoke, Thread& prevThread, AddressSpace& nextAS) {
    CHECK_LOCK_COUNT(1);
    AddressSpace& prevAS = CurrAS();
    if fastpath(prevThread.state == Thread::Finishing) {
      if (prevAS.user() && reinterpret_cast<Process*>(&prevAS)->destroyThread(prevThread)) {
        prevAS.clearUserPaging();
        nextAS.enter();
        delete reinterpret_cast<Process*>(&prevAS);
      } else {
        nextAS.enter();
      }
      prevThread.destroy();
    } else {
      if (prevAS.user()) prevThread.ctx.save();
      nextAS.enter();
    }
    if (nextAS.user()) Runtime::getCurrThread()->ctx.restore();
    if (invoke) LocalProcessor::unlock(true);
  }
}

#else
#error undefined platform: only __KOS__ supported at this time
#endif

#endif /* _RuntimeImpl_h_ */
