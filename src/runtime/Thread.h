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
#ifndef _Thread_h_
#define _Thread_h_ 1

#include "generic/EmbeddedContainers.h" 
#include "runtime/Runtime.h"

class Scheduler;
class UnblockInfo;

class Thread : public EmbeddedList<Thread>::Link {
  friend class Scheduler;   // Scheduler accesses many internals
  friend void Runtime::postResume(bool, Thread&, AddressSpace&);

  vaddr stackPointer;       // holds stack pointer while thread inactive
  vaddr stackBottom;        // bottom of allocated memory for thread/stack
  size_t stackSize;         // size of allocated memory

  mword priority;           // scheduling priority
  bool affinity;            // stick with scheduler
  Scheduler* nextScheduler; // resume on same core (for now)

  Runtime::MachContext ctx;
  Runtime::ThreadStats stats;

  Thread(const Thread&) = delete;
  const Thread& operator=(const Thread&) = delete;

protected:
  enum State { Running, Blocked, Cancelled, Finishing } state;
  UnblockInfo* unblockInfo; // unblock vs. timeout vs. cancel

  Thread(vaddr sb, size_t ss) :
    stackPointer(vaddr(this)), stackBottom(sb), stackSize(ss),
    priority(defPriority), affinity(false), nextScheduler(nullptr),
    state(Running), unblockInfo(nullptr) {}

  // called directly when creating idle thread(s)
  static Thread* create(vaddr mem, size_t ss);

public:
  static Thread* create(size_t ss);
  static Thread* create();
  void destroy();
  void start(ptr_t func, ptr_t p1 = nullptr, ptr_t p2 = nullptr, ptr_t p3 = nullptr);
  void direct(ptr_t func, ptr_t p1 = nullptr, ptr_t p2 = nullptr, ptr_t p3 = nullptr, ptr_t p4 = nullptr);
  void cancel();

  bool block(UnblockInfo* ubi) {
    GENASSERT1(this == Runtime::getCurrThread(), Runtime::getCurrThread());
    GENASSERT1(state != Blocked, state);
    unblockInfo = ubi;
    State expected = Running;
    return __atomic_compare_exchange_n( &state, &expected, Blocked, 0, __ATOMIC_RELAXED, __ATOMIC_RELAXED );
  }

  bool unblock() {
    GENASSERT1(this != Runtime::getCurrThread(), Runtime::getCurrThread());
    State expected = Blocked;
    return __atomic_compare_exchange_n( &state, &expected, Running, 0, __ATOMIC_RELAXED, __ATOMIC_RELAXED );
  }

  UnblockInfo& getUnblockInfo() {
    GENASSERT0(unblockInfo);
    return *unblockInfo;
  }

  Thread* setPriority(mword p)      { priority = p; return this; }
  Thread* setAffinity(Scheduler* s) { affinity = (nextScheduler = s); return this; }
  Scheduler* getAffinity() const    { return affinity ? nextScheduler : nullptr; }

  const Runtime::ThreadStats& getStats() const { return stats; }
};

#endif /* _Thread_h_ */
