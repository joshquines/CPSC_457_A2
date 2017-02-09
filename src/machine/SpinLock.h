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
#ifndef _SpinLock_h_
#define _SpinLock_h_ 1

#include "machine/CPU.h"
#include "machine/Processor.h"

class BinaryLock {
  volatile bool locked;
public:
  BinaryLock() : locked(false) {}
  bool check() const { return locked; }
  bool tryAcquire() {
    KASSERT0(!CPU::interruptsEnabled());
    return !__atomic_test_and_set(&locked, __ATOMIC_SEQ_CST);
  }
  void acquire() {
    KASSERT0(!CPU::interruptsEnabled());
    for (;;) {
      if fastpath(!__atomic_test_and_set(&locked, __ATOMIC_SEQ_CST)) return;
      while (locked) CPU::Pause();
    }
  }
  void release() {
    KASSERT0(!CPU::interruptsEnabled());
    KASSERT0(check());
    locked = false;
  }
} __caligned;

class TicketLock {
  volatile mword serving;
  mword ticket;
public:
  TicketLock() : serving(0), ticket(0) {}
  bool check() const { return sword(ticket-serving) > 0; }
  bool tryAcquire() {
    KASSERT0(!CPU::interruptsEnabled());
    mword tryticket = serving;
    return __atomic_compare_exchange_n(&ticket, &tryticket, tryticket + 1, 0, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED);
  }
  void acquire() {
    KASSERT0(!CPU::interruptsEnabled());
    mword myticket = __atomic_fetch_add(&ticket, 1, __ATOMIC_SEQ_CST);
    while (myticket != serving) CPU::Pause();
  }
  void release() {
    KASSERT0(!CPU::interruptsEnabled());
    KASSERT0(check());
    serving += 1;
  }
};

class SpinLock : protected BinaryLock {
public:
  bool tryAcquire() {
    LocalProcessor::lock();
    if (BinaryLock::tryAcquire()) return true;
    LocalProcessor::unlock();
    return false;
  }
  void acquire(SpinLock* l = nullptr) {
    LocalProcessor::lock();
    BinaryLock::acquire();
    if (l) l->release();
  }
  void release() {
    BinaryLock::release();
    LocalProcessor::unlock();
  }
  bool check() const { return BinaryLock::check(); }
};

class NoLock {
public:
  void acquire() {}
  void release() {}
};

template <typename Lock = SpinLock>
class ScopedLock {
  Lock& lk;
public:
  ScopedLock(Lock& lk) : lk(lk) { lk.acquire(); }
  ~ScopedLock() { lk.release(); }
};

template <>
class ScopedLock<LocalProcessor> {
public:
  ScopedLock() { LocalProcessor::lock(); }
  ~ScopedLock() { LocalProcessor::unlock(); }
};

#endif /* _SpinLock_h_ */
