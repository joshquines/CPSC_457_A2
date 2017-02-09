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
#include "generic/Buffers.h"
#include "runtime/SyncQueues.h"
#include "runtime/Thread.h"
#include "kernel/Clock.h"
#include "kernel/Output.h"
#include <atomic>

static Mutex mtx;
static Semaphore sem(1);
static Semaphore tsem;
static mword acquireCount;
static mword releaseCount;

static const int testcount  = 5000;
static const int printcount = 500;

// Mutex Test
static void mutexTestMain(ptr_t x) {
  for (int i = 0; i < testcount; i++) {
    mword t = Clock::now();
    if (t % 11 == 0 && mtx.tryAcquire(t + t % 11)) {
      DBG::outl(DBG::Tests, "mutex ", (char*)x, " nb");
    } else {
      mtx.acquire();
    }
    __atomic_add_fetch( &acquireCount, 1, __ATOMIC_RELAXED);
    if (i % printcount == 0) {
      DBG::outl(DBG::Tests, "mutex ", (char*)x, ' ', i);
    }
    mtx.release();
    __atomic_add_fetch( &releaseCount, 1, __ATOMIC_RELAXED);
  }
  DBG::outl(DBG::Tests, "mutex ", (char*)x, " done");
  tsem.V();
}

void MutexTest() {
  KOUT::outl("running MutexTest...");
  acquireCount = releaseCount = 0;
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m0");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m1");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m2");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m3");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m4");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m5");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m6");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m7");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m8");
  Thread::create()->start((ptr_t)mutexTestMain, (ptr_t)"m9");
  DBG::outl(DBG::Tests, "MutexTest: all threads running...");
  for (mword i = 0; i < 10; i += 1) tsem.P();
  KASSERT1(acquireCount == releaseCount, "acquire/release counts differ");
  KASSERT1(acquireCount == testcount * 10, "wrong number of acquire/release");
}

// Semaphore Test
static void semaphoreTestMain(ptr_t x) {
  for (int i = 0; i < testcount; i++) {
    mword t = Clock::now();
    if (t % 11 == 0 && sem.tryP(t + t % 11)) {
      DBG::outl(DBG::Tests, "semaphore ", (char*)x, " nb");
    } else {
      sem.P();
    }
    __atomic_add_fetch( &acquireCount, 1, __ATOMIC_RELAXED);
    if (i % printcount == 0) {
      DBG::outl(DBG::Tests, "semaphore ", (char*)x, ' ', i);
    }
    sem.V();
    __atomic_add_fetch( &releaseCount, 1, __ATOMIC_RELAXED);
  }
  DBG::outl(DBG::Tests, "semaphore ", (char*)x, " done");
  tsem.V();
}

void SemaphoreTest() {
  KOUT::outl("running SemaphoreTest...");
  acquireCount = releaseCount = 0;
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s0");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s1");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s2");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s3");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s4");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s5");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s6");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s7");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s8");
  Thread::create()->start((ptr_t)semaphoreTestMain, (ptr_t)"s9");
  DBG::outl(DBG::Tests, "SemaphoreTest: all threads running...");
  for (mword i = 0; i < 10; i += 1) tsem.P();
  KASSERT1(acquireCount == releaseCount, "acquire/release count differ");
  KASSERT1(acquireCount == testcount * 10, "wrong number of acquire/release");
}

// SyncQueue Test
static const mword SENTINEL = ~0;
static MessageQueue<FixedRingBuffer<mword, 256>> syncQueue;

static void consumer(ptr_t) {
  for (;;) {
    mword val = syncQueue.recv();
    if (val == SENTINEL) break;
    if (val % printcount == 0) DBG::outl(DBG::Basic, "removed:", val);
  }
  tsem.V();
}

static void producer(ptr_t) {
  for (mword i = 0; i < testcount; i += 1) {
    while (!syncQueue.trySend(i)) CPU::Pause();
  }
  syncQueue.send(SENTINEL);
}

void SyncQueueTest() {
  KOUT::outl("running SyncQueueTest...");
  Thread::create()->start((ptr_t)consumer);
  Thread::create()->start((ptr_t)producer);
  tsem.P();
}

int LockTest() {
  MutexTest();
  SemaphoreTest();
  SyncQueueTest();
  KOUT::outl("LockTest done");
  return 0;
}
