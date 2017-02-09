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
#ifndef _BlockingSync_h_
#define _BlockingSync_h_ 1

#include "generic/EmbeddedContainers.h"
#include "runtime/Runtime.h"
#include "runtime/Scheduler.h"
#include "runtime/Thread.h"

#include <list>
#include <map>

class Timeout {
  friend class TimeoutInfo;
  friend class TimeoutBlockingInfo;
  static BasicLock lock;
  static multimap<mword,Thread*> queue;

public:
  static inline void sleep(mword timeout);
  static inline void checkExpiry(mword now);
};

class UnblockInfo {
public:
  virtual void cancelTimeout() {}
  virtual void cancelBlocking(Thread& t) {}
};

class TimeoutInfo : public virtual UnblockInfo {
protected:
  decltype(Timeout::queue)::iterator titer;
public:
  void suspend(mword timeout) {
    Thread* thr = Runtime::getCurrThread();
    Timeout::lock.acquire();
    if (thr->block(this)) {
      titer = Timeout::queue.insert( {timeout, thr} ); // set up timeout
      Runtime::getScheduler()->suspend(Timeout::lock);
    } else {
      Timeout::lock.release();
    }
  }
  virtual void cancelTimeout() {
    AutoLock al(Timeout::lock);
    Timeout::queue.erase(titer);
  }
};

class BlockingInfo : public virtual UnblockInfo {
protected:
  BasicLock& bLock;
  bool timedOut;
public:
  BlockingInfo(BasicLock& bl) : bLock(bl),  timedOut(false) {
    GENASSERT0(bl.check());
  }
  bool suspend(EmbeddedList<Thread>& queue) {
    Thread* thr = Runtime::getCurrThread();
    if (thr->block(this)) {
      queue.push_back(*thr);                           // set up block
      Runtime::getScheduler()->suspend(bLock);
      return !timedOut;
    }
    bLock.release();
    return false;
  }
  virtual void cancelBlocking(Thread& t) {
    timedOut = true;
    AutoLock al(bLock);
    EmbeddedList<Thread>::remove(t);
  }
};

class TimeoutBlockingInfo : public TimeoutInfo, public BlockingInfo {
public:
  TimeoutBlockingInfo(BasicLock& bl) : BlockingInfo(bl) {}
  bool suspend(EmbeddedList<Thread>& queue, mword timeout) {
    Thread* thr = Runtime::getCurrThread();
    Timeout::lock.acquire();
    if (thr->block(this)) {
      queue.push_back(*thr);                           // set up block
      titer = Timeout::queue.insert( {timeout, thr} ); // set up timeout
      Runtime::getScheduler()->suspend(bLock, Timeout::lock);
      return !timedOut;
    }
    Timeout::lock.release();
    bLock.release();
    return false;
  }
};

inline void Timeout::sleep(mword timeout) {
  TimeoutInfo ti;
  ti.suspend(timeout);
}

inline void Timeout::checkExpiry(mword now) {
  list<Thread*> fireList;
  lock.acquire();
  for (auto it = queue.begin(); it != queue.end() && it->first <= now; ) {
    Thread* t = it->second;
    if (t->unblock()) {
      it = queue.erase(it);
      fireList.push_back(t);
    } else {
      it = next(it);
    }
  }
  lock.release();
  for (auto t : fireList) {
    t->getUnblockInfo().cancelBlocking(*t);
    Scheduler::resume(*t);
  }
}

class BlockingQueue {
  EmbeddedList<Thread> queue;

  BlockingQueue(const BlockingQueue&) = delete;                  // no copy
  const BlockingQueue& operator=(const BlockingQueue&) = delete; // no assignment

public:
  BlockingQueue() = default;
  ~BlockingQueue() { GENASSERT0(empty()); }
  bool empty() const { return queue.empty(); }

  // suspend releases bLock; returns 'false' if interrupted
  bool block(BasicLock& bLock, mword timeout = limit<mword>()) {
    if (timeout == limit<mword>()) {
      BlockingInfo bi(bLock);
      return bi.suspend(queue);
    } else if (timeout > 0) {
      TimeoutBlockingInfo tbi(bLock);
      return tbi.suspend(queue, timeout);
    } // else non-blocking
    bLock.release();
    return false;
  }

  bool resume(BasicLock& bLock, Thread*& t) {
    for (t = queue.front(); t != queue.fence(); t = EmbeddedList<Thread>::next(*t)) {
      if (t->unblock()) {
        EmbeddedList<Thread>::remove(*t);
        bLock.release();
        t->getUnblockInfo().cancelTimeout();
        Scheduler::resume(*t);
        return true;
      }
    }
    return false;
  }

  bool resume(BasicLock& bl) { Thread* dummy; return resume(bl, dummy); }
};

class Mutex {
protected:
  BasicLock lock;
  Thread* owner;
  BlockingQueue bq;

  bool internalAcquire(bool ownerLock, mword timeout = limit<mword>()) {
    if slowpath(owner == Runtime::getCurrThread()) {
      GENASSERT1(ownerLock, FmtHex(owner));
    } else {
      lock.acquire();
      if slowpath(owner != nullptr) return bq.block(lock, timeout);
      owner = Runtime::getCurrThread();
      lock.release();
    }
    return true;
  }

  void internalRelease() {
    if slowpath(!bq.resume(lock, owner)) {      // try baton passing
      owner = nullptr;                          // baton not passed
      lock.release();
    }
  }

public:
  Mutex() : owner(nullptr) {}

  bool acquire() {
    return internalAcquire(false);
  }

  bool tryAcquire(mword t = 0) {
    return internalAcquire(false, t);
  }

  void release() {
    GENASSERT1(owner == Runtime::getCurrThread(), FmtHex(owner));
    lock.acquire();
    internalRelease();
  }
};

class OwnerLock : private Mutex {
  mword counter;

public:
  OwnerLock() : counter(0) {}

  mword acquire() {
    if slowpath(internalAcquire(true)) return ++counter; else return 0;
  }

  mword tryAcquire(mword t = 0) {
    if slowpath(internalAcquire(true, t)) return ++counter; else return 0;
  }

  mword release() {
    GENASSERT1(owner == Runtime::getCurrThread(), FmtHex(owner));
    lock.acquire();
    counter -= 1;
    mword retval = counter;
    if (counter == 0) internalRelease();
    else lock.release();
    return retval;
  }
};

class Semaphore {
  BasicLock lock;
  mword counter;
  BlockingQueue bq;

  bool internalP(BasicLock* l, mword timeout = limit<mword>()) {
    lock.acquire(l);
    if fastpath(counter < 1) return bq.block(lock, timeout);
    counter -= 1;
    lock.release();
    return true;
  }

public:
  explicit Semaphore(mword c = 0) : counter(c) {}
  bool empty() { return bq.empty(); }

  bool P(BasicLock* l = nullptr) {
    return internalP(l);
  }

  bool tryP(mword t = 0, BasicLock* l = nullptr) {
    return internalP(l, t);
  }

  void V(BasicLock* l = nullptr) {
    lock.acquire(l);
    if slowpath(!bq.resume(lock)) {            // try baton passing
      counter += 1;                            // baton not passed
      lock.release();
    }
  }
};

class Condition {
  BlockingQueue bq;
public:
  bool empty() { return bq.empty(); }
  bool wait(BasicLock& lock) { return bq.block(lock); }
  void signal(BasicLock& lock) { if slowpath(!bq.resume(lock)) lock.release(); }
};

#endif /* _BlockingSync_h_ */
