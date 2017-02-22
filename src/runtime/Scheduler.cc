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
#include "runtime/RuntimeImpl.h"
#include "runtime/Scheduler.h"
#include "runtime/Stack.h"
#include "runtime/Thread.h"
#include "kernel/Output.h"


// Assignment 2 Variables
unsigned int Scheduler::schedMinGranularity;
unsigned int Scheduler::defaultEpochLength;

Tree<ThreadNode> *readyTree;

//-----


Scheduler::Scheduler() : readyCount(0), preemption(0), resumption(0), partner(this) {
  Thread* idleThread = Thread::create((vaddr)idleStack, minimumStack);
  idleThread->setAffinity(this)->setPriority(idlePriority);
  // use low-level routines, since runtime context might not exist
  idleThread->stackPointer = stackInit(idleThread->stackPointer, &Runtime::getDefaultMemoryContext(), (ptr_t)Runtime::idleLoop, this, nullptr, nullptr);
  readyQueue[idlePriority].push_back(*idleThread);

  
  // ASSIGNMENT 2 STUFF
  epochLengthTicks = defaultEpochLength;
  readyTree = new Tree<ThreadNode>();
  readyTree->insert(*(new ThreadNode(idleThread)));
  readyCount += 1;
  minvRuntime = 0;
}

// Assignment 2 get/set for Variables
void Scheduler::setMinGran(unsigned int parsedNum)   {schedMinGranularity = parsedNum;}
void Scheduler::setDefaultEpoch(unsigned int parsedNum)    {defaultEpochLength = parsedNum;}

unsigned int Scheduler::getMinGran()   {return schedMinGranularity;}
unsigned int Scheduler::getDefaultEpoch()    {return defaultEpochLength;}
// ----

static inline void unlock() {}

template<typename... Args>
static inline void unlock(BasicLock &l, Args&... a) {
  l.release();
  unlock(a...);
}

// very simple N-class prio scheduling!
template<typename... Args>
inline void Scheduler::switchThread(Scheduler* target, Args&... a) {
  preemption += 1;
  CHECK_LOCK_MIN(sizeof...(Args));
  Thread* nextThread;
  readyLock.acquire();

 /* 
  for (mword i = 0; i < (target ? idlePriority : maxPriority); i += 1) {
    if (!readyQueue[i].empty()) {
      nextThread = readyQueue[i].pop_front();
      readyCount -= 1;
      goto threadFound;
    }
  }**Convert for readyTree
  */
  // FOR ASSIGNMENT 2

  // If readyTree is not empty
  if(!readyTree->empty()){
    nextThread = readyTree->popMinNode()->th;
    
    // suspend thread if need be
    // Whenever a task is suspended for I/O, after removing it from readyTree, the vRuntime is updated
    if (sizeof...(Args)!= 0){
      nextThread->vRuntime -= minvRuntime;
    }

    readyCount -= 1;
      // switch current thread with left most node
      while(!readyTree->empty() && nextThread == Runtime::getCurrThread()){
        nextThread = readyTree->popMinNode()->th;
        readyCount -= 1;
      }

      // if tree is empty use current thread
      if(readyTree->empty()){
          if(nextThread == Runtime::getCurrThread()){
            readyLock.release();
            unlock(a...);
            return;
        }
      }

    goto threadFound;
  }
  //-----


  readyLock.release();
  GENASSERT0(target);
  GENASSERT0(!sizeof...(Args));
  return;                                         // return to current thread

threadFound:
  readyLock.release();
  resumption += 1;
  Thread* currThread = Runtime::getCurrThread();
  GENASSERTN(currThread && nextThread && nextThread != currThread, currThread, ' ', nextThread);

  if (target) currThread->nextScheduler = target; // yield/preempt to given processor
  else currThread->nextScheduler = this;          // suspend/resume to same processor
  unlock(a...);                                   // ...thus can unlock now
  CHECK_LOCK_COUNT(1);
  Runtime::debugS("Thread switch <", (target ? 'Y' : 'S'), ">: ", FmtHex(currThread), '(', FmtHex(currThread->stackPointer), ") to ", FmtHex(nextThread), '(', FmtHex(nextThread->stackPointer), ')');

  Runtime::MemoryContext& ctx = Runtime::getMemoryContext();
  Runtime::setCurrThread(nextThread);
  Thread* prevThread = stackSwitch(currThread, target, &currThread->stackPointer, nextThread->stackPointer);
  // REMEMBER: Thread might have migrated from other processor, so 'this'
  //           might not be currThread's Scheduler object anymore.
  //           However, 'this' points to prevThread's Scheduler object.
  Runtime::postResume(false, *prevThread, ctx);
  if (currThread->state == Thread::Cancelled) {
    currThread->state = Thread::Finishing;
    switchThread(nullptr);
    unreachable();
  }
}

extern "C" Thread* postSwitch(Thread* prevThread, Scheduler* target) {
  CHECK_LOCK_COUNT(1);
  if fastpath(target) Scheduler::resume(*prevThread);
  return prevThread;
}

extern "C" void invokeThread(Thread* prevThread, Runtime::MemoryContext* ctx, funcvoid3_t func, ptr_t arg1, ptr_t arg2, ptr_t arg3) {
  Runtime::postResume(true, *prevThread, *ctx);
  func(arg1, arg2, arg3);
  Runtime::getScheduler()->terminate();
}

void Scheduler::enqueue(Thread& t) {
  GENASSERT1(t.priority < maxPriority, t.priority);
  readyLock.acquire();
  //readyQueue[t.priority].push_back(t);
  // FOR ASSIGNMENT 2
  if(readyTree->empty()){
    minvRuntime = 0;            // if tree is empty minvRuntime is 0
  }else{
    minvRuntime = readyTree->readMinNode()->th->vRuntime;     // if not, left-most node's vRuntime is minvRuntime
  }
  if(t.checkThread){          // if thread has been scheduled before
    t.checkThread = false;
    t.vRuntime = minvRuntime;
  }

  //-----

  readyTree->insert(*(new ThreadNode(&t)));
  bool wake = (readyCount == 0);
  readyCount += 1;
  adjustEpochTicks();
  readyLock.release();
  Runtime::debugS("Thread ", FmtHex(&t), " queued on ", FmtHex(this));
  if (wake) Runtime::wakeUp(this);
}


void Scheduler::resume(Thread& t) {
  GENASSERT1(&t != Runtime::getCurrThread(), Runtime::getCurrThread());
  if (t.nextScheduler){
    // ASSIGNMENT 2 
    //** Minor bug: had to compile this a few times for it to run without infinite pinging
    //** Infinite pinging happens approx 1/3 times 
    // Whenever a task is unblocked after I/O, before inserting it into readyTree, the vRuntime is updated
    t.vRuntime += t.nextScheduler->minvRuntime;
    t.nextScheduler->enqueue(t);
  }else {
    // Whenever a new task is created, its vRuntime is set as
    // vRuntime = minvRuntime
    t.vRuntime += Runtime::getScheduler()->minvRuntime;
    Runtime::getScheduler()->enqueue(t);
  }
    //-----
}

void Scheduler::preempt() {               // IRQs disabled, lock count inflated

//FOR ASSIGNMENT 2

  // Init Variables
  Thread* currentThread = Runtime::getCurrThread();
  Scheduler* target = currentThread->getAffinity();
  mword currPriority = Runtime::getCurrThread()->priority;
  // vRUntime of the currently running task is updated
  currentThread->vRuntime += currPriority;


  if(target != this && target){
    switchThread(target);
  }

  // Do Ticks

  //If the currently running task already ran for minGranularity
  if(currentThread->vRuntime >= schedMinGranularity){
    // if not all threads have been completed
    if(!readyTree->empty()){
    //then if (vRuntime of leftmost task in readyTree) < (vRuntime of currently)
      if(readyTree->readMinNode()->th->vRuntime < currentThread->vRuntime){
        //Insert current task back to readyTree
        enqueue(*currentThread);
        //Set minvRuntime = vRuntime of leftmost task (min of tree)
        minvRuntime = readyTree->readMinNode()->th->vRuntime;

        //Schedule the leftmost tree to run
        Thread* nextThread = readyTree->readMinNode()->th; 
        if (nextThread == currentThread){
          readyTree->popMinNode();
        } else{
          switchThread(nextThread->getAffinity());
        }
      }
    }
 
  }
  switchThread(this);

//-----
}


void Scheduler::suspend(BasicLock& lk) {
  Runtime::FakeLock fl;
  switchThread(nullptr, lk);
}

void Scheduler::suspend(BasicLock& lk1, BasicLock& lk2) {
  Runtime::FakeLock fl;
  switchThread(nullptr, lk1, lk2);
}

void Scheduler::terminate() {
  Runtime::RealLock rl;
  Thread* thr = Runtime::getCurrThread();
  GENASSERT1(thr->state != Thread::Blocked, thr->state);
  thr->state = Thread::Finishing;
  switchThread(nullptr);
  unreachable();
}

// FOR ASSIGNMENT 2
void Scheduler::adjustEpochTicks(){
  if (defaultEpochLength >= (readyCount*schedMinGranularity)){
    epochLengthTicks = defaultEpochLength;
  } else{
    epochLengthTicks = readyCount*schedMinGranularity;
  }
}
//-----
