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
#ifndef _Process_h_
#define _Process_h_ 1

#include "runtime/SynchronizedArray.h"
#include "runtime/JoinableThread.h"
#include "kernel/AddressSpace.h"
#include "kernel/MemoryManager.h"
#include "world/Access.h"

class Process : public AddressSpace {

  struct UserThread : public JoinableThread {
    mword idx;
    vaddr stackAddr;          // bottom of allocated memory for thread/stack
    size_t stackSize;         // size of allocated memory
    UserThread(vaddr ksb, size_t kss) : JoinableThread(ksb, kss) {}
    static inline UserThread* create(size_t kss = defaultStack) {
      vaddr mem = kernelSpace.allocStack(kss);
      vaddr This = mem + kss - sizeof(UserThread);
      DBG::outl(DBG::Threads, "UserThread create: ", FmtHex(mem), '/', FmtHex(kss), '/', FmtHex(This));
      return new (ptr_t(This)) UserThread(mem, kss);
    }
  };

  SpinLock threadLock;
  ManagedArray<UserThread*,KernelAllocator> threadStore;

  vaddr sigHandler;

  static void invokeUser(funcvoid2_t func, ptr_t arg1, ptr_t arg2) __noreturn;

public:
  SynchronizedArray<Access*,KernelAllocator> ioHandles; // used in syscalls.cc
  ManagedArray<Semaphore*,KernelAllocator> semStore;    // used in syscalls.cc
  SpinLock semStoreLock;                                // used in syscalls.cc

  Process() : threadStore(1), sigHandler(0), ioHandles(4) {
    ioHandles.store(knew<InputAccess>());
    ioHandles.store(knew<OutputAccess>(StdOut));
    ioHandles.store(knew<OutputAccess>(StdErr));
    ioHandles.store(knew<OutputAccess>(StdDbg));
  }
  ~Process();

  void exec(const string& fileName);
  void exit() __noreturn;

  void  setSignalHandler(vaddr sh) { sigHandler = sh; }
  vaddr getSignalHandler() { return sigHandler; }

  mword createThread(funcvoid2_t wrapper, funcvoid1_t func, ptr_t data);
  void  exitThread(ptr_t result) __noreturn;
  int   joinThread(mword idx, ptr_t& result);
  bool  destroyThread(Thread& t);

  mword getID() { return 0; }
  static mword getCurrentThreadID() {
    return reinterpret_cast<UserThread*>(LocalProcessor::getCurrThread())->idx;
  }
};

static inline Process& CurrProcess() {
  AddressSpace& as = CurrAS();
  KASSERT0(as.user());
  return reinterpret_cast<Process&>(as);
}

#endif /* _Process_h_ */
