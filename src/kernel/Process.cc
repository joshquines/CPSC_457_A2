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
#include "runtime/Scheduler.h"
#include "runtime/Thread.h"
#include "kernel/Process.h"
#include "extern/elfio/elfio.hpp"

void Process::invokeUser(funcvoid2_t func, ptr_t arg1, ptr_t arg2) {
  UserThread* ut = reinterpret_cast<UserThread*>(LocalProcessor::getCurrThread());
  KASSERT0(ut);
  ut->stackSize = defaultUserStack;
  ut->stackAddr = CurrAS().allocStack(ut->stackSize);
  DBG::outl(DBG::Threads, "UThread start: ", FmtHex(ut), '/', FmtHex((ptr_t)func));
  startUserCode(arg1, arg2, vaddr(ut), func, ut->stackAddr + ut->stackSize);
  unreachable();
}

Process::~Process() {
  DBG::outl(DBG::Threads, "Process delete: ", FmtHex(this));
  for (size_t i = 0; i < ioHandles.currentIndex(); i += 1) {
    Access* a = ioHandles.access(i);
    if (a) kdelete(a);
  }
  for (size_t i = 0; i < semStore.currentIndex(); i += 1) {
    if (semStore.valid(i)) kdelete(semStore.get(i));
  }
}

void Process::exec(const string& fileName) {
  KASSERT0(threadStore.empty());
  AddressSpace& as = this->enter<true>();
  auto iter = kernelFS.find(fileName);
  KASSERT1(iter != kernelFS.end(), fileName.c_str())
  RamFile& rf = iter->second;
  ELFIO::elfio elfReader;
  bool check = elfReader.load(fileName.c_str());
  KASSERT0(check);
  KASSERT1(elfReader.get_class() == ELFCLASS64, elfReader.get_class());

  vaddr currBreak = 0;
  for (int i = 0; i < elfReader.segments.size(); ++i) {
    const ELFIO::segment* pseg = elfReader.segments[i];
    if (pseg->get_type() != PT_LOAD) continue;  // not a loadable segment

    KASSERTN(pseg->get_file_offset() + pseg->get_file_size() <= rf.size, FmtHex(pseg->get_file_offset()), ' ', FmtHex(pseg->get_file_size()), ' ', FmtHex(rf.size));
    KASSERTN(pseg->get_memory_size() >= pseg->get_file_size(), FmtHex(pseg->get_file_size()), ' ', FmtHex(pseg->get_memory_size()));
    paddr pma = rf.pma + pseg->get_file_offset();
    paddr apma = align_down(pma, pagesize<1>());
    vaddr vma = pseg->get_virtual_address();
    vaddr avma = align_down(vma, pagesize<1>());
    KASSERTN(vma - avma == pma - apma, FmtHex(vma), ' ', FmtHex(pma));
    vaddr fend = vma + pseg->get_file_size();
    vaddr afend = align_up(fend, pagesize<1>());
    vaddr mend = vma + pseg->get_memory_size();
    vaddr amend = align_up(mend, pagesize<1>());

    // If .rodata and .text are in the same elf segment and small enough to
    // fit into a single page, then .rodata ends up being marked executable.
    PageType pageType = (pseg->get_flags() & PF_W) ? Data :
      (pseg->get_flags() & PF_X) ? Code : RoData;

    DBG::outl(DBG::Process,
      pageType == Data ? "data" : pageType == Code ? "code" : "ro",
      " segment: ", FmtHex(vma), '-', FmtHex(fend));
    mapDirect<1>(apma, avma, afend - avma, pageType);

    if (mend > fend) {
      DBG::outl(DBG::Process, "bss: ", FmtHex(fend), '-', FmtHex(mend));
      if (amend > afend) allocDirect<1>(afend, amend - afend, Data);
      memset((void*)fend, 0, mend - fend);
    }
    if (mend > currBreak) currBreak = mend;
  }

  initUser(currBreak);
  ptr_t entry = (ptr_t)elfReader.get_entry();
  DBG::outl(DBG::Process, "entry: ", FmtHex(entry));
  createThread((funcvoid2_t)entry, (funcvoid1_t)nullptr, nullptr);
  as.enter<true>();
}

// detach all -> cancel all
void Process::exit() {
  UserThread* ut = reinterpret_cast<UserThread*>(LocalProcessor::getCurrThread());
  KASSERT0(ut);
  DBG::outl(DBG::Threads, "Process exit: ", FmtHex(ut));
  threadLock.acquire();
  for (mword i = 0; i < threadStore.currentIndex(); i += 1) {
    if fastpath(threadStore.valid(i) && threadStore.get(i) != ut)	{
      threadStore.get(i)->cancel();
    }
  }
  threadLock.release();
  LocalProcessor::getScheduler()->terminate();
}

mword Process::createThread(funcvoid2_t wrapper, funcvoid1_t func, ptr_t data) {
  UserThread* ut = UserThread::create();
  KASSERT0(ut);
  threadLock.acquire();
  ut->idx = threadStore.put(ut);
  DBG::outl(DBG::Threads, "UThread create: ", FmtHex(ut), '/', ut->idx);
  ut->start((ptr_t)invokeUser, (ptr_t)wrapper, (ptr_t)func, data);
  threadLock.release();
  return ut->idx;
}

void Process::exitThread(ptr_t result) {
  UserThread* ut = reinterpret_cast<UserThread*>(LocalProcessor::getCurrThread());
  KASSERT0(ut);
  DBG::outl(DBG::Threads, "UThread exit: ", FmtHex(ut), '/', ut->idx);
  releaseStack(ut->stackAddr, ut->stackSize);
  threadLock.acquire();
  if (threadStore.size() > 1) ut->post(result, threadLock);
  else threadLock.release();
  LocalProcessor::getScheduler()->terminate();
}

int Process::joinThread(mword idx, ptr_t& result) {
  DBG::outl(DBG::Threads, "UThread join: ", idx);
  threadLock.acquire();
  if (!threadStore.valid(idx)) {
    threadLock.release();
    return -ESRCH;
  }
  if (!threadStore.get(idx)->join(result, threadLock)) {
    threadLock.release();
    return -EINVAL;
  }
  return 0;
}

bool Process::destroyThread(Thread& t) {
  UserThread& ut = reinterpret_cast<UserThread&>(t);
  ScopedLock<> sl(threadLock);
  threadStore.remove(ut.idx);
  return threadStore.empty();
}
