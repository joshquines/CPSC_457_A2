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
#ifndef _Processor_h_
#define _Processor_h_ 1

#include "machine/asmdecl.h"
#include "machine/asmshare.h"
#include "machine/CPU.h"
#include "machine/Descriptors.h"
#include "machine/Memory.h"

class Thread;
class AddressSpace;
class FrameManager;
class Scheduler;
struct PageInvalidation;

class Processor {
  friend class Machine;             // init, setup, and IPI routines
  friend class LocalProcessor;      // member offsets for %gs-based access

  /* execution context */
  mword             lockCount;
  Thread*           currThread;
  AddressSpace*     currAS;
  PageInvalidation* userPI;
  PageInvalidation* kernPI;

  /* processor configuration */
  Scheduler*    scheduler;
  FrameManager* frameManager;

  /* processor information */
  mword         index;
  mword         apicID;
  mword         systemID;

  /* task state segment: kernel stack for interrupts/exceptions */
  static const unsigned int nmiIST = 1;
  static const unsigned int dbfIST = 2;
  static const unsigned int stfIST = 3;
  static const unsigned int pgfIST = 4;
  TaskStateSegment tss;
  buf_t faultStack[minimumStack];

  // layout for syscall/sysret, because of (strange) rules for SYSCALL_LSTAR
  // SYSCALL_LSTAR essentially forces userCS = userDS + 1
  // layout for sysenter/sysexit would be: kernCS, kernDS, userCS, userDS
  // also: kernDS = 2 consistent with convention and segment setup in boot.S
  static const unsigned int kernCS  = 1;
  static const unsigned int kernDS  = 2;
  static const unsigned int userDS  = 3;
  static const unsigned int userCS  = 4;
  static const unsigned int tssSel  = 5; // TSS entry uses 2 entries
  static const unsigned int maxGDT  = 7;
  SegmentDescriptor gdt[maxGDT];

  void install()                                              __section(".boot.text");
  static void check(bool output)                              __section(".boot.text");
  void init(paddr, InterruptDescriptor*, size_t, funcvoid0_t) __section(".boot.text");
  void setupGDT(uint32_t n, uint32_t dpl, bool code)          __section(".boot.text");
  void setupTSS(uint32_t num, paddr addr)                     __section(".boot.text");

  Processor(const Processor&) = delete;            // no copy
  Processor& operator=(const Processor&) = delete; // no assignment

  void setup(AddressSpace& as, PageInvalidation* ki, Scheduler& s, FrameManager& fm, mword idx, mword apic, mword sys) {
    currAS = &as;
    kernPI = ki;
    scheduler = &s;
    frameManager = &fm;
    index = idx;
    apicID = apic;
    systemID = sys;
  }

public:
  Processor() : lockCount(1), currThread(nullptr), currAS(nullptr),
    userPI(nullptr), kernPI(nullptr), scheduler(nullptr),
    frameManager(nullptr), index(0), apicID(0), systemID(0) {}

  static inline bool userSegment(mword cs) {
    // check for not kernCS, because userCS (always?) has bits 0,1 set
    return cs != (kernCS * sizeof(SegmentDescriptor));
  }
} __packed __caligned;

class LocalProcessor {
  static void enableInterrupts() { asm volatile("sti"); }
  static void disableInterrupts() { asm volatile("cli"); }
  static void incLockCount() {
    asm volatile("addq $1, %%gs:%c0" :: "i"(offsetof(Processor, lockCount)) : "cc");
  }
  static void decLockCount() {
    asm volatile("subq $1, %%gs:%c0" :: "i"(offsetof(Processor, lockCount)) : "cc");
  }

public:
  static void initInterrupts(bool irqs);
  static mword getLockCount() {
    mword x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, lockCount)));
    return x;
  }
  static mword checkLock() {
    KASSERT1(CPU::interruptsEnabled() == (getLockCount() == 0), getLockCount());
    return getLockCount();
  }
  static void lockFake() {
    KASSERT0(!CPU::interruptsEnabled());
    incLockCount();
  }
  static void unlockFake() {
    KASSERT0(!CPU::interruptsEnabled());
    decLockCount();
  }
  static void lock(bool check = false) {
    if (check) KASSERT1(checkLock() == 0, getLockCount());
    // despite looking like trouble, I believe this is safe: race could cause
    // multiple calls to disableInterrupts(), but this is no problem!
    if slowpath(getLockCount() == 0) disableInterrupts();
    incLockCount();
  }
  static void unlock(bool check = false) {
    if (check) KASSERT1(checkLock() == 1, getLockCount());
    decLockCount();
    // no races here (interrupts disabled)
    if slowpath(getLockCount() == 0) enableInterrupts();
  }


  static Thread* getCurrThread() {
    Thread* x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, currThread)));
    return x;
  }
  static void setCurrThread(Thread* x) {
    asm volatile("movq %0, %%gs:%c1" :: "r"(x), "i"(offsetof(Processor, currThread)));
  }
  static AddressSpace* getCurrAS() {
    AddressSpace* x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, currAS)));
    return x;
  }
  static void setCurrAS(AddressSpace* x) {
    asm volatile("movq %0, %%gs:%c1" :: "r"(x), "i"(offsetof(Processor, currAS)));
  }
  static PageInvalidation* getUserPI() {
    PageInvalidation* x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, userPI)));
    return x;
  }
  static void setUserPI(PageInvalidation* x) {
    asm volatile("movq %0, %%gs:%c1" :: "r"(x), "i"(offsetof(Processor, userPI)));
  }
  static PageInvalidation* getKernPI() {
    PageInvalidation* x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, kernPI)));
    return x;
  }
  static void setKernPI(PageInvalidation* x) {
    asm volatile("movq %0, %%gs:%c1" :: "r"(x), "i"(offsetof(Processor, kernPI)));
  }
  static Scheduler* getScheduler() {
    Scheduler* x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, scheduler)));
    return x;
  }
  static FrameManager* getFrameManager() {
    FrameManager* x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, frameManager)));
    return x;
  }
  static mword getIndex() {
    mword x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, index)));
    return x;
  }
  static mword getApicID() {
    mword x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, apicID)));
    return x;
  }
  static mword getSystemID() {
    mword x;
    asm volatile("movq %%gs:%c1, %0" : "=r"(x) : "i"(offsetof(Processor, systemID)));
    return x;
  }
  static void setKernelStack() {
    static const mword offset = offsetof(Processor, tss) + offsetof(TaskStateSegment, rsp);
    static_assert(offset == TSSRSP, "TSSRSP");
    asm volatile("movq %0, %%gs:%c1" :: "r"(getCurrThread()), "i"(offset));
  }
};

#endif /* Processor_h_ */
