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
#ifndef _AddressSpace_h_
#define _AddressSpace_h_ 1

#include "generic/EmbeddedContainers.h"
#include "kernel/FrameManager.h"
#include "kernel/Output.h"
#include "machine/Paging.h"

struct PageInvalidation : public EmbeddedList<PageInvalidation>::Link {
  Paging::PageEntry* pentry;
  vaddr              vma;
  size_t             size;
  mword              count;
  bool               alloc;
};

// TODO: store shared & swapped virtual memory regions in separate data
// structures - checked during page fault (swapped) resp. unmap (shared)
class AddressSpace : public Paging {
  SpinLock ulock;          // lock protecting page invalidation data
  mword activeCores;       // page invalidation data
  EmbeddedList<PageInvalidation> invList;

  SpinLock plock;          // lock protecting hardware page tables
  paddr pagetable;         // root page table *physical* address

  SpinLock vlock;          // lock protecting virtual address range
  vaddr mapBottom, mapStart, mapTop;

  const bool kernel;

  AddressSpace(const AddressSpace&) = delete;                  // no copy
  const AddressSpace& operator=(const AddressSpace&) = delete; // no assignment

  enum MapCode { NoAlloc, Alloc, Guard, Lazy };

  template<unsigned int N, bool present>
  bool mapPage(vaddr vma, paddr pma, uint64_t type) {
    ScopedLock<> sl(plock);
    return Paging::map<N,present>(vma, pma, type, *LocalProcessor::getFrameManager());
  }

  template<size_t N, MapCode mc>
  void mapPageRegion( paddr pma, vaddr vma, size_t size, uint64_t type ) {
    static_assert( N > 0 && N < pagelevels, "page level template violation" );
    static_assert( mc >= NoAlloc && mc <= Lazy, "invalid MapCode" );
    KASSERT1( aligned(vma, pagesize<N>()), vma );
    KASSERT1( aligned(size, pagesize<N>()), size );
    for (vaddr end = vma + size; vma < end; vma += pagesize<N>()) {
      switch (mc) {
        case NoAlloc: break;
        case Alloc:   pma = LocalProcessor::getFrameManager()->allocFrame<N>(); break;
        case Guard:   pma = guardPage; break;
        case Lazy:    pma = lazyPage; break;
      }
      KASSERT0(pma != topaddr);
      if (!kernel) type |= User;
      DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/map<", N, ">: ", FmtHex(vma), " -> ", FmtHex(pma), " flags:", Paging::FmtPE(type));
      bool check;
      switch (mc) {
        case NoAlloc:
        case Alloc:   check = mapPage<N,true>(vma, pma, type); break;
        case Guard:   check = mapPage<N,false>(vma, pma, type); break;
        case Lazy:    check = mapPage<N,false>(vma, pma, type); break;
      }
      KASSERT1(check, vma);
      if (mc == NoAlloc) pma += pagesize<N>();
    }
  }

  template<unsigned int N, bool async=false>
  PageEntry* unmapPage1(vaddr vma) {
    ScopedLock<> sl(plock);
    return Paging::unmap1<N,async>(vma);
  }

  paddr unmapPage2(PageEntry* pe) {
    ScopedLock<> sl(plock);
    return Paging::unmap2(pe);
  }

  template<unsigned int N>
  paddr unmapPage(vaddr vma) {
    ScopedLock<> sl(plock);
    return Paging::unmap<N>(vma);
  }

  template<size_t N, bool alloc, bool direct>
  void unmapPageRegion( vaddr vma, size_t size ) {
    static_assert( N > 0 && N < pagelevels, "page level template violation" );
    KASSERT1( aligned(vma, pagesize<N>()), vma );
    KASSERT1( aligned(size, pagesize<N>()), size );
    for (vaddr end = vma + size; vma < end; vma += pagesize<N>()) {
      if (direct) {
        paddr pma = unmapPage<N>(vma);
        DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/unmap: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> ", FmtHex(pma));
        CPU::InvTLB(vma);
        if (alloc && pma != lazyPage && pma != guardPage) LocalProcessor::getFrameManager()->releaseFrame<N>(pma);
      } else {
        PageEntry* pe = unmapPage1<N,true>(vma);
    if (!pe) continue;
        ulock.acquire();
        if (activeCores > 1) {
          DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/post: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), ":", activeCores, " PE:", FmtHex(pe));
          PageInvalidation* pi = invList.back();
          pi->pentry = pe;
          pi->vma = vma;
          pi->size = pagesize<N>();
          pi->count = activeCores;
          pi->alloc = alloc;
          invList.push_back(*knew2<PageInvalidation>());
          ulock.release();
        } else {
          ulock.release();
          paddr pma = unmapPage2(pe);
          DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/unmap2: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> ", FmtHex(pma), " PE:", FmtHex(pe));
          CPU::InvTLB(vma);
        }
      }
    }
  }

  template<size_t N>
  vaddr getVmRange(vaddr addr, size_t& size) {
    KASSERT1(mapBottom < mapTop, "no AS memory break set yet");
    ScopedLock<> sl(vlock);
    vaddr end = addr ? align_up(addr + size, pagesize<N>()) : align_down(mapStart, pagesize<N>());
    vaddr start = align_down(end - size, pagesize<N>());
    if (start < mapBottom) return topaddr;
    if (start < mapStart) mapStart = start;
    size = end - start;
    DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/get: ", FmtHex(start), '-', FmtHex(end));
    return start;
  }

  void putVmRange(vaddr addr, size_t size) {
    vaddr end = addr + size;
    ScopedLock<> sl(vlock);
    if (addr <= mapStart && end >= mapStart) {
      while ((size = Paging::test(end, Available)) && end + size <= mapTop) end += size;
      DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/put: ", FmtHex(mapStart), '-', FmtHex(end));
      mapStart = end;
    }
  }

  template <bool invalidate>
  void runInvalidation(PageInvalidation* pi) {
    KASSERT0(this);
    while (pi != invList.back()) {
      if (invalidate) CPU::InvTLB(pi->vma);
      KASSERT0(pi->count > 0);
      pi->count -= 1;
      PageInvalidation* npi = invList.next(*pi);
      if (pi->count == 0) {
        DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/inv: ", FmtHex(pi->vma), '/', FmtHex(pi->size), ":", pi->count, " PE:", FmtHex(pi->pentry));
        paddr pma = unmapPage2(pi->pentry);
        if (pi->alloc && pma != lazyPage && pma != guardPage) LocalProcessor::getFrameManager()->releaseFrames(pma, pi->size);
        putVmRange(pi->vma, pi->size);
        invList.remove(*pi);
        kdelete2(pi);
      }
      pi = npi;
    }
  }

public:
  inline AddressSpace(const bool k = false);

  ~AddressSpace() {
    KASSERT0(!kernel); // kernelSpace is never destroyed
    KASSERT0(pagetable != topaddr);
    DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/destruct:", FmtHex(pagetable));
    KASSERT1(pagetable != CPU::readCR3(), FmtHex(CPU::readCR3()));
    KASSERT0(invList.front() == invList.back());
    kdelete2(invList.back());
    LocalProcessor::getFrameManager()->releaseFrame<pagetablepl>(pagetable);
  }

  bool user() const { return !kernel; }

  void clearUserPaging() {
    KASSERT0(!kernel); // kernelSpace is never destroyed
    KASSERT0(pagetable != topaddr);
    DBG::outl(DBG::VM, "AS(", FmtHex(pagetable), ")/destroy:", *this);
    runInvalidation<false>(LocalProcessor::getUserPI());
    LocalProcessor::setUserPI(invList.back());
    clearAll(align_down(userbot, pagesize<pagelevels>()), align_up(usertop, pagesize<pagelevels>()), *LocalProcessor::getFrameManager());
  }

  void initKernel(vaddr bot, vaddr top, paddr pt) {
    KASSERT0(kernel);
    pagetable = pt;
    mapBottom = bot;
    mapStart = mapTop = top;
    invList.push_back(*knew2<PageInvalidation>());
  }

  void initUser(vaddr bssEnd) {
    KASSERT0(!kernel);
    mapBottom = bssEnd;
    mapStart = mapTop = usertop;
  }

  PageInvalidation* initProcessor() {
    KASSERT0(kernel);
    ScopedLock<> slk(ulock);
    activeCores += 1;
    return invList.back();
  }

  void runKernelInvalidation() {
    KASSERT0(kernel);
    ScopedLock<> sl(ulock);
    runInvalidation<true>(LocalProcessor::getKernPI());
    LocalProcessor::setKernPI(invList.back());
  }

  template<bool lock=false>
  AddressSpace& enter() {
    AddressSpace* prevAS = LocalProcessor::getCurrAS();
    KASSERT0(prevAS);
    KASSERTN(prevAS->pagetable == CPU::readCR3(), FmtHex(prevAS->pagetable), ' ', FmtHex(CPU::readCR3()));
    KASSERT0(pagetable != topaddr);
    if (prevAS != this) {
      DBG::outl(DBG::Scheduler, "AS switch: ", FmtHex(prevAS->pagetable), " -> ", FmtHex(pagetable));
      if (!prevAS->kernel) {
        ScopedLock<> sl(prevAS->ulock);
        prevAS->runInvalidation<false>(LocalProcessor::getUserPI());
        prevAS->activeCores -= 1;
      }
      if (lock) LocalProcessor::lock(true);
      installPagetable(pagetable);
      LocalProcessor::setCurrAS(this);
      if (lock) LocalProcessor::unlock(true);
      if (!kernel) {
        ScopedLock<> sl(ulock);
        LocalProcessor::setUserPI(invList.back());
        activeCores += 1;
      }
    }
    return *prevAS;
  }

  template<size_t N, bool alloc=true>
  vaddr map(vaddr addr, size_t size, mword prot, mword flags, mword filedes, mword off, paddr pma = 0) {
    KASSERT1(prot == 0, prot);
    KASSERT1(flags == 0, flags);
    KASSERT1(filedes == mword(-1), filedes);
    KASSERT1(off == 0, off);
    vaddr start = getVmRange<N>(addr, size);
    if (kernel) mapPageRegion<N,alloc ? Alloc : NoAlloc>(pma, start, size, Data);
#if TESTING_NEVER_ALLOC_LAZY
    else mapPageRegion<N,alloc ? Alloc : NoAlloc>(pma, start, size, Data);
#else
    else mapPageRegion<N,alloc ? Lazy : NoAlloc>(pma, start, size, Data);
#endif
    return start;
  }

  template<size_t N, bool alloc=true>
  void unmap(vaddr addr, size_t size) {
    KASSERT1(aligned(addr, pagesize<N>()), addr);
    size = align_up(size, pagesize<N>());
    unmapPageRegion<N,alloc,false>(addr, size);
  }

  template<size_t N, bool alloc=true>
  vaddr kmap(vaddr addr, size_t size, paddr pma = 0) {
    vaddr start = getVmRange<N>(addr, size);
    mapPageRegion<N,alloc ? Alloc : NoAlloc>(pma, start, size, KernelData);
    return start;
  }

  vaddr allocStack(size_t ss) {
    KASSERT1(ss >= minimumStack, ss);
    size_t size = ss + stackGuardPage;
    vaddr vma = getVmRange<stackpl>(0, size);
    KASSERT0(vma != topaddr);
    KASSERTN(size == ss + stackGuardPage, ss, ' ', size);
    mapPageRegion<stackpl,Guard>(0, vma, stackGuardPage, Data);
    vma += stackGuardPage;
    if (kernel) mapPageRegion<stackpl,Alloc>(0, vma, ss, Data);
#if TESTING_NEVER_ALLOC_LAZY
    else mapPageRegion<stackpl,Alloc>(0, vma, ss, Data);
#else
    else  mapPageRegion<stackpl,Lazy>(0, vma, ss, Data);
#endif
    return vma;
  }

  void releaseStack(vaddr vma, size_t ss) {
    unmapPageRegion<stackpl,true,false>(vma - stackGuardPage, ss + stackGuardPage);
  }

  template<size_t N,bool check=true> // allocate memory and map to specific virtual address
  void allocDirect( vaddr vma, size_t size, PageType t ) {
    KASSERT1(!check || vma < mapBottom || vma > mapTop, vma);
    mapPageRegion<N,Alloc>(0, vma, size, t);
  }

  template<size_t N, bool check=true> // unmap & free allocated memory
  void releaseDirect( vaddr vma, size_t size ) {
    KASSERT1(!check || vma < mapBottom || vma > mapTop, vma);
    unmapPageRegion<N,true,true>(vma, size);
  }  

  template<size_t N, bool check=true> // map memory to specific virtual address
  void mapDirect( paddr pma, vaddr vma, size_t size, PageType t ) {
    KASSERT1(!check || vma < mapBottom || vma > mapTop, vma);
    mapPageRegion<N,NoAlloc>(pma, vma, size, t);
  }

  template<size_t N, bool check=true> // unmap memory
  void unmapDirect( vaddr vma, size_t size ) {
    KASSERT1(!check || vma < mapBottom || vma > mapTop, vma);
    unmapPageRegion<N,false,true>(vma, size);
  }

  void print(ostream& os) const;
};

inline ostream& operator<<(ostream& os, const AddressSpace& as) {
  as.print(os);
  return os;
}

extern AddressSpace kernelSpace;

inline AddressSpace::AddressSpace(const bool k) : activeCores(0),
  pagetable(topaddr), mapBottom(0), mapStart(0), mapTop(0), kernel(k) {
  if (!kernel) { // shallow copy; clone from user AS -> make deep copy!
    kernelSpace.plock.acquire();
    pagetable = Paging::cloneKernelPT(*LocalProcessor::getFrameManager());
    kernelSpace.plock.release();
    DBG::outl(DBG::VM, "AS(", FmtHex(kernelSpace.pagetable), ")/cloned: ", FmtHex(pagetable));
    invList.push_back(*knew2<PageInvalidation>());
  }
}

static inline AddressSpace& CurrAS() {
  AddressSpace* as = LocalProcessor::getCurrAS();
  KASSERT0(as);
  return *as;
}

#endif /* _AddressSpace_h_ */
