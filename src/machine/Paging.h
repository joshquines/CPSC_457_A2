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
#ifndef _Paging_h_
#define _Paging_h_ 1

#include "kernel/FrameManager.h"
#include "kernel/Output.h"
#include "machine/asmshare.h"
#include "machine/CPU.h"

#include <cstring>

/* page map access with recursive PML4: address of 8-byte aligned page entry
bit width:    9999 9 3
target vaddr: ABCD|x|y
PML4 self     XXXX|X|0
L4 entry      XXXX|A|0
L3 entry      XXXA|B|0
L2 entry      XXAB|C|0
L1 entry      XABC|D|0
with X = per-level bit pattern (position of recursive index in PML4). */

class Paging {
  friend class Machine;

public:
  typedef uint64_t PageEntry; // struct PageInvalidation
  struct PageFaultFlags {     // exception_handler_0x0e
    uint64_t t;
    static const BitString<uint64_t, 0, 1> P;		 // page not present
    static const BitString<uint64_t, 1, 1> WR;   // write not allowed
    static const BitString<uint64_t, 2, 1> US;   // user access not allowed
    static const BitString<uint64_t, 3, 1> RSVD; // reserved flag in paging structure
    static const BitString<uint64_t, 4, 1> ID;   // instruction fetch
    PageFaultFlags( uint64_t t ) : t(t) {}
  };
  friend ostream& operator<<(ostream&, const PageFaultFlags&);

private:
  static const BitString<uint64_t, 0, 1> P;
  static const BitString<uint64_t, 1, 1> RW;
  static const BitString<uint64_t, 2, 1> US;
  static const BitString<uint64_t, 3, 1> PWT;
  static const BitString<uint64_t, 4, 1> PCD;
  static const BitString<uint64_t, 5, 1> A;
  static const BitString<uint64_t, 6, 1> D;
  static const BitString<uint64_t, 7, 1> PS;
  static const BitString<uint64_t, 8, 1> G;
  static const BitString<uint64_t,12,40> ADDR;
  static const BitString<uint64_t,63, 1> XD;

protected:
  enum PageType {
    Code       = 0,
    User       = US(),
    RoData     = XD(),
    Data       = XD() | RW(),
    KernelData = XD() | RW() | G(),
    MMapIO     = XD() | RW() | PWT() | PCD(),
    KernelPT   = RW() | P(),           // NOTE: setting G() upsets VirtualBox
    PageTable  = RW() | P() | US(),
  };

  enum PageStatus {
    Available = 0x01,
    Unmapped  = 0x02,
    Reserved  = 0x04,
    Mapped    = 0x08
  };

  struct FmtPE {
    PageEntry t;
    FmtPE(PageEntry t) : t(t) {}
  };
  friend ostream& operator<<(ostream&, const FmtPE&);

private:
	template<unsigned int N> uint64_t PAT(PageEntry c) {
    static_assert( N >= 1 && N < pagelevels, "illegal template parameter" );
		if (N == 1) return PS.get(c);
		else return ADDR.get(c) & pagesize<1>();
  }

  // recursively compute page table prefix at level N
  // specialization for <1> below (must be outside of class scope)
  template<unsigned int N> static constexpr mword ptprefix() {
    static_assert( N > 0 && N <= pagelevels, "page level template violation" );
    return (recptindex << pagesizebits<1+pagelevels-N>()) | ptprefix<N-1>();
  }

  // ptprefix<1>()...ptend() is the VM region occupied by recursive page table
  static constexpr mword ptend() {
    return canonPrefix | ((recptindex + 1) << pagesizebits<pagelevels>());
  }

  // compute beginning of page table for vma
  template <unsigned int N> static constexpr PageEntry* getTable( vaddr vma ) {
    static_assert( N > 0 && N <= pagelevels, "page level template violation" );
    return (PageEntry*)(ptprefix<N>() | 
      (((vma & bitmask<PageEntry>(pagebits)) >> pagesizebits<N+1>()) << pagesizebits<1>()));
  }

  // compute page table entry fpr vma
  template <unsigned int N> static constexpr PageEntry* getEntry( vaddr vma ) {
    static_assert( N > 0 && N <= pagelevels, "page level template violation" );
    return (PageEntry*)(ptprefix<N>() |
      (((vma & bitmask<PageEntry>(pagebits)) >> pagesizebits<N>()) << 3));
  }

  // compute index in page table for vma
  template<unsigned int N> static constexpr mword getIndex(mword x) {
    return (x & bitmask<mword>(pagesizebits<N+1>())) >> pagesizebits<N>();
  }

  template<unsigned int N> static constexpr PageEntry pageBit() {
    static_assert( N > 0 && N < pagelevels, "page level template violation" );
    return N > 1 ? PS() : 0;
  }

  template<unsigned int N> static constexpr bool isPage(const PageEntry& pe) {
    static_assert( N > 0 && N <= pagelevels, "page level template violation" );
    return (N < pagelevels) && (N == 1 || PS.get(pe));
  }

  static void setPE(PageEntry& pe, const PageEntry& newpe) { pe = newpe; }

  // implementation below after ptprefix<0> template specialization
  static inline paddr bootstrap(vaddr kernelEnd);
  static inline void bootstrap2(FrameManager& fm);

  template <unsigned int N>
  static inline bool mapTable( vaddr vma, FrameManager& fm ) __useresult;

protected:
  static const paddr guardPage = topaddr & ADDR();
  static const paddr lazyPage =  guardPage - pagesize<1>();

  template <unsigned int N, bool present>
  static inline bool map( vaddr vma, paddr pma, uint64_t type, FrameManager& fm ) __useresult;

  template <unsigned int N, bool async=false>
  static PageEntry* unmap1( vaddr vma ) {
    static_assert( N >= 1 && N <= pagelevels, "page level template violation" );
    KASSERT1(aligned(vma, pagesize<N>()), FmtHex(vma));
    PageEntry* pe = getEntry<N>(vma);
    KASSERT1(isPage<N>(*pe), FmtHex(vma));
    if (P.get(*pe)) {
      DBG::outl(DBG::Paging, "Paging::unmap1<", N, ">: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> ", FmtPE(*pe));
      if (async) setPE( *pe, *pe & ~P() );
      return pe;
    } else {
      KASSERT1(((*pe & ADDR()) == lazyPage) || ((*pe & ADDR()) == guardPage), FmtHex(vma));
      DBG::outl(DBG::Paging, "Paging::unmap1<", N, ">: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> 0");
      setPE( *pe, 0 );
      return nullptr;
    }
  }

  static paddr unmap2(PageEntry* pe) {
    paddr pma = *pe & ADDR();
    setPE( *pe, 0 );
    return pma;
  }

  template <unsigned int N>
  static paddr unmap( vaddr vma ) {
    PageEntry* pe = unmap1<N,false>(vma);
    return unmap2(pe);
  }

  template <unsigned int N=pagelevels>
  static size_t test( vaddr vma, uint64_t status ) {
    static_assert( N > 0 && N <= pagelevels, "page level template violation" );
    PageEntry* pe = getEntry<N>(vma);
    PageStatus s;
    if (P.get(*pe)) {
      if (isPage<N>(*pe)) s = Mapped;
      else return test<N-1>(vma, status);
    } else switch (*pe & ADDR()) {
      case lazyPage:
      case guardPage: s = Reserved;  break;
      case 0:         s = Available; break;
      default:        s = Unmapped;  break;
    }
    return (s & status) ? pagesize<N>() : 0;
  }

  static size_t testfree( vaddr vma ) { return test(vma, Available|Unmapped); }
  static size_t testused( vaddr vma ) { return test(vma, Reserved|Mapped); }

  template <unsigned int N = pagelevels>
  static void clearAll( vaddr start, vaddr end, FrameManager& fm ) {
    static_assert( N >= 1 && N <= pagelevels, "page level template violation" );
    for (vaddr vma = start; vma < end; vma += pagesize<N>()) {
      PageEntry* pe = getEntry<N>(vma);
      if (P.get(*pe)) {
        paddr pma = *pe & ADDR();
        if (isPage<N>(*pe)) {
          DBG::outl(DBG::Paging, "Paging::clearAllP<", N, ">: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> ", FmtPE(*pe));
          KASSERT1(N < pagelevels, N);
          fm.releaseFrame<N>(pma);
        } else {
          clearAll<N-1>(vma, vma + pagesize<N>(), fm);
          DBG::outl(DBG::Paging, "Paging::clearAllT<", N-1, ">: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> ", FmtPE(*pe));
          fm.releaseFrame<pagetablepl>(pma);
        }
      }
      setPE( *pe, 0 );
    }
  }

  static void installPagetable(paddr pt) {
    CPU::writeCR3(pt);
  }

  static inline paddr cloneKernelPT( FrameManager& fm );

  Paging() = default;
  Paging(const Paging&) = delete;            // no copy
  Paging& operator=(const Paging&) = delete; // no assignment

public:
  template <unsigned int N = pagelevels>
  static inline bool fault( vaddr vma, FrameManager& fm ) __useresult;

  template<unsigned int N = pagelevels>
  static paddr vtop( vaddr vma ) {
    static_assert( N > 0 && N <= pagelevels, "page level template violation" );
    PageEntry* pe = getEntry<N>(vma);
    KASSERT1(P.get(*pe), FmtHex(vma));
    if (!isPage<N>(*pe)) return vtop<N-1>(vma);
    return (*pe & ADDR()) + pageoffset<N>(vma);
  }
};

// corner cases for which dummy template instantiations are needed
template<> inline size_t Paging::test<0>(vaddr, uint64_t ) { KABORT0(); return 0; }
template<> inline void Paging::clearAll<0>(vaddr, vaddr, FrameManager&) { KABORT0(); }
template<> inline bool Paging::fault<0>(vaddr, FrameManager&) { KABORT0(); return false; }
template<> inline paddr Paging::vtop<0>(vaddr) { KABORT0(); return 0; }

// corner cases for which actual template instantiations are needed
template<> inline constexpr mword Paging::ptprefix<0>() {
  return recptindex < pagetableentries/2 ? 0 : canonPrefix;
}

// function definition outside class, because of __useresult
template <unsigned int N>
inline bool Paging::mapTable( vaddr vma, FrameManager& fm ) {
  static_assert( N >= 1 && N < pagelevels, "page level template violation" );
  if (!mapTable<N+1>(vma, fm)) return false;
  PageEntry* pe = getEntry<N+1>(vma);
  if (!P.get(*pe)) {
    paddr pma = fm.allocFrame<pagetablepl>();
    setPE( *pe, pma | PageTable );
    DBG::outl(DBG::Paging, "Paging::/mapT<", N, ">: ", FmtHex(align_down(vma, pagesize<N+1>())), '/', FmtHex(pagesize<N+1>()), " -> ", FmtPE(*pe), " created");
    memset( getTable<N>(vma), 0, pagesize<pagetablepl>() );	// TODO: alloczero
    return true;
  }
  if (!isPage<N+1>(*pe)) {
    DBG::outl(DBG::Paging, "Paging::mapT<", N, ">: ", FmtHex(align_down(vma, pagesize<N+1>())), '/', FmtHex(pagesize<N+1>()), " -> ", FmtPE(*pe), " checked");
    return true;
  }
  DBG::outl(DBG::Paging, "Paging::mapT<", N, ">: ", FmtHex(align_down(vma, pagesize<N+1>())), '/', FmtHex(pagesize<N+1>()), " -> ", FmtPE(*pe), " is page!");
  return false;
}

// template specialization outside class
template<> inline bool Paging::mapTable<pagelevels>(vaddr, FrameManager&) {
  return true;
}

// function definition outside class, because of __useresult
template <unsigned int N, bool present>
inline bool Paging::map( vaddr vma, paddr pma, uint64_t type, FrameManager& fm ) {
  static_assert( N >= 1 && N < pagelevels, "page level template violation" );
  KASSERT1( aligned(vma, pagesize<N>()), FmtHex(vma) );
  KASSERT1( (pma & ~ADDR()) == 0, FmtHex(pma) );
  if (!mapTable<N>(vma, fm)) return false;
  PageEntry* pe = getEntry<N>(vma);
  KASSERT1( !P.get(*pe), FmtHex(vma) );
  setPE( *pe, pma | type | pageBit<N>() | (present ? P() : 0) );
  DBG::outl(DBG::Paging, "Paging::map<", N, present ? ",P" : ",R", ">: ", FmtHex(vma), '/', FmtHex(pagesize<N>()), " -> ", FmtPE(*pe));
  return true;
}

// function definition outside class, because of __useresult
template <unsigned int N>
inline bool Paging::fault( vaddr vma, FrameManager& fm ) {
  static_assert( N > 0 && N <= pagelevels, "page level template violation" );
  PageEntry* pe = getEntry<N>(vma);
  if (P.get(*pe)) {
    if (!isPage<N>(*pe)) return fault<N-1>(vma, fm);
  } else if (isPage<N>(*pe) && (*pe & ADDR()) == lazyPage) {
    paddr pma = fm.allocFrame<N>();
    KASSERT0(pma != topaddr);
    setPE( *pe, pma | (*pe & ~ADDR()) | P() );
//    DBG::outl(DBG::Paging, "Paging::fault<", N, ">: ", FmtHex(align_down(vma, pagesize<N>())), '/', FmtHex(pagesize<N>()), " -> ", FmtPE(*pe));
    return true;
  }
//  DBG::outl(DBG::Paging, "Paging::fault<", N, ">: ", FmtHex(align_down(vma, pagesize<N>())), '/', FmtHex(pagesize<N>()), " -> ", FmtPE(*pe));
  return false;
}

// must be defined after ptprefix<0> specialization
inline paddr Paging::bootstrap(vaddr kernelEnd) {
  KASSERT1(kernelEnd <= kernelBase + MAXKERNSIZE, kernelEnd);

  static buf_t bootMemPT[bootSizePT]        __aligned(pagesize<pagetablepl>());
  memset(bootMemPT, 0, bootSizePT);

  PageEntry* pml4  = (PageEntry*)(bootMemPT);
  PageEntry* pdpth = (PageEntry*)(bootMemPT + 1 * pagesize<pagetablepl>());
  PageEntry* pdh   = (PageEntry*)(bootMemPT + 2 * pagesize<pagetablepl>());
  PageEntry* pdptk = (PageEntry*)(bootMemPT + 3 * pagesize<pagetablepl>());
  PageEntry* pdk   = (PageEntry*)(bootMemPT + 4 * pagesize<pagetablepl>()); // must be last

  pml4[recptindex] = (paddr(pml4) - kernelBase) | KernelPT;

  pml4 [getIndex<4>(bootHeapMap)] = (paddr(pdpth) - kernelBase) | KernelPT;
  pdpth[getIndex<3>(bootHeapMap)] = (paddr(pdh)   - kernelBase) | KernelPT;

  pml4 [getIndex<4>(kernelBase)]  = (paddr(pdptk) - kernelBase) | KernelPT;
  for (paddr m = 0; m < kernelEnd - kernelBase; m += kernelps) {
    if (m % pagesize<kernelpl+1>() == 0) {
      size_t idx = getIndex<kernelpl+1>(kernelBase) + m / pagesize<kernelpl+1>();
      pdptk[idx] = (paddr(pdk) - kernelBase) | PageTable;
    }
    *pdk = m | PS() | KernelPT;
    pdk += 1;
    DBG::outl(DBG::Paging, "Paging(", FmtHex(pml4), ")/map<", kernelpl, ">: ", FmtHex(kernelBase + m), '/', FmtHex(pagesize<kernelpl>()), " -> ", FmtHex(m), " PE:", FmtHex(getEntry<kernelpl>(kernelBase + m)));
  }

  installPagetable(paddr(pml4) - kernelBase);
  return paddr(pml4) - kernelBase;
}

// must be defined after ptprefix<0> specialization
inline void Paging::bootstrap2(FrameManager& fm) {
  for (vaddr vma = kernelbot; vma < kerneltop; vma += pagesize<pagelevels>()) {
    bool check = mapTable<pagelevels-1>(vma, fm);
    KASSERT1(check, vma);
  }
}

// must be defined after ptprefix<0> specialization
inline paddr Paging::cloneKernelPT(FrameManager& fm) {
  paddr newpt = fm.allocFrame<pagetablepl>();
  bool check = map<pagetablepl,true>(cloneAddr, newpt, Data, fm);
  KASSERT0(check);
  memset(ptr_t(cloneAddr), 0, pagesize<pagetablepl>());	// TODO: alloczero
  PageEntry* clonedPE = (PageEntry*)cloneAddr;
  clonedPE[recptindex] = newpt | KernelPT;
  PageEntry* kernelPE = (PageEntry*)ptprefix<4>();
  for (size_t idx = kernbindex; idx < pagetableentries; idx += 1) {
    clonedPE[idx] = kernelPE[idx];
  }
  unmap<pagetablepl>(cloneAddr);
  CPU::InvTLB(cloneAddr);              // must invalidate TLB entry explicitly!
  return newpt;
}

#endif /* _Paging_h_ */
