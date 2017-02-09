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
#ifndef _FrameManager_h_
#define _FrameManager_h_ 1

#include "generic/Bitmap.h"
#include "kernel/MemoryManager.h"
#include "kernel/Output.h"

#include <map>

// TODO: store shared frames in separate data structure, including refcounter
class FrameManager {
  friend ostream& operator<<(ostream&, const FrameManager&);

  template<typename T> class MapAllocator : public allocator<T> {
  public:
    template<typename U> struct rebind { typedef MapAllocator<U> other; };
    MapAllocator() = default;
    MapAllocator(const MapAllocator& x) = default;
    template<typename U> MapAllocator (const MapAllocator<U>& x) : allocator<T>(x) {}
    ~MapAllocator() = default;
    T* allocate(size_t n, const void* = 0) { return kmalloc<T>(n); }
    void deallocate(T* p, size_t s) { kfree<T>(p, s); }
  };

  static const size_t dpl = kernelpl;
  static const size_t spl = kernelpl - 1;
  static const size_t dps = pagesize<dpl>(); // same as kernelps
  static const size_t sps = pagesize<spl>();

  typedef HierarchicalBitmap<pagetableentries,framebits-pagesizebits<dpl>()> LFBitmap;
  typedef map<paddr,Bitmap<pagetableentries>,less<paddr>,MapAllocator<paddr>> SFBitmap;
  SpinLock lplock;      // large page lock
  LFBitmap largeFrames; // large page container
  SpinLock splock;      // small page lock
  SFBitmap smallFrames; // small page container

  size_t bitcount; // for output, see ostream operator

  size_t releaseSmall(paddr addr, size_t size = sps) {
    // find appropriate bitmap in smallFrames, or create empty bitmap
    size_t idx = addr / dps;
    auto it = smallFrames.lower_bound(idx);
    if (it == smallFrames.end() || it->first != idx) it = smallFrames.emplace_hint(it, idx, Bitmap<pagetableentries>());

    // set bits in bitmap, limited to this bitmap
    size_t start = (addr % dps) / sps;
    size_t end = min(pagetableentries, start + size / sps);
    for (size_t i = start; i < end; i += 1) it->second.set(i);

    // check for full bitmap; one small page bitmap represents one large page
    if (it->second.full()) {
      smallFrames.erase(it);
      ScopedLock<> sl(lplock);
      largeFrames.set(it->first);
    }

    return (end - start) * sps;
  }

public:
  static constexpr size_t getSize( paddr top ) {
    return LFBitmap::allocsize(divup(top, dps));
  }

  void init( bufptr_t p, paddr top ) {
    bitcount = divup(top, dps);
    largeFrames.init(bitcount, p);
  }

  template<size_t N>
  paddr allocFrame() {
    paddr addr;
    switch (N) {
    case spl: {
      ScopedLock<> sl(splock);
      if (smallFrames.empty()) {
        lplock.acquire();
        size_t idx = largeFrames.findset();
        if slowpath(idx == limit<mword>()) return topaddr;
        largeFrames.clear(idx);
        lplock.release();
        smallFrames.emplace(idx, Bitmap<pagetableentries>::filled());
      }
      auto it = smallFrames.begin();
      size_t idx2 = it->second.findset();
      it->second.clear(idx2);
      if (it->second.empty()) smallFrames.erase(it);
      addr = it->first * dps + idx2 * sps;
    } break;
    case dpl: {
      ScopedLock<> sl(lplock);
      size_t idx = largeFrames.findset();
      if slowpath(idx == limit<mword>()) return topaddr;
      largeFrames.clear(idx);
      addr = idx * dps;
    } break;
    default: KABORT1(N);
    }
    DBG::outl(DBG::Frame, "FM/alloc<", N, ">: ", FmtHex(addr));
    return addr;
  }

  template<size_t N>
  void releaseFrame( paddr addr ) {
    KASSERT1( aligned(addr, pagesize<N>()), addr );
    switch (N) {
    case spl: {
        ScopedLock<> sl(splock);
        releaseSmall(addr);
    } break;
    case dpl: {
       ScopedLock<> sl(lplock);
       largeFrames.set(addr / dps);
    } break;
    default: KABORT1(N);
    }
    DBG::outl(DBG::Frame, "FM/release<", N, ">: ", FmtHex(addr));
  }

  void releaseFrames( paddr addr, size_t size ) {
    if (size < dps) releaseFrame<spl>(addr);
    else releaseFrame<dpl>(addr);
  }

  void releaseRegion( paddr addr, size_t size ) { // only used during bootstrap
    KASSERT1( aligned(addr, sps), addr );
    KASSERT1( aligned(size, sps), size );
    DBG::outl(DBG::Frame, "FM/releaseRegion: ", FmtHex(addr), '/', FmtHex(size));
    while (size > 0) {
      size_t level_addr = 1 + (alignment(addr) - pageoffsetbits) / pagetablebits;
      size_t level_size = 1 + (floorlog2(size) - pageoffsetbits) / pagetablebits;
      size_t level = min(level_addr, level_size);
      size_t len;
      if (level < dpl) {
        ScopedLock<> sl(splock);
        len = releaseSmall(addr, size);
      } else {
        ScopedLock<> sl(lplock);
        largeFrames.set(addr / dps);
        len = dps;
      }
      addr += len;
      size -= len;
    }
  }

  paddr allocContig( size_t& size, paddr align, paddr limit );
};

#endif /* _FrameManager_h_ */
