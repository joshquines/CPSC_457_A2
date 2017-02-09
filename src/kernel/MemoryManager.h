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
#ifndef _MemoryManager_h_
#define _MemoryManager_h_ 1

#include "machine/Memory.h"
#include "machine/SpinLock.h"

extern "C" void free(void* p);
extern "C" void* malloc(size_t);

struct BlockStore {
  struct Free { Free* next; };
  SpinLock lock;
  Free* freestack;
public:
  BlockStore() : freestack(nullptr) {}
  bool empty() { return freestack == nullptr; }
  template<size_t S> void fill(vaddr p, size_t s) {
    static_assert(S >= sizeof(Free), "S to small");
    while (s >= S) {
      ((Free*)p)->next = freestack;
      freestack = (Free*)p;
      p += S;
      s -= S;
    }
  }
  vaddr alloc() {
    KASSERT0(freestack);
    vaddr ret = (vaddr)freestack;
    freestack = freestack->next;
    return ret;
  }
  void release(vaddr p) {
    ((Free*)p)->next = freestack;
    freestack = (Free*)p;
  }
};

class PageStore : public BlockStore {
public:
  static const size_t psize = pagesize<1>();
  inline void check();
};

/* MemoryManager is where complex memory algorithms could be implemented */
class MemoryManager : public NoObject {
  friend void free(void*);
  friend void* malloc(size_t);
  static SpinLock mallocLock;
  static void* mallocSpace;
  static ptr_t legacy_malloc(size_t s);
  static void legacy_free(ptr_t p);

  static const size_t botidx = 6;
  static const size_t topidx = 7;
  static BlockStore blockStore[1 + topidx - botidx];
  static PageStore pageStore;

public:
  static void init0( vaddr p, size_t s );
  static void reinit( vaddr p, size_t s );

  static vaddr alloc( size_t s ) { return (vaddr)legacy_malloc(s); }
  static void release( vaddr p, size_t s ) { legacy_free((ptr_t)p); }
  static vaddr map(size_t s, paddr pma = 0);
  static void unmap(vaddr v, size_t s, bool alloc = true);
  static vaddr allocContig(size_t& size, paddr align, paddr limit);

  template<typename T> static T* alloc2() {
    const size_t tidx = ceilinglog2(sizeof(T));
    static_assert(tidx >= botidx && tidx <= topidx, "alloc2: type size outside range");
    const size_t idx = tidx - botidx;
    ScopedLock<> sl1(blockStore[idx].lock);
    if (blockStore[idx].empty()) {
      ScopedLock<> sl2(pageStore.lock);
      pageStore.check();
      blockStore[idx].fill<pow2<size_t>(tidx)>(pageStore.alloc(), PageStore::psize);
    }
    return (T*)blockStore[idx].alloc();
  }

  template<typename T> static void release2(T* p) {
    const size_t tidx = ceilinglog2(sizeof(T));
    static_assert(tidx >= botidx && tidx <= topidx, "release2: type size outside range");
    const size_t idx = tidx - botidx;
    ScopedLock<> sl1(blockStore[idx].lock);
    blockStore[idx].release(vaddr(p));
  }
};

inline void PageStore::check() {
  if (empty()) {
    vaddr mem = MemoryManager::map(psize);
    freestack = (Free*)mem;
    freestack->next = nullptr;
  }
}

template<typename T>
T* kmalloc(size_t n = 1) {
  return (T*)MemoryManager::alloc(n * sizeof(T));
}

template<typename T>
void kfree(T* p, size_t n = 1) {
  MemoryManager::release((vaddr)p, n * sizeof(T));
}

template<typename T, typename... Args>
T* knew(Args&&... a) {
  return new (kmalloc<T>()) T(std::forward<Args>(a)...);
}

template<typename T>
T* knewN(size_t n) {
  return new (kmalloc<T>(n)) T[n];
}

template<typename T>
void kdelete(T* p, size_t n = 1) {
  for (size_t i = 0; i < n; i += 1) p[i].~T();
  kfree<T>(p, n);
}

template<typename T>
T* kmalloc2() {
  return (T*)MemoryManager::alloc2<T>();
}

template<typename T>
void kfree2(T* p) {
  MemoryManager::release2<T>(p);
}

template<typename T, typename... Args>
T* knew2(Args&&... a) {
  return new (MemoryManager::alloc2<T>()) T(std::forward<Args>(a)...);
}

template<typename T>
void kdelete2(T* p) {
  p->~T();
  MemoryManager::release2<T>(p);
}

template<typename T> class KernelAllocator : public allocator<T> {
public:
  template<typename U> struct rebind { typedef KernelAllocator<U> other; };
  KernelAllocator() = default;
  KernelAllocator(const KernelAllocator& x) = default;
  template<typename U> KernelAllocator (const KernelAllocator<U>& x) : allocator<T>(x) {}
  ~KernelAllocator() = default;
  T* allocate(size_t n, const void* = 0) { return kmalloc<T>(n); }
  void deallocate(T* p, size_t s) { kfree<T>(p, s); }
};

#endif /* _MemoryManager_h_ */
