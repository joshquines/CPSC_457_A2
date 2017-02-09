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
#include "kernel/AddressSpace.h"
#include "kernel/MemoryManager.h"
#include "kernel/Output.h"

#include "extern/dlmalloc/malloc_glue.h"
#include "extern/dlmalloc/malloc.h"

SpinLock MemoryManager::mallocLock;
void* MemoryManager::mallocSpace;

BlockStore MemoryManager::blockStore[1 + MemoryManager::topidx - MemoryManager::botidx];
PageStore MemoryManager::pageStore;

static_assert(DEFAULT_GRANULARITY == kernelps, "dlmalloc DEFAULT_GRANULARITY != kernelps");

void* dl_mmap(void* addr, size_t len, int, int, int, _off64_t) {
  KASSERT1( aligned(len, size_t(DEFAULT_GRANULARITY)), len);
  vaddr va = kernelSpace.kmap<kernelpl>(vaddr(addr), len);
  KASSERT0(va != topaddr);
  return (void*)va;
}

int dl_munmap(void* addr, size_t len) {
  KASSERT1( aligned(len, size_t(DEFAULT_GRANULARITY)), len);
  kernelSpace.unmap<kernelpl>(vaddr(addr), len);
  return 0;
}

void MemoryManager::init0( vaddr p, size_t s ) {
  mallocSpace = create_mspace_with_base((ptr_t)p, s, 0);
}

void MemoryManager::reinit( vaddr p, size_t s ) {
  destroy_mspace(mallocSpace);
  mallocSpace = create_mspace_with_base((ptr_t)p, s, 0);
}

ptr_t MemoryManager::legacy_malloc(size_t s) {
  ScopedLock<> sl(mallocLock);
  return mspace_malloc(mallocSpace, s);
}

void MemoryManager::legacy_free(ptr_t p) {
  ScopedLock<> sl(mallocLock);
  mspace_free(mallocSpace, p);
}

vaddr MemoryManager::map(size_t s, paddr pma) {
  if (s < kernelps) {
    if (!pma) return kernelSpace.kmap<1,true>(0, s);
    else return kernelSpace.kmap<1,false>(0, s, pma);
  } else {
    if (!pma) return kernelSpace.kmap<kernelpl,true>(0, s);
    else return kernelSpace.kmap<kernelpl,false>(0, s, pma);
  }
}

void MemoryManager::unmap(vaddr v, size_t s, bool alloc) {
  if (s < kernelps) {
    if (alloc) kernelSpace.unmap<1>(v, s);
    else kernelSpace.unmap<1,false>(v, s);
  } else {
    if (alloc) kernelSpace.unmap<kernelpl>(v, s);
    else kernelSpace.unmap<kernelpl,false>(v ,s);
  }
}

vaddr MemoryManager::allocContig(size_t& size, paddr align, paddr limit) {
  paddr pma = LocalProcessor::getFrameManager()->allocContig(size, align, limit);
  KASSERT0(pma != topaddr);
  return map(size, pma);
}
