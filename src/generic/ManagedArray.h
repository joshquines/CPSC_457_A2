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
#ifndef _ManagedArray_h_
#define _ManagedArray_h_ 1

#include "generic/basics.h"

#include <vector>

// ManagedArray automatically keeps track of which fields are in use and
// which are unused.  Unused fields are stored in a free-stack, so can be
// found at small constant cost.
template<typename T, template<typename> class Alloc>
class ManagedArray {
  struct Free {
    Free* next;
  };
  static_assert(sizeof(T) >= sizeof(Free), "free stack condition");
  Alloc<T> allocator;
  T* data;
  Free* freestack;
  size_t index;
  size_t capacity;
  size_t freecount;
  vector<bool,Alloc<bool>> checkbits;

protected:
  void internalSet(size_t idx, const T& elem) {
    checkbits[idx] = true;
    data[idx] = elem;
  }

  void internalRemove(size_t idx) {
    Free* tmp = (Free*)&data[idx];
    tmp->next = freestack;
    freestack = tmp;
    freecount += 1;
  }

public:
  ManagedArray() : data(nullptr), freestack(nullptr),
    index(0), capacity(0), freecount(0), checkbits(0,false) {}
  ManagedArray(size_t n) : data(allocator.allocate(n)), freestack(nullptr),
    index(0), capacity(n), freecount(0), checkbits(n,false) {}
  ~ManagedArray() { allocator.deallocate(data, capacity); }

  bool   valid(size_t idx) const { return idx < index &&  checkbits[idx]; }
  bool   clear(size_t idx) const { return idx < index && !checkbits[idx]; }
  size_t size()            const { return index - freecount; }
  size_t currentIndex()    const { return index; }
  size_t currentCapacity() const { return capacity; }
  bool   empty()           const { return size() == 0; }

  size_t reserveIndex() {
    if (freestack) {
      size_t idx = (T*)freestack - data;
      freestack = freestack->next;
      freecount -= 1;
      GENASSERT1(!checkbits[idx], idx);
      return idx;
    } else if (capacity > index) {
      size_t idx = index;
      index += 1;
      GENASSERT1(!checkbits[idx], idx);
      return idx;
    } else {
      size_t newCapacity = capacity ? capacity * 2 : 1;
      checkbits.resize(newCapacity, false);
      T* newData = allocator.allocate(newCapacity);
      if (!newData) return limit<size_t>();
      for (size_t i = 0; i < capacity; i += 1) newData[i] = data[i];
      allocator.deallocate(data, capacity);
      data = newData;
      capacity = newCapacity;
      return reserveIndex();
    }
  }

  void set(size_t idx, const T& elem) {
    GENASSERT1(clear(idx), idx);
    internalSet(idx, elem);
  }

  size_t put(const T& elem) {
    size_t idx = reserveIndex();
    internalSet(idx, elem);
    return idx;
  }

  T& get(size_t idx) const {
    GENASSERT1(valid(idx), idx);
    return data[idx];
  }

  void invalidate(size_t idx) {
    GENASSERT1(valid(idx), idx);
    checkbits[idx] = false;
  }

  void release(size_t idx) {
    GENASSERT1(clear(idx), idx);
    internalRemove(idx);
  }

  void remove(size_t idx) {
    invalidate(idx);
    internalRemove(idx);
  }

  bool get(size_t idx, T& elem) {
    if (!valid(idx)) return false;
    elem = data[idx];
    internalRemove(idx);
    return true;
  }

};

#endif /* _ManagedArray_h_ */
