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
#ifndef _SynchronizedArray_h_
#define _SynchronizedArray_h_ 1

#include "generic/ManagedArray.h"
#include "runtime/BlockingSync.h"

class SynchronizedElement {
  template <typename, template<typename> class> friend class SynchronizedArray;
  Condition wait;
  mword count;
public:
  SynchronizedElement() : count(0) {}
};

// NOTE: T must be a pointer type, otherwise synchronization is not much help
template<typename T, template<typename> class Alloc>
class SynchronizedArray : protected ManagedArray<T,Alloc> {
  BasicLock lock;
  using Baseclass = ManagedArray<T,Alloc>;
public:
  SynchronizedArray(size_t n = 0) : Baseclass(n) {}
  size_t currentIndex() const { return Baseclass::currentIndex(); }
  size_t store(const T& elem) {
    lock.acquire();
    size_t idx = Baseclass::reserveIndex();
    lock.release();
    Baseclass::set(idx, elem);
    if (!elem) Baseclass::invalidate(idx);
    return idx;
  };
  T access(size_t idx) {
    AutoLock al(lock);
    if (!Baseclass::valid(idx)) return nullptr;
    T& elem = Baseclass::get(idx);
    GENASSERT0(elem);
    if (!elem->SynchronizedElement::wait.empty()) return nullptr;
    elem->SynchronizedElement::count += 1;
    return elem;
  }
  void done(size_t idx) {
    lock.acquire();
    GENASSERT1(Baseclass::valid(idx), idx);
    T& elem = Baseclass::get(idx);
    GENASSERT1(elem && elem->SynchronizedElement::count > 0, FmtHex(elem));
    elem->SynchronizedElement::count -= 1;
    if (elem->SynchronizedElement::count == 0 && !elem->SynchronizedElement::wait.empty()) {
      Baseclass::invalidate(idx);
      elem->SynchronizedElement::wait.signal(lock);
    } else {
      lock.release();
    }
  }
  T remove(size_t idx) {
    lock.acquire();
    if (!Baseclass::valid(idx)) { lock.release(); return nullptr; }
    T& elem = Baseclass::get(idx);
    GENASSERT0(elem);
    if (elem->SynchronizedElement::count > 0) {
      elem->SynchronizedElement::wait.wait(lock);
    } else {
      Baseclass::invalidate(idx);
      lock.release();
    }
    return elem;
  }
  void release(size_t idx) {
    AutoLock al(lock);
    Baseclass::release(idx);
  }
};

#endif /* _SynchronizedArray_h_ */
