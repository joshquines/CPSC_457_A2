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
#ifndef _Buffers_h_
#define _Buffers_h_ 1

#include "generic/basics.h"

#include <array>
#include <queue>

template<typename Element, size_t N>
class FixedArray : public array<Element,N> {
public:
  typedef Element ElementType;
  explicit FixedArray( size_t ) {}
};

template<typename Element, typename Allocator>
class RuntimeArray {
  Allocator alloc;
  Element* buffer;
  size_t max;
public:
  typedef Element ElementType;
  explicit RuntimeArray( size_t N, Element* ptr = nullptr )
    : alloc(), buffer(alloc.allocate(N)), max(N) { GENASSERT0(N); }
  ~RuntimeArray() { alloc.deallocate(buffer, max); }
  size_t max_size() const { return max; }
  Element& operator[] (size_t i) { return buffer[i]; }
  const Element& operator[] (size_t i) const { return buffer[i]; }
};

template<typename Array>
class RingBuffer {
  size_t next;
  size_t count;
  Array array;
public:
  typedef typename Array::ElementType Element;
  explicit RingBuffer( size_t N = 0 )
    : next(0), count(0), array(N) {}
  size_t size() const { return count; }
  size_t max_size() const { return array.max_size(); }
  bool empty() const { return size() == 0; }
  bool full() const { return size() == max_size(); }
  Element& front() {
    GENASSERT0(!empty());
    return array[next];
  }
  const Element& front() const {
    GENASSERT0(!empty());
    return array[next];
  }
  Element& back() {
    GENASSERT0(!empty());
    return array[(next + count - 1) % max_size()];
  }
  const Element& back() const {
    GENASSERT0(!empty());
    return array[(next + count - 1) % max_size()];
  }
  void push( const Element& x ) {
    GENASSERT0(!full());
    array[(next + count) % max_size()] = x;
    count += 1;
  }
  void pop() {
    GENASSERT0(!empty());
    next = (next + 1) % max_size();
    count -= 1;
  }
};

template<typename Element, size_t N>
class FixedRingBuffer : public RingBuffer<FixedArray<Element,N>> {
public:
  explicit FixedRingBuffer(size_t) {}
};

template<typename Element, typename Allocator>
class RuntimeRingBuffer : public RingBuffer<RuntimeArray<Element,Allocator>> {
public:
  explicit RuntimeRingBuffer(size_t N) : RingBuffer<RuntimeArray<Element,Allocator>>(N) {}
};

template<typename Element, typename Allocator>
class QueueBuffer : public queue<Element,deque<Element,Allocator>> {
  using baseclass = queue<Element,deque<Element,Allocator>>;
  size_t max;
public:
  QueueBuffer( size_t N ) : max(N) { GENASSERT0(N); }
  bool full() const { return baseclass::size() == max; }
  size_t max_size() const { return max; }
};

#endif /* _Buffers_h_ */
