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
#ifndef _bimanip_h_
#define _bimanip_h_ 1

#include "generic/basics.h"

template <typename T>
static inline constexpr size_t bitsize() {
  return sizeof(T) * charbits;
}

template <typename T>
static inline constexpr T bitmask(unsigned int Width) {
  return Width == bitsize<T>() ? limit<T>() : pow2<T>(Width) - 1;
}

template <typename T>
static inline constexpr T bitmask(unsigned int Pos, unsigned int Width) {
  return bitmask<T>(Width) << Pos;
}

#define __kos_builtin(x, func) \
  ( sizeof(x) <= sizeof(int) ? __builtin_ ## func(x) : \
    sizeof(x) <= sizeof(long) ? __builtin_ ## func ## l(x) : \
    sizeof(x) <= sizeof(long long) ? __builtin_ ## func ## ll(x) : \
    sizeof(x) * charbits )

template <typename T>
static inline constexpr int lsbcond(T x, T alt = bitsize<T>()) {
  return x == 0 ? alt : __kos_builtin(x, ctz);
}

template <typename T>
static inline constexpr int msbcond(T x, T alt = bitsize<T>()) {
  return x == 0 ? alt : (bitsize<T>() - 1) - __kos_builtin(x, clz);
}

template <typename T>
static inline constexpr int lsb(T x) {
  return __kos_builtin(x, ctz);
}

template <typename T>
static inline constexpr int msb(T x) {
  return (bitsize<T>() - 1) - __kos_builtin(x, clz);
}

template <typename T>
static inline constexpr int popcount(T x) {
  return __kos_builtin(x, popcount);
}

template <typename T>
static inline constexpr int floorlog2( T x ) {
  return msbcond(x, bitsize<T>());
}

template <typename T>
static inline constexpr int ceilinglog2( T x ) {
  return msbcond(x - 1, limit<T>()) + 1; // x = 1 -> msb = -1 (alt) -> result is 0
}

template <typename T>
static inline constexpr int alignment( T x ) {
  return lsbcond(x, bitsize<T>());
}

template <typename T, unsigned int Pos, unsigned int Width>
class BitString {
  static_assert( Pos + Width <= 8*sizeof(T), "illegal parameters" );
public:
  constexpr T operator()() const { return bitmask<T>(Pos,Width); }
  constexpr T put(T f) const { return (f & bitmask<T>(Width)) << Pos; }
  constexpr T get(T f) const { return (f >> Pos) & bitmask<T>(Width); }
#if defined(__clang__)
  BitString() {}
#endif
};

template<bool atomic=false>
static inline void bit_set(mword& a, mword idx) {
  mword b = mword(1) << idx;
  if (atomic) __atomic_or_fetch(&a, b, __ATOMIC_RELAXED);
  else a |= b;
}

template<bool atomic=false>
static inline void bit_clr(mword& a, mword idx) {
  mword b = ~(mword(1) << idx);
  if (atomic) __atomic_and_fetch(&a, b, __ATOMIC_RELAXED);
  else a &= b;
}

template<bool atomic=false>
static inline void bit_flp(mword& a, mword idx) {
  mword b = mword(1) << idx;
  if (atomic) __atomic_xor_fetch(&a, b, __ATOMIC_RELAXED);
  else a ^= b;
}

#if defined(__x86_64__)

// loop is unrolled at -O3
// "=&r"(scan) to mark as 'earlyclobber': modified before all input processed
// "+r"(newmask) to keep newmask = mask
template<size_t N, bool findset = true>
static inline mword multiscan(const mword* data) {
  mword result = 0;
  mword mask = ~mword(0);
  mword newmask = mask;
  for (size_t i = 0; i < N; i++) {
    mword scan;
    asm volatile("\
      bsfq %2, %0\n\t\
      cmovzq %3, %0\n\t\
      cmovnzq %4, %1"
    : "=&r"(scan), "+r"(newmask)
    : "rm"(findset ? data[i] : ~data[i]), "r"(bitsize<mword>()), "r"(mword(0))
    : "cc");
    result += scan & mask;
    mask = newmask;
  }
  return result;
}

// loop is unrolled at -O3
// "=&r"(scan) to mark as 'earlyclobber': modified before all input processed
// "+r"(newmask) to keep newmask = mask
template<size_t N, bool findset = true>
static inline mword multiscan_r(const mword* data) {
  mword result = 0;
  mword mask = ~mword(0);
  mword newmask = mask;
  size_t i = N;
  do {
    i -= 1;
    mword scan;
    asm volatile("\
      bsrq %2, %0\n\t\
      cmovzq %3, %0\n\t\
      cmovnzq %4, %1"
    : "=&r"(scan), "+r"(newmask)
    : "rm"(findset ? data[i] : ~data[i]), "r"(mword(0)), "r"(mword(0))
    : "cc");
    result += (scan & mask) + (bitsize<mword>() & ~mask);
    mask = newmask;
  } while (i != 0);
  return result;
}

#else
#error unsupported architecture: only __x86_64__ supported at this time
#endif

#endif /* _bimanip_h_ */
