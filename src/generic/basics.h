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
#ifndef _basics_h_
#define _basics_h_ 1

#define fastpath(x)  (__builtin_expect((bool(x)),true))
#define slowpath(x)  (__builtin_expect((bool(x)),false))

#define __section(x) __attribute__((__section__(x)))
#define __aligned(x) __attribute__((__aligned__(x)))
#define __packed     __attribute__((__packed__))

#define __finline    __attribute__((__always_inline__))
#define __ninline    __attribute__((__noinline__))
#define __noreturn   __attribute__((__noreturn__))
#define __useresult  __attribute__((__warn_unused_result__))

typedef void* ptr_t;
typedef const void* cptr_t;

#if defined(__KOS__)
#include "kostypes.h"
#else
#error undefined platform: only __KOS__ supported at this time
#endif

#include <iostream>
#include <iomanip>

using namespace std;

struct FmtHex {
  mword val;
  int digits;
  FmtHex(mword p,       int d = 0) : val(mword(p)), digits(d) {}
  FmtHex(ptr_t p,       int d = 0) : val(mword(p)), digits(d) {}
  FmtHex(const char* p, int d = 0) : val(mword(p)), digits(d) {}
};

inline ostream& operator<<(ostream &os, const FmtHex& h) {
  os << "0x" << hex << uppercase << setw(h.digits) << setfill('0') << h.val << dec;
  return os;
}

#if defined(__KOS__)
#include "kernel/OutputBasic.h"
#define GENASSERT0  KASSERT0
#define GENASSERT1  KASSERT1
#define GENASSERTN  KASSERTN
#define GENABORT0   KABORT0
#define GENABORT1   KABORT1
#else
#error undefined platform: only __KOS__ supported at this time
#endif

static inline void unreachable() __noreturn;
static inline void unreachable() {
  __builtin_unreachable();
  __builtin_trap();
}

class NoObject {
  NoObject() = delete;                                 // no creation
  NoObject(const NoObject&) = delete;                  // no copy
  const NoObject& operator=(const NoObject&) = delete; // no assignment
};

typedef void (*funcvoid0_t)();
typedef void (*funcvoid1_t)(ptr_t);
typedef void (*funcvoid2_t)(ptr_t, ptr_t);
typedef void (*funcvoid3_t)(ptr_t, ptr_t, ptr_t);

template <typename T>
static inline constexpr T pow2( unsigned int x ) {
  return T(1) << x;
}

template <typename T>
static inline constexpr bool ispow2( T x ) {
  return (x & (x - 1)) == 0;
}

template <typename T>
static inline constexpr T align_up( T x, T a ) {
  return (x + a - 1) & (~(a - 1));
}

template <typename T>
static inline constexpr T align_down( T x, T a ) {
  return x & (~(a - 1));
}

template <typename T>
static inline constexpr bool aligned( T x, T a ) {
  return (x & (a - 1)) == 0;
}

template <typename T>
static inline constexpr T divup( T n, T d ) {
  return ((n - 1) / d) + 1;
}

template <typename T>
static inline constexpr T limit() {
  return ~T(0);
}

#endif /* _basics_h_ */
