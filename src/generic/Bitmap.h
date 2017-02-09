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
#ifndef _Bitmap_h_
#define _Bitmap_h_ 1

#include "generic/bitmanip.h"

template<size_t X> constexpr mword BitmapEmptyHelper(const mword* bits) {
  return bits[X] | BitmapEmptyHelper<X-1>(bits);
}

template<> constexpr mword BitmapEmptyHelper<0>(const mword* bits) {
  return bits[0];
}

template<size_t X> constexpr mword BitmapFullHelper(const mword* bits) {
  return bits[X] & BitmapFullHelper<X-1>(bits);
}

template<> constexpr mword BitmapFullHelper<0>(const mword* bits) {
  return bits[0];
}

template<size_t X> constexpr mword BitmapCountHelper(const mword* bits) {
  return popcount(bits[X]) + BitmapCountHelper<X-1>(bits);
}

template<> constexpr mword BitmapCountHelper<0>(const mword* bits) {
  return popcount(bits[0]);
}

template<size_t B = bitsize<mword>()>
class Bitmap {
  static const int N = divup(B,bitsize<mword>());
  mword bits[N];
public:
  explicit Bitmap( mword b = 0 ) { for (size_t i = 0; i < N; i += 1) bits[i] = b; }
  static constexpr bool valid( mword idx ) { return idx < N * bitsize<mword>(); }
  static Bitmap filled() { return Bitmap(~mword(0)); }
  template<bool atomic=false> void set  ( mword idx ) {
    bit_set<atomic>(bits[idx / bitsize<mword>()], idx % bitsize<mword>());
  }
  template<bool atomic=false> void clear( mword idx ) {
    bit_clr<atomic>(bits[idx / bitsize<mword>()], idx % bitsize<mword>());
  }
  template<bool atomic=false> void flip ( mword idx ) {
    bit_flp<atomic>(bits[idx / bitsize<mword>()], idx % bitsize<mword>());
  }
  constexpr bool test( mword idx ) const {
    return bits[idx / bitsize<mword>()] & (mword(1) << (idx % bitsize<mword>()));
  }
  constexpr bool empty() const { return BitmapEmptyHelper<N-1>(bits) == mword(0); }
  constexpr bool full() const { return BitmapFullHelper<N-1>(bits) == ~mword(0); }
  constexpr mword count() const { return BitmapCountHelper<N-1>(bits); }
  constexpr mword findset() const { return multiscan<N,true>(bits); }
  constexpr mword findset_rev() const { return multiscan_r<N,true>(bits); }
  constexpr mword findclear() const { return multiscan<N,false>(bits); }
};

template<>
class Bitmap<bitsize<mword>()> {
  mword bits;
public:
  static constexpr bool valid(mword idx) { return idx < bitsize<mword>(); }
  constexpr explicit Bitmap( mword b = 0 ) : bits(b) {}
  template<bool atomic=false> void set  ( mword idx ) {
    bit_set<atomic>(bits, idx);
  }
  template<bool atomic=false> void clear( mword idx ) {
    bit_clr<atomic>(bits, idx);
  }
  template<bool atomic=false> void flip ( mword idx ) {
    bit_flp<atomic>(bits, idx);
  }
  constexpr bool test( mword idx ) const {
    return bits & (mword(1) << idx);
  }
  constexpr bool empty() const { return bits == mword(0); }
  constexpr bool full() const { return bits == ~mword(0); }
  constexpr mword count() const { return popcount(bits); }
  mword findset() const { return lsbcond(bits); }
  mword findset_rev() const { return msbcond(bits); }
  mword findclear() const { return lsbcond(~bits); }
  mword findnextset( mword idx = 0 ) const { return lsbcond( bits & ~bitmask<mword>(idx) ); }
};

// N: number of machine words in elementary bitmap (set to cacheline size)
// W: log2 of maximum width of bitmap
template<size_t B, size_t W>
class HierarchicalBitmap {
  static_assert( B >= bitsize<mword>(), "template parameter B less than word size");
  static const size_t N = divup(B, bitsize<mword>());
  static const size_t logB = floorlog2(B);
  static_assert( B == pow2<size_t>(logB), "template parameter B not a power of 2" );
  static_assert( W >= logB, "template parameter W smaller than log B" );
  Bitmap<B>* bitmaps[divup(W,logB)];
  size_t toplevel;
  static constexpr size_t size( size_t bitcount ) {
    return bitcount < B ? B : divup(bitcount,B) + size(divup(bitcount,B));
  }
public:
  HierarchicalBitmap() : toplevel(0) {
    for (size_t i = 0; i < divup(W,logB); i += 1) bitmaps[i] = nullptr;
  }
  static constexpr size_t allocsize( size_t bitcount ) {
    return sizeof(Bitmap<B>) * size(bitcount);
  }
  void init( size_t bitcount, bufptr_t p ) {
    bitmaps[0] = new (p) Bitmap<B>[size(bitcount)];
    do {
      toplevel += 1;
      bitcount = divup(bitcount,B);
      bitmaps[toplevel] = bitmaps[toplevel-1] + bitcount;
    } while (bitcount > 1);
  }
  void clone( size_t bitcount, bufptr_t p ) {
    bitmaps[0] = (Bitmap<B>*)p;
    do {
      toplevel += 1;
      bitcount = divup(bitcount,B);
      bitmaps[toplevel] = bitmaps[toplevel-1] + bitcount;
    } while (bitcount > 1);
  }
  void set( size_t idx ) {
    for (size_t i = 0; i <= toplevel; i += 1) {
      bool done = !bitmaps[i][idx / B].empty();
      bitmaps[i][idx / B].set(idx % B);
      if slowpath(done) return;
      idx = idx / B;
    }
  }
  void clear( size_t idx ) {
    for (size_t i = 0; i <= toplevel; i += 1) {
      bitmaps[i][idx / B].clear(idx % B);
      if slowpath(!bitmaps[i][idx / B].empty()) return;
      idx = idx / B;
    }
  }
  constexpr bool test( size_t idx ) const {
    return bitmaps[0][idx / B].test(idx % B);
  }
  constexpr bool empty() const {
    return bitmaps[toplevel][0].empty();
  }
  size_t findset() const {
    size_t idx = 0;
    for (size_t i = toplevel;; i -= 1) {
      size_t ldx = bitmaps[i][idx].findset();
      if slowpath(ldx == B) return limit<mword>();
      idx = idx * B + ldx;
      if slowpath(i == 0) return idx;
    }
  }
  size_t findset_rev() const {
    size_t idx = 0;
    for (size_t i = toplevel;; i -= 1) {
      size_t ldx = bitmaps[i][idx].findset_rev();
      if slowpath(ldx == B) return limit<mword>();
      idx = idx * B + ldx;
      if slowpath(i == 0) return idx;
    }
  }
  size_t getrange( size_t idx, size_t bitcount ) const { // used for printing
    bool isset = test(idx);
    do {
      idx += 1;
    } while (idx < bitcount && test(idx) == isset);
    return idx;
  }
};

#endif /* _Bitmap_h_ */
