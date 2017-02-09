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
#include "kernel/FrameManager.h"

paddr FrameManager::allocContig(size_t& size, paddr align, paddr limit) {
  size = align_up(size, sps);
  if (size < dps) { // search in small frames first
    ScopedLock<> sl1(splock);
    for (auto it = smallFrames.begin(); it != smallFrames.end(); ++it) {
      size_t idx = it->second.findset();
      mword found = 0;
      size_t baseidx;
      paddr baseaddr;
      for (;;) { 
        if (found) {
          found += sps;
        } else {
          baseidx = idx;
          baseaddr = it->first * dps + baseidx * sps;
    if (baseaddr + size > limit) goto use_large_frame;
          if (aligned(baseaddr, align)) found = sps;
        }
        if (found >= size) {
          for (size_t i = baseidx; i <= idx; i += 1) it->second.clear(i);
          if (it->second.empty()) smallFrames.erase(it);
          return baseaddr;
        }
        for (;;) { // keeps found while consecutive bits are set
          idx += 1;
      if (idx >= pagetableentries) goto next_small_set;
        if (it->second.test(idx)) break;
          found = 0;
        }
      }
next_small_set:;
    }
use_large_frame:
    ScopedLock<> sl2(lplock);
    size_t idx = largeFrames.findset();
    if (idx * dps + size > limit) return topaddr;
    largeFrames.clear(idx);
    auto it = smallFrames.emplace(idx, Bitmap<pagetableentries>::filled()).first;
    for (size_t i = 0; i < size/sps; i += 1) it->second.clear(i);
    KASSERT0(!it->second.empty());
    return idx * dps;
  } else { // search in large frames by index, same as above
    ScopedLock<> sl(lplock);
    size = align_up(size, dps);
    size_t idx = largeFrames.findset();
    mword found = 0;
    size_t baseidx;
    paddr baseaddr;
    for (;;) {
      if (found) {
        found += dps;
      } else {
        baseidx = idx;
        baseaddr = idx * dps;
    if (baseaddr + size > limit) return topaddr;
        if (aligned(baseaddr, align)) found = dps;
      }
      if (found >= size) {
        for (size_t i = baseidx; i <= idx; i += 1) largeFrames.clear(i);
        return baseaddr;
      }
      for (;;) { // keeps found while consecutive bits are set
        idx += 1;
    if (idx >= bitcount) return topaddr;
      if (largeFrames.test(idx)) break;
        found = 0;
      }
    }
  }
  KABORT0();
  unreachable();
}

ostream& operator<<(ostream& os, const FrameManager& fm) {
  size_t bc = fm.bitcount;
  size_t start = fm.largeFrames.findset();
  size_t end = start;
  const_cast<FrameManager&>(fm).lplock.acquire();
  for (;;) {
    if (start >= bc) break;
    end = fm.largeFrames.getrange(start, bc);
    os << ' ' << FmtHex(start*fm.dps) << '-' << FmtHex(end*fm.dps);
    if (end >= bc) break;
    start = fm.largeFrames.getrange(end, bc);
  }
  const_cast<FrameManager&>(fm).lplock.release();
  start = topaddr;
  ScopedLock<> sl(const_cast<FrameManager&>(fm).splock);
  for (	auto it = fm.smallFrames.begin(); it != fm.smallFrames.end(); ++it ) {
    if (start != topaddr && it->first * fm.dps != end) {
      os << ' ' << FmtHex(start) << '-' << FmtHex(end);
      start = topaddr;
    }
    for (size_t idx = 0; idx < pagetableentries; idx += 1) {
      if (it->second.test(idx)) {
        if (start == topaddr) end = start = it->first * fm.dps + idx * pagesize<1>();
        end += pagesize<1>();
      } else if (start != topaddr) {
        os << ',' << FmtHex(start) << '-' << FmtHex(end);
        start = topaddr;
      }
    }
  }
  return os;
}
