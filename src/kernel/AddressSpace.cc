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

void AddressSpace::print(ostream& os) const {
  os << "AS(" << FmtHex(pagetable) << "):";
  vaddr start = kernel ? kernelbot : 0;
  size_t size = 0;
  while ((size = Paging::testfree(start))) start += size;
  for (;;) {
    vaddr end = start;
    while ((size = Paging::testused(end))) end += size;
    os << ' ' << FmtHex((start & canonTest) ? (start | canonPrefix) : start)
       << "-" << FmtHex((end   & canonTest) ? (end   | canonPrefix) : end);
    start = end;
    while ((size = Paging::testfree(start))) start += size;
    if (start >= (kernel ? pow2<size_t>(pagebits) : usertop)) return;
  }
}
