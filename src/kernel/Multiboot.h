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
#ifndef _Multiboot_h_
#define _Multiboot_h_ 1

#include "generic/basics.h"
#include "generic/RegionSet.h"

class Multiboot {
  static vaddr mbiStart                               __section(".boot.data");
  static vaddr mbiEnd                                 __section(".boot.data");
  static void initDebug(bool msg)                     __section(".boot.text");
public:
  static vaddr init(mword magic, vaddr mbi)           __section(".boot.text");
  static void init2()                                 __section(".boot.text");
  static void remap(vaddr disp)                       __section(".boot.text");
  static vaddr getRSDP()                              __section(".boot.text");
  static void getMemory(RegionSet<Region<paddr>>& rs) __section(".boot.text");
  static void readModules(vaddr disp)                 __section(".boot.text");
};

#endif /* _Multiboot_h_ */
