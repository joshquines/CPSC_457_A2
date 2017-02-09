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
#ifndef _RTC_h_
#define _RTC_h_ 1

#include "machine/CPU.h"

class RTC {
  volatile mword currentTick;
public:
  RTC() : currentTick(0) {}
  void init()                                          __section(".boot.text");
  void staticInterruptHandler() {
    CPU::out8(0x70,0x0C);   // select Status Register C
    CPU::in8(0x71);         // read RTC -> needed to keep interrupts coming
    currentTick += 1;
  }
  void wait(mword ticks) {
    mword start = currentTick;
    while (currentTick < start + ticks) CPU::Pause();
  }
  mword tick() {
    return currentTick;
  }
};

#endif /* _RTC_h_ */
