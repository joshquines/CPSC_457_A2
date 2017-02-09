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
#ifndef _Clock_h_
#define _Clock_h_ 1

#include "machine/CPU.h"

class Clock : public NoObject {
  static volatile mword tick;
public:
  static void ticker() { tick += 1; }
  static mword now() { return tick; }
  static void wait(mword ticks) {
    mword start = tick;
    while (tick < start + ticks) CPU::Pause();
  }
};

#endif /* _Clock_h_ */
