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
#include "machine/APIC.h"
#include "machine/Machine.h"
#include "devices/RTC.h"

void RTC::init() { // see http://wiki.osdev.org/RTC
  Machine::registerIrqSync(PIC::RTC, 0xf8);

  CPU::out8(0x70, CPU::in8(0x70) | 0x80); // disable NMI

  CPU::out8(0x70, 0x0A);             // select Status Register A
  uint8_t prev = CPU::in8(0x71);     // read current value
  CPU::out8(0x70, 0x0A);             // select Status Register A
  CPU::out8(0x71, prev | 0x06);      // set rate to 32768 / (2^(6-1)) = 1024 Hz

  CPU::out8(0x70, 0x0B);             // select Status Register B
  prev = CPU::in8(0x71);             // read current value
  CPU::out8(0x70, 0x0B);             // select Status Register B
  CPU::out8(0x71, prev | 0x40);      // enable RTC with periodic firing

  CPU::out8(0x70, CPU::in8(0x70) & 0x7F); // enable NMI

  staticInterruptHandler();     // read RTC once -> needed to get things going
}
