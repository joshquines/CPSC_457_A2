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
#ifndef _Serial_h_
#define _Serial_h_ 1

#include "machine/CPU.h"

class DebugDevice : public NoObject {
  friend class Machine;
  friend class SerialDevice;
  static bool valid;
  static void init() { valid = (CPU::in8(0xE9) == 0xE9); }
public:
  static inline void write(char c);
};

static const int maxSerial = 2;

static const uint16_t SerialPort[maxSerial] = { 0x3F8, 0x2F8 };

class SerialDevice : public NoObject { // see http://wiki.osdev.org/Serial_Ports
  friend class DebugDevice;
  friend class Machine;
  static bool gdb;

  static void init(bool g) {
    gdb = g;
    for (int i = 0; i < maxSerial; i += 1) {
      CPU::out8(SerialPort[i] + 1, 0x00);    // Disable all interrupts
      CPU::out8(SerialPort[i] + 3, 0x80);    // Enable DLAB (set baud rate divisor)
      CPU::out8(SerialPort[i] + 0, 0x01);    // Set divisor to 1 (lo byte) 115200 baud
      CPU::out8(SerialPort[i] + 1, 0x00);    //                  (hi byte)
      CPU::out8(SerialPort[i] + 3, 0x03);    // 8 bits, no parity, one stop bit
      CPU::out8(SerialPort[i] + 2, 0xC7);    // Enable FIFO, clear them, with 14-byte threshold
//    	CPU::out8(SerialPort[i] + 4, 0x0B);    // IRQs enabled, RTS/DSR set
    }
  }
public:
  static void write(char c, uint16_t idx = 0) {
    while ((CPU::in8(SerialPort[idx] + 5) & 0x20) == 0);
    CPU::out8(SerialPort[idx], c);
  }
  static char read(uint16_t idx = 0) {
    while ((CPU::in8(SerialPort[idx] + 5) & 1) == 0);
    return CPU::in8(SerialPort[idx]);
  }
  static void dbgwrite(char c) {
    for (uint16_t idx = gdb ? 1 : 0; idx < maxSerial; idx += 1) {
      write(c, idx);
      if (c == '\n') write('\r', idx);
    }
  }
};

void DebugDevice::write(char c) {
  if (valid) CPU::out8(0xE9, c);
  SerialDevice::dbgwrite(c);
}

#endif /* _Serial_h_ */
