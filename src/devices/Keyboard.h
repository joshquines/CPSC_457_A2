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
// details taken from
// http://wiki.osdev.org/%228042%22_PS/2_Controller
// http://www.brokenthorn.com/Resources/OSDev19.html
// http://www.brokenthorn.com/Resources/Demos/Demo15/SysCore/Keyboard/kybrd.cpp

#ifndef _Keyboard_h_
#define _Keyboard_h_ 1

#include "generic/Buffers.h"
#include "runtime/SyncQueues.h"

class Keyboard {
public:
  typedef int KeyCode;

private:
  MessageQueue<FixedRingBuffer<KeyCode,256>> kbq;

  // state machine variables
  bool is_break;                        // make or break code
  bool shift, alt, ctrl;                // shift, alt, and ctrl keys
  int  led;                             // led status byte
  int  acks;                            // number of expected acks
  int  extended;                        // extended key code

  inline void irqHandlerInternal();

public:
  Keyboard() : is_break(false), shift(false), alt(false), ctrl(false),
    led(0), acks(0), extended(0) {}

  void init()                                          __section(".boot.text");
  static void irqHandler(Keyboard* keyb);
  KeyCode read() { return kbq.recv(); }                 // read keycode
  bool tryRead(KeyCode& kc) { return kbq.tryRecv(kc); } // read keycode NB

  // returns status of lock keys
  bool get_scroll_lock() { return led & 1; }
  bool get_numlock()     { return led & 2; }
  bool get_capslock()    { return led & 4; }

  // returns status of special keys
  bool get_alt()         { return alt; }
  bool get_ctrl()        { return ctrl; }
  bool get_shift()       { return shift; }

  // reset system
  void reset_system();
};

#endif /* _Keyboard_h_ */
