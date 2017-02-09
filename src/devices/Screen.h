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
#ifndef _Screen_h_
#define _Screen_h_ 1

#include "machine/CPU.h"

#include <cstring>

// no lock needed: mutual exclusion through ScreenSegment/KernelOutput
class Screen : public NoObject {
	friend class Machine;

	static const int xmax = 160;
	static const int ymax = 25;
	static char  buffer[xmax * ymax] __aligned(0x1000); // defined in Machine.cc
	static char* video;                                 // defined in Machine.cc

	static bool init(mword displacement) {
		// monochrome screen would be at 0xb0000, increment by 1 byte
		if (((*(uint16_t*)0x410) & 0x30) == 0x30) return false;
		video = (char*)(displacement + 0xb8000);
		memcpy(buffer, video, ymax * xmax);
		return true;
	}

	static vaddr getAddress() { return vaddr(video); }
	static void setAddress( vaddr vma ) { video = (char*)vma; }

	static void scroll( int offset, int length ) {
		memmove(buffer + offset, buffer + offset + xmax, length - xmax);
		memset(buffer + offset + length - xmax, 0, xmax );
		memcpy(video + offset, buffer + offset, length );
	}

	static void setcursor( int position ) {
		position = position / 2;
		CPU::out8(0x3D4, 14);
		CPU::out8(0x3D5, position >> 8);
		CPU::out8(0x3D4, 15);
		CPU::out8(0x3D5, position);
	}

public:
	static int offset( int firstline ) {
		return (firstline - 1) * xmax;
	}

	static int length( int firstline, int lastline ) {
		return (1 + lastline - firstline) * xmax;
	}

	static void cls( int offset, int length ) {
		memset(buffer + offset, 0, length);
		memcpy(video + offset, buffer + offset, length );
	}

	static void putchar( char c, int& position, int offset, int length ) {
		if (c == '\n') {
			position = ((position / xmax) + 1) * xmax;
		} else {
			buffer[position] = c;
			video[position] = c;
			position += 1;
			buffer[position] = 0x07;
			video[position] = 0x07;
			position += 1;
		}
		if (position >= offset + length) {
			scroll(offset, length);
			position -= xmax;
		}
		setcursor(position);
	}

};

class ScreenSegment {
  int offset;
  int length;
  int position;
public:
  ScreenSegment(int firstline, int lastline, int startline = 0)
    : offset(Screen::offset(firstline)),
      length(Screen::length(firstline,lastline)) {
    position = (startline > firstline) ? Screen::offset(startline) : Screen::offset(firstline);
  }
  ScreenSegment(const ScreenSegment&) = delete;            // no copy
  ScreenSegment& operator=(const ScreenSegment&) = delete; // no assignment
  void cls() { Screen::cls( offset, length ); }
  void write(char c) {
    Screen::putchar( c, position, offset, length );
  }
};

#endif /* _Screen_h_ */
