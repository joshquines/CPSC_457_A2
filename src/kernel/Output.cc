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
#include "kernel/MemoryManager.h"
#include "kernel/Output.h"
#include "devices/Screen.h"
#include "devices/Serial.h"

#include <cstring>
#undef __STRICT_ANSI__ // to get access to vsnprintf
#include <cstdio>

class ScreenBuffer : public OutputBuffer<char> {
  ScreenSegment segment;
protected:
  virtual streamsize xsputn(const char* s, streamsize n) {
    for (streamsize i = 0; i < n; i += 1) segment.write(s[i]);
    return n;
  }
public:
  ScreenBuffer(int firstline, int lastline, int startline = 0)
  : segment(firstline, lastline, startline) {}
};

static ScreenBuffer top_screen( 1, 20, 2 );
static ScreenBuffer bot_screen( 21, 25 );

class DebugBuffer : public OutputBuffer<char> {
protected:
  virtual streamsize xsputn(const char* s, streamsize n) {
    for (streamsize i = 0; i < n; i += 1) DebugDevice::write(s[i]);
    return n;
  }
};

static DebugBuffer dbg_buffer;

KernelOutput StdOut(top_screen);
KernelOutput StdErr(bot_screen);
KernelOutput StdDbg(dbg_buffer);

static const char* options[] = {
  "acpi",
  "boot",
  "basic",
  "cdi",
  "devices",
  "error",
  "frame",
  "file",
  "gdbdebug",
  "gdbenable",
  "kmem",
  "libc",
  "lwip",
  "memacpi",
  "paging",
  "perf",
  "pci",
  "process",
  "scheduler",
  "tests",
  "threads",
  "vm",
  "warning",
};

Bitmap<> DBG::levels;               // stored in .bss, initialized early enough!

static_assert( sizeof(options)/sizeof(char*) == DBG::MaxLevel, "debug options mismatch" );

void DBG::init( char* dstring, bool msg ) {
  levels.set(Basic);
  char* wordstart = dstring;
  char* end = wordstart + strlen( dstring );
  for (;;) {
    char* wordend = strchr( wordstart, ',' );
    if ( wordend == nullptr ) wordend = end;
    *wordend = 0;
    size_t level = -1;
    for ( size_t i = 0; i < MaxLevel; ++i ) {
      if ( !strncmp(wordstart,options[i],wordend - wordstart) ) {
        if ( level == size_t(-1) ) level = i;
        else {
          if (msg) {
            KERR::outl("multiple matches for debug option: ", wordstart);
          }
          goto nextoption;
        }
      }
    }
    if ( level != size_t(-1) ) {
      levels.set(level);
      if (msg) {
        StdDbg.printl("matched debug option: ", wordstart, '=', options[level], kendl);
      }
    } else if (msg) {
      KERR::outl("unmatched debug option: ", wordstart);
    }
nextoption:
  if ( wordend == end ) break;
    *wordend = ',';
    wordstart = wordend + 1;
  }
}

void kassertprints(const char* const loc, int line, const char* const func) {
  kassertprint1(loc, line, " in ", func);
}

void kassertprinte(const char* const msg) {
  kassertprint2(" - ", msg);
}

void kassertprinte(const unsigned long long num) {
  kassertprint2(" - ", num);
}

void kassertprinte(const FmtHex& ptr) {
  kassertprint2(" - ", ptr);
}

void kassertprinte() {
  kassertprint2();
}

void ExternDebugPrintf(DBG::Level c, const char* fmt, va_list args) {
  va_list tmpargs;
  va_copy(tmpargs, args);
  int size = vsnprintf(nullptr, 0, fmt, tmpargs);
  va_end(tmpargs);
  if (size < 0) return;
  size += 1;
  char* buffer = knewN<char>(size);
  vsnprintf(buffer, size, fmt, args);
  DBG::out1(c, buffer);
  kdelete(buffer, size);
}
