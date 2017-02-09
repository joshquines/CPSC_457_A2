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
#ifndef _Output_h_
#define _Output_h_ 1

#include "generic/Bitmap.h"
#include "machine/SpinLock.h"

#include <cstdarg>

static const char kendl = '\n';

template<typename _CharT, typename _Traits = char_traits<_CharT>>
class OutputBuffer : public basic_streambuf<_CharT, _Traits> {
public:
  typedef _CharT														char_type;
  typedef _Traits														traits_type;
  typedef typename traits_type::int_type		int_type;
  typedef typename traits_type::pos_type		pos_type;
  typedef typename traits_type::off_type		off_type;

  typedef basic_streambuf<_CharT, _Traits>  BaseClass;

protected:
  virtual streamsize xsputn(const char_type* s, streamsize n) = 0;
  virtual int sync() { return BaseClass::sync(); }
};

class KernelOutput {
  SpinLock olock;
  ostream os;

public:
  KernelOutput( OutputBuffer<char>& ob ) : os(&ob) {}

  void lock() { olock.acquire(); }
  void unlock() { olock.release(); }

  void print() {}

  template<typename T, typename... Args>
  void print( const T& msg, const Args&... a ) {
    os << msg;
    print(a...);
  }

  template<typename T, typename... Args>
  void printl( const T& msg, const Args&... a ) {
    ScopedLock<> sl(olock);
    os << msg;
    print(a...);
  }

  ssize_t write(const void *buf, size_t len) {
    ScopedLock<> sl(olock);
    os.write((cbufptr_t)buf, len);
    return len;
  }
};

extern KernelOutput StdOut;
extern KernelOutput StdErr;
extern KernelOutput StdDbg;

class DBG {
public:
  enum Level : size_t {
    Acpi = 0,
    Boot,
    Basic,
    CDI,
    Devices,
    Error,
    Frame,
    File,
    GDBDebug,
    GDBEnable,
    KernMem,
    Libc,
    Lwip,
    MemAcpi,
    Paging,
    PCI,
    Perf,
    Process,
    Scheduler,
    Tests,
    Threads,
    VM,
    Warning,
    MaxLevel
  };

private:
  static Bitmap<> levels;
  static_assert(MaxLevel <= sizeof(levels) * 8, "too many debug levels");

public:
  static void init( char* dstring, bool msg );
  static bool test( Level c ) { return levels.test(c); }

  template<typename... Args> static void out1( Level c, const Args&... a ) {
    if (c && !test(c)) return;
    StdDbg.printl(a...);
#if TESTING_DEBUG_STDOUT
    if (c) StdOut.printl(a...);
#endif
  }
  template<typename... Args> static void outl( Level c, const Args&... a ) {
    if (c && !test(c)) return;
    StdDbg.printl('C', LocalProcessor::getIndex(), '/', FmtHex(CPU::readCR3()), ": ", a..., kendl);
#if TESTING_DEBUG_STDOUT
    if (c) StdOut.printl('C', LocalProcessor::getIndex(), '/', FmtHex(CPU::readCR3()), ": ", a..., kendl);
#endif
  }
  static void outl( Level c ) {
    if (c && !test(c)) return;
    StdDbg.printl(kendl);
#if TESTING_DEBUG_STDOUT
    if (c) StdOut.printl(kendl);
#endif
  }
};

class KOUT {
public:
  template<typename... Args> static void out1( const Args&... a ) {
    StdOut.printl(a...);
#if TESTING_STDOUT_DEBUG
    StdDbg.printl(a...);
#endif
  }
  template<typename... Args> static void outl( const Args&... a ) {
    StdOut.printl(a..., kendl);
#if TESTING_STDOUT_DEBUG
    StdDbg.printl(a..., kendl);
#endif
  }
};

class KERR {
public:
  template<typename... Args> static void out1( const Args&... a ) {
    StdErr.printl(a...);
#if TESTING_STDOUT_DEBUG
    StdDbg.printl(a...);
#endif
  }
  template<typename... Args> static void outl( const Args&... a ) {
    StdErr.printl(a..., kendl);
#if TESTING_STDOUT_DEBUG
    StdDbg.printl(a..., kendl);
#endif
  }
};

template<typename... Args>
static inline void kassertprint1(const Args&... a) {
  StdDbg.lock();
  StdDbg.print(a...);
  StdErr.lock();
  StdErr.print(a...);
}

template<typename... Args>
static inline void kassertprint2(const Args&... a) {
  StdDbg.print(a..., kendl);
  StdDbg.unlock();
  StdErr.print(a..., kendl);
  StdErr.unlock();
}

#define KASSERTN(expr,args...) { if slowpath(!(expr)) { kassertprints( "KASSERT: " #expr " in " __FILE__ ":", __LINE__, __func__); kassertprint2(args); Reboot(); } }
#define KCHECKN(expr,args...)  { if slowpath(!(expr)) { kassertprints(  "KCHECK: " #expr " in " __FILE__ ":", __LINE__, __func__); kassertprint2(args); } }

extern void ExternDebugPrintf(DBG::Level c, const char* fmt, va_list args);

#endif /* _Output_h_ */
