#include "runtime/Scheduler.h"
#include "runtime/Thread.h"
#include "kernel/MemoryManager.h"
#include "kernel/Output.h"
#include "machine/APIC.h"
#include "machine/Machine.h"
#include "machine/Processor.h"
#include "devices/Serial.h"

#include <cstring>
#include <map>

#undef __STRICT_ANSI__ // get declaration of 'snprintf'
#include <cstdio>

// convert KOS CPU ID (0...n-1) to GDB's thread ID (1...n)
static int cTID() { return LocalProcessor::getIndex() + 1; }


/************************* assembler code interface *************************/

extern "C" void resumeExecution(void*) __noreturn;

extern "C" int get_char(const char* addr);
extern "C" void set_char(char* addr, int val);

extern "C" void catchException0x00();
extern "C" void catchException0x01();
extern "C" void catchException0x02();
extern "C" void catchException0x03();
extern "C" void catchException0x04();
extern "C" void catchException0x05();
extern "C" void catchException0x06();
extern "C" void catchException0x07();
extern "C" void catchException0x08();
extern "C" void catchException0x09();
extern "C" void catchException0x0a();
extern "C" void catchExceptionFault0x0b();
extern "C" void catchExceptionFault0x0c();
extern "C" void catchExceptionFault0x0d();
extern "C" void catchExceptionFault0x0e();
extern "C" void catchException0x10();
extern "C" void catchException0xef(); // cf. APIC::StopIPI

// variables shared with GdbAsm.S
volatile int gdbFaultHandlerEnabled = 0; // can activate fault handler for gdb session
volatile int gdbFaultHandlerCalled  = 0; // flags an error from mem2hex/hex2mem


/************************** remote serial protocol **************************/

static const char hexchars[]="0123456789abcdef";

static int char2hex(char ch) {
  if ((ch >= '0') && (ch <= '9')) return (ch - '0');
  if ((ch >= 'a') && (ch <= 'f')) return (ch - 'a') + 10;
  if ((ch >= 'A') && (ch <= 'F')) return (ch - 'A') + 10;
  return -1;
}

static void putbyte(unsigned char ch) {
  SerialDevice::write(ch);
}

static int getbyte() {
  return SerialDevice::read();
}

// scan for the sequence $<data>#<checksum>
static char* getpacket(char* buffer, int buflen) {
  char ch;
  while (true) {
    while ((ch = getbyte()) != '$'); // find start of packet
retry:
    char checksum = 0;
    int count = 0;
    for (; count < buflen - 1; count += 1) {
      ch = getbyte();
      if (ch == '$') goto retry;
      if (ch == '#') break;
      checksum += ch;
      buffer[count] = ch;
    }
    buffer[count] = 0;
    if (ch == '#') {                 // found '#' now validate checksum
      ch = getbyte();
      char xmitcsum = char2hex(ch) << 4;
      ch = getbyte();
      xmitcsum += char2hex(ch);      // xmitsum is computed checksum
      if (checksum != xmitcsum) {
        putbyte('-');                // failed checksum
      } else {
        putbyte('+');                // success
        if (buffer[2] == ':') {      // if seq ID present, reply seq ID
          putbyte(buffer[0]);
          putbyte(buffer[1]);
          return &buffer[3];
        }
        return &buffer[0];
      } // if
    } // if
  } // while
}

// send the packet in buffer
static void putpacket(char *buffer) {
  DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): sending ", buffer);
  do {                                // $<packet info>#<checksum>.
    putbyte('$');                     // start of packet
    unsigned char checksum = 0;
    for ( int count = 0; buffer[count]; count += 1 ) {
      putbyte(buffer[count]);
      checksum += buffer[count];
    }
    putbyte('#');                     // end of data
    putbyte(hexchars[checksum >> 4]); // write checksum
    putbyte(hexchars[checksum % 16]);
  } while (getbyte() != '+');         // try until ack ('+') is heard
}

// put mem to buf in hex format and return next buf position
static char* mem2hex(const char *mem, char *buf, int count, int may_fault) {
  gdbFaultHandlerCalled  = 0;
  gdbFaultHandlerEnabled = may_fault;
  for (int i = 0; i < count; i++) {
    unsigned char ch = get_char(mem++);
    if (gdbFaultHandlerCalled) return buf;
    *buf++ = hexchars[ch >> 4];
    *buf++ = hexchars[ch % 16];
  }
  *buf = 0;
  gdbFaultHandlerEnabled = 0;
  return buf;
}

// put buf in hex format to mem and return next mem position
static char* hex2mem(char *buf, char *mem, int count, int may_fault) {
  gdbFaultHandlerCalled  = 0;
  gdbFaultHandlerEnabled = may_fault;
  for (int i = 0; i < count; i++) {
    unsigned char ch = char2hex(*buf++) << 4;
    ch = ch + char2hex(*buf++);
    set_char(mem++, ch);
    if (gdbFaultHandlerCalled) return mem;
  }
  gdbFaultHandlerEnabled = 0;
  return mem;
}

// convert hex number in ptr to integer
static int hexToInt(char **ptr, long *intValue) {
  int numChars = 0;
  int hexValue = 0;
  *intValue = 0;
  while (**ptr) {
    hexValue = char2hex(**ptr);
    if (hexValue >= 0) {
      *intValue = (*intValue << 4) | hexValue;
      numChars++;
    } else break;
    ++(*ptr);
  }
  return numChars;
}


/****************** multi-core data structures and routines ******************/

typedef uint64_t reg64_t;
typedef uint32_t reg32_t;

enum Reg {
  RAX, RBX, RCX, RDX, RSI, RDI, RBP, RSP,     // registers are in order gdb front-end expects!
  R8, R9, R10, R11, R12, R13, R14, R15, RIP,  // it is preferred to not reorder
  EFLAGS = 0, CS, SS, DS, ES, FS, GS          // 32-bit registers
};

static const size_t reg64cnt  = 17;
static const size_t reg32cnt  = 7;

static const int TF = 0x100;

struct GdbCpuBase {
  /* Registers are divided into 64/32-bits and stored in separate buffers
   * to simplify send/receive operations.  Registers are ordered as the
   * above enum, because gdb front-end differentiates register sizes and
   * expects this order.  */

  // These two arrays *MUST* be the first two members (gdb_asm_functions.S)
  reg64_t reg64buf[reg64cnt];
  reg32_t reg32buf[reg32cnt];
  volatile enum { Unknown, Stopped, Running } state;
  int64_t ticket;
  GdbCpuBase() : state(Unknown), ticket(0) {}
} __packed;

struct GdbCpu : public GdbCpuBase {
  static const size_t stackSize = (1<<16);
  char stack[stackSize-sizeof(GdbCpuBase)];
} __packed;

static GdbCpu* cpus = nullptr;

void startGdbCpu(mword idx) {
  if (!DBG::test(DBG::GDBEnable)) return;
  cpus[idx].state = GdbCpu::Running;
}

void initGdb(mword bspIdx) {
  if (!DBG::test(DBG::GDBEnable)) return;
  cpus = knewN<GdbCpu>(Machine::getProcessorCount());
  Machine::setupIDT(0x00, (vaddr)catchException0x00);
  Machine::setupIDT(0x01, (vaddr)catchException0x01);
  Machine::setupIDT(0x02, (vaddr)catchException0x02);
  Machine::setupIDT(0x03, (vaddr)catchException0x03);
  Machine::setupIDT(0x04, (vaddr)catchException0x04);
  Machine::setupIDT(0x05, (vaddr)catchException0x05);
  Machine::setupIDT(0x06, (vaddr)catchException0x06);
  Machine::setupIDT(0x07, (vaddr)catchException0x07);
  Machine::setupIDT(0x08, (vaddr)catchException0x08);
  Machine::setupIDT(0x09, (vaddr)catchException0x09);
  Machine::setupIDT(0x0a, (vaddr)catchException0x0a);
  Machine::setupIDT(0x0b, (vaddr)catchExceptionFault0x0b);
  Machine::setupIDT(0x0c, (vaddr)catchExceptionFault0x0c);
  Machine::setupIDT(0x0d, (vaddr)catchExceptionFault0x0d);
  Machine::setupIDT(0x0e, (vaddr)catchExceptionFault0x0e);
  Machine::setupIDT(0x10, (vaddr)catchException0x10);
  Machine::setupIDT(0xef, (vaddr)catchException0xef);
  startGdbCpu(bspIdx);
  StdOut.printl("Waiting for Gdb connection", kendl);
  CPU::forceTrap();
}

static GdbCpu& getCPU() { return cpus[LocalProcessor::getIndex()]; }

// returns buffer storing gdb manipulated registers for current CPU
static reg64_t* getAllRegisters(mword cpuIdx = cTID()-1) {
  return cpus[cpuIdx].reg64buf;
}

static reg64_t* getRegptr64(size_t regno, mword cpuIdx = cTID()-1) {
  return cpus[cpuIdx].reg64buf + regno;
}
static reg32_t* getRegptr32(size_t regno, mword cpuIdx = cTID()-1) {
  return cpus[cpuIdx].reg32buf + regno;
}

static reg64_t getReg64(size_t regno, mword cpuIdx = cTID()-1) {
  return cpus[cpuIdx].reg64buf[regno];
}
static reg32_t getReg32(size_t regno, mword cpuIdx = cTID()-1) {
  return cpus[cpuIdx].reg32buf[regno];
}
static void setReg64(size_t regno, reg64_t val, mword cpuIdx = cTID()-1) {
  cpus[cpuIdx].reg64buf[regno] = val;
}
static void setReg32(size_t regno, reg32_t val, mword cpuIdx = cTID()-1) {
  cpus[cpuIdx].reg32buf[regno] = val;
}


/********************** basic packet interface: queries **********************/

static map<mword,char> bpMap;
static mword _Unwind_DebugHook_addr = 0;
static bool  _Unwind_DebugHook_break = false; // detect if breakpoint was inserted on _Unwind_DebugHook

// gdb front-end queries gdb stub's supported features
static bool handle_qSupported(char* in, char* out) {
  if (!strncmp(in, "Supported", 9)) {
    strcpy(out, "PacketSize=1000");              // allow max 4096 bytes per packet
    return true;
  }
  return false;
}

// return current thread ID
static bool handle_qC(char* in, char* out) {
  if (*in == 'C') {
    int threadId = cTID();
    out[0] = 'Q'; out[1] = 'C';
    out[2] = hexchars[threadId >> 4];
    out[3] = hexchars[threadId % 16];
    out[4] = 0;
    return true;
  }
  return false;
}

// tells if gdb stub process is attached to existing processor
static bool handle_qAttached(char* in, char* out) {
  if (!strncmp(in, "Attached", 8)) {
    out[0] = '0';   // stub should be killed when session is ended with 'quit'
    out[1] = 0;
    return true;
  }
  return false;
}

// tell section offsets target used when relocating kernel image?
static bool handle_qOffsets(char* in, char* out) {
  if (!strncmp(in, "Offsets", 7)) {
    strcpy(out, "Text=0;Data=0;Bss=0"); // no relocation
    return true;
  }
  return false;
}

// gdb stub can ask for symbol lookups
static bool handle_qSymbol(char* in, char* out) {
  if (!strncmp(in, "Symbol::", 8)) {
    strcpy(out, "qSymbol:"); out += 8;
    const char* symbolName = "_Unwind_DebugHook";
    mem2hex(symbolName, out, strlen(symbolName), 0);
    return true;
  }
  return false;
}

// parse replies from above symbol lookup requests (Only looks up _Unwind_DebugHook for now)
static bool handle_qSymbolResponse(char* in, char* out) {
  if (!strncmp(in, "Symbol:", 7)) {
    in = in + 7;
    long addr;
    if (hexToInt(&in, &addr)) {
      char buf[32];
      char* endbuf = hex2mem(++in, buf, strlen("_Unwind_DebugHook"), 0);
      endbuf[0] = 0;
      KASSERT1(!strcmp(buf, "_Unwind_DebugHook"), buf);
      DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): _Unwind_DebugHook address ", FmtHex(addr), '/', buf);
      _Unwind_DebugHook_addr = addr;
      strcpy(out, "OK");
      return true;
    }
  }
  return false;
}

// query if trace experiment running now
static bool handle_qTStatus(char* in, char* out) {
  if (!strncmp(in, "TStatus", 7)) {
    putpacket(out);
    return true;
  }
  return false;
}

// query list of all active thread IDs
static bool handle_qfThreadInfo(char* in, char* out) {
  if (!strncmp(in, "fThreadInfo", 11)) {
    out[0] = 'm'; ++out;
    for (mword i = 0; i < Machine::getProcessorCount(); ++i) {
      if (i > 0) {
        out[0] = ',';
        out += 1;
      }
      out[0] = hexchars[(i+1) >> 4];
      out[1] = hexchars[(i+1) % 16];
      out += 2;
    }
    out[0] = 0;
    return true;
  }
  return false;
}

// continued querying list of all active thread IDs
static bool handle_qsThreadInfo(char* in, char* out) {
  if (!strncmp(in, "sThreadInfo", 11)) {
    out[0] = 'l';
    out[1] = 0;
    return true;
  }
  return false;
}

// ask printable string description of a thread's attribute (used by 'info threads')
static bool handle_qThreadExtraInfo(char* in, char* out) {
  static const char* states[] = { "Unknown", "Stopped", "Running" };
  if (!strncmp(in, "ThreadExtraInfo", 15)) {
    in += 16;
    long threadId;
    if (hexToInt(&in, &threadId)) {
      char info[40];
      snprintf(info, 40, "CPU %lu [%s]", threadId-1, states[cpus[threadId-1].state]);
      mem2hex(info, out, strlen(info), 0);
      return true;
    }
  }
  return false;
}

/********************* basic packet interface: handlers *********************/

// thread alive status
static bool handleThreadAlive(char* in, char* out) {
  long threadId;
  if (hexToInt(&in, &threadId)) {
    if (mword(threadId) >= 1 && mword(threadId) <= Machine::getProcessorCount()) {
      strcpy(out, "OK");
      return true;
    }
  }
  strcpy(out, "E01");
  return false;
}

// store thread id specified in 'H' packet for next ?/g/G/p/P operation
static int Htid = -2;

// tells reason target halted (specifically, which thread stopped with what signal)
static bool handleReasonTargetHalted(char* in, char* out, int sigval, int idx = cTID()) {
  if (Htid != -2) {
    idx = Htid;
    Htid = -2;
  }
  out[0] = 'T';
  out[1] = hexchars[sigval >> 4];
  out[2] = hexchars[sigval % 16];
  out += 3;
  strncpy(out, "thread:", 7);
  out += 7;
  out[0] = hexchars[idx >> 4];
  out[1] = hexchars[idx % 16];
  out += 2;
  out[0] = ';';
  out[1] = hexchars[Reg::RIP >> 4];
  out[2] = hexchars[Reg::RIP % 16];
  out[3] = ':';
  out += 4;
  reg64_t val = getReg64(Reg::RIP, idx - 1);
  out = mem2hex((char*)&val, out, sizeof(reg64_t), 0);
  out[0] = ';';
  out[1] = 0;
  return true;
}

// read all registers
static bool handleReadRegisters(char* in, char* out, mword idx = cTID()) {
  if (Htid != -2) {
    idx = Htid - 1;
    Htid = -2;
  } else {
    idx -= 1;
  }
  mem2hex((char*)getAllRegisters(idx), out, reg64cnt * sizeof(reg64_t) + reg32cnt * sizeof(reg32_t), 0);
  return true;
}

// write all registers
static bool handleWriteRegisters(char* in, char* out, mword idx = cTID()) {
  if (Htid != -2) {
    idx = Htid - 1;
    Htid = -2;
  } else {
    idx -= 1;
  }
  hex2mem(in, (char*)getAllRegisters(idx), reg64cnt * sizeof(reg64_t) + reg32cnt * sizeof(reg32_t), 0);
  strcpy(out, "OK");
  return true;
}

// read register n with value r
static bool handleReadRegister(char* in, char* out, mword idx = cTID()) {
  if (Htid != -2) {
    idx = Htid - 1;
    Htid = -2;
  } else {
    idx -= 1;
  }
  long regno;
  if (hexToInt(&in, &regno) && *in++ == '=') { 
    if (regno >= 0 && regno < (long)reg64cnt) {
      reg64_t val = getReg64(regno, idx);
      mem2hex((char*)&val, out, sizeof(reg64_t), 0);
      return true;
    }
    regno -= reg64cnt;
    if (regno >= 0 && regno < (long)reg32cnt) {
      reg32_t val = getReg32(regno, idx);
      mem2hex((char*)&val, out, sizeof(reg32_t), 0);
      return true;
    }
  }
  strcpy(out, "E01");
  return false;
}

// write register n with value r
static bool handleWriteRegister(char* in, char* out, mword idx = cTID()) {
  if (Htid != -2) {
    idx = Htid - 1;
    Htid = -2;
  } else {
    idx -= 1;
  }
  long regno;
  if (hexToInt(&in, &regno) && *in++ == '=') {
    if (regno >= 0 && regno < (long)reg64cnt) {
      hex2mem(in, (char*)getRegptr64(regno, idx), sizeof(reg64_t), 0);
      strcpy(out, "OK");
      return true;
    }
    regno -= reg64cnt;
    if (regno >= 0 && regno < (long)reg32cnt) {
      hex2mem(in, (char*)getRegptr32(regno, idx), sizeof(reg32_t), 0);
      strcpy(out, "OK");
      return true;
    }
  }
  strcpy(out, "E01");
  return false;
}

// choose thread for next operation, deprected for control
static bool handleH(char* in, char* out) {
  char op = *in;
  if (op == 'g') {
    ++in;
    long threadId;
    if (hexToInt(&in, &threadId)) {
      if (threadId > 0) {
        Htid = threadId;
        strcpy(out, "OK");
        return true;
      }
    }
  }
  strcpy(out, "E01");
  return false;
}

// read memory from given address
static bool handleReadMemory(char* in, char* out) {
  long addr, length;
  if (hexToInt(&in, &addr) && *in++ == ',' && hexToInt(&in, &length)) {
    mem2hex((char *)addr, out, length, 1);
    if (gdbFaultHandlerCalled) strcpy(out, "E01");
    return !gdbFaultHandlerCalled;
  }
  strcpy(out, "E01");
  return false;
}

// write to memory at given address
static bool handleWriteMemory(char* in, char* out) {
  long addr, length;
  if (hexToInt(&in, &addr) && *in++ == ',' && hexToInt(&in, &length) && *in++ == ':') {
    hex2mem(in, (char *)addr, length, 1);
    if (gdbFaultHandlerCalled) strcpy(out, "E01");
    else strcpy(out, "OK");
    return !gdbFaultHandlerCalled;
  }
  strcpy(out, "E01");
  return false;
}

// set software breakpoints
static bool handleSetSoftBreak(char* in, char* out) {
  long addr, length;
  while (hexToInt(&in, &addr) && *in++ == ',' && hexToInt(&in, &length)) {
    DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): set breakpoint at ", FmtHex(addr));
    gdbFaultHandlerCalled  = 0;
    gdbFaultHandlerEnabled = 1;       // enable fault handler
    char opCode = get_char((char *)addr);
    if (gdbFaultHandlerCalled) break;
    bpMap[(mword)addr] = opCode;
    KASSERT1(length == 1, length);    // 1 byte for x86-64?
    set_char((char *)addr, 0xcc);     // set break
    if (gdbFaultHandlerCalled) break;
    if ((mword)addr == _Unwind_DebugHook_addr) {
      _Unwind_DebugHook_break = true;  // only set for step/next
      DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): _Unwind_DebugHook breakpoint set");
    }
    gdbFaultHandlerEnabled = 0;
    strcpy(out, "OK");
    return true;
  }
  strcpy(out, "E01");
  return false;
}

// remove software breakpoints
static bool handleRemoveSoftBreak(char* in, char* out) {
  long addr, length;
  while (hexToInt(&in, &addr) && *in++ == ',' && hexToInt(&in, &length)) {
    DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): remove breakpoint at ", FmtHex(addr));
    if (bpMap.count((mword)addr) == 0) break;
    gdbFaultHandlerCalled  = 0;
    gdbFaultHandlerEnabled = 1;       // enable fault handler
    set_char((char *)addr, bpMap[(mword)addr]);
    if (gdbFaultHandlerCalled) break;
    if ((mword)addr == _Unwind_DebugHook_addr) {
      _Unwind_DebugHook_break = false; // remove always as a last step before step/next returns
      DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): _Unwind_DebugHook breakpoint removed");
    }
    gdbFaultHandlerEnabled = 0;
    strcpy(out, "OK");
    return true;
  }
  strcpy(out, "E01");
  return false;
}


/************************** vCont packet interface **************************/

struct VCont {
  char action;
  long signal;
  long tid;
  VCont() : action(0) {}
  bool pending() { return action != 0 && (tid <= 0 || tid == cTID()); }
  void parse(char* in, char* out) {
    if (strncmp(in, "Cont", strlen("Cont"))) return;
    in += 4;
    if (*in == '?') {
      strcpy(out, "vCont;c;C;s;S"); // supported vCont packets
      return;
    }
    KASSERT0(action == 0);
    if (*in != ';') return;
    ++in;
    char a = *in;
    ++in;
    if (a == 'C' || a == 'S') {
      a = tolower(a);
      while (*in == ' ') ++in;
      if (!hexToInt(&in, &signal)) return;
    } else {
      signal = 0;
    }
    if (*in == ':') {
      ++in;
      if (!hexToInt(&in, &tid)) return;
    } else {
      tid = -1;
    }
//    KASSERT1(tid >= -1 && mword(tid) <= Machine::getProcessorCount(), tid);
    DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): T=", tid, " action: ", a, " signal: ", signal);
    if (a != 'c' && a != 's') return;
    action = a;
  }
  void detach() {
    action ='c';
    signal = 0;
    tid = -1;
  }
} vCont;

static char in[4096];
static char out[4096];

static void handlePacket(int sigval) {
  out[0] = 0;
  char *ptr = getpacket(in, 4096);     // get request from gdb
  DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): recv ", ptr);
  switch (*ptr++) {
    case 'q': {   // queries
      if (handle_qSupported(ptr, out)) break;
      if (handle_qC(ptr, out)) break;
      if (handle_qAttached(ptr, out)) break;
      if (handle_qOffsets(ptr, out)) break;
      if (handle_qSymbol(ptr, out)) break;
      if (handle_qSymbolResponse(ptr, out)) break;
      if (handle_qTStatus(ptr, out)) break;
      if (handle_qfThreadInfo(ptr, out)) break;
      if (handle_qsThreadInfo(ptr, out)) break;
      if (handle_qThreadExtraInfo(ptr, out)) break;
      // if this happens: time to support new packets
      strcpy(out, "qUnimplemented");
    } break;
    case 'v': vCont.parse(ptr, out); break;            // vCont packets
    case 'T': handleThreadAlive(ptr, out); break;      // thread alive?
    case '?': handleReasonTargetHalted(ptr, out, sigval); break;
    case 'g': handleReadRegisters(ptr, out); break;    // read all registers
    case 'G': handleWriteRegisters(ptr, out); break;   // write all registers
    case 'p': handleReadRegister(ptr, out); break;     // read one register
    case 'P': handleWriteRegister(ptr, out); break;    // write one register
    case 'H': handleH(ptr, out); goto Htidset;         // specific thread request
    case 'm': handleReadMemory(ptr, out); break;       // read memory
    case 'M': handleWriteMemory(ptr, out); break;      // write memory
    case 'k': vCont.detach(); break;                   // kill request -> detach
    case 'Z': {                                        // breakpoints, watchpoints, etc
      long type;
      if (hexToInt(&ptr, &type)) {
        ptr++;
        switch (type) {
          case 0: handleSetSoftBreak(ptr, out); break;
          default: strcpy(out, "E01");
        }
      }
    } break;
    case 'z': {
      long type;
      if (hexToInt(&ptr, &type)) {
        ptr++;
        switch (type) {
          case 0: handleRemoveSoftBreak(ptr, out); break;
          default: strcpy(out, "E01");
        }
      }
    } break;
    default: strcpy(out, "E01");
  } // switch
  Htid = -2;
Htidset:
  // send back reply to the request
  if (out[0] != 0) putpacket(out);
}

// ticket lock infrastructure for stopping & mediating access to frontend
static SpinLock lock;
static int64_t tcounter = 1;
static volatile int64_t serving = 1;
static volatile bool allstop = false;

// pin thread to core during 'step' and 'next'
static Thread* savedThread = nullptr;
static Scheduler* savedAffinity = nullptr;

static void setGdbAffinity() {
  if (!savedThread) {
    savedThread = LocalProcessor::getCurrThread();
    savedAffinity = savedThread->getAffinity();
    savedThread->setAffinity(LocalProcessor::getScheduler());
    DBG::outl(DBG::GDBDebug, "GDB(", cTID(),") set GDB affinity");
  }
}

static void resetGdbAffinity() {
  if (savedThread) {
    savedThread->setAffinity(savedAffinity);
    savedThread = nullptr;
    DBG::outl(DBG::GDBDebug, "GDB(", cTID(),") reset GDB affinity");
  }
}

// does all command processing for interfacing to gdb
extern "C" void handle_exception (int64_t vec) __noreturn;
extern "C" void handle_exception (int64_t vec) {
  int sigval;
  switch (vec) {
    case  0: sigval =  8; break;            // divide by zero
    case  1: sigval =  5; break;            // debug exception
    case  2: sigval =  0; break;            // nmi exception
    case  3: sigval =  5; break;            // breakpoint
    case  4: sigval = 16; break;            // into instruction (overflow)
    case  5: sigval = 16; break;            // bound instruction
    case  6: sigval =  4; break;            // Invalid opcode
    case  7: sigval =  8; break;            // coprocessor not available
    case  8: sigval =  7; break;            // double fault
    case  9: sigval = 11; break;            // coprocessor segment overrun
    case 10: sigval = 11; break;            // Invalid TSS
    case 11: sigval = 11; break;            // Segment not present
    case 12: sigval = 11; break;            // stack exception
    case 13: sigval = 11; break;            // general protection
    case 14: sigval = 11; break;            // page fault
    case 16: sigval =  7; break;            // coprocessor error
    default: sigval =  7;                   // "software generated"
  }

  LocalProcessor::lock();                   // inflate lock count to keep IRQs disabled
  getCPU().state = GdbCpu::Stopped;

  if (vec == 3) {                           // hit breakpoint
    getCPU().reg64buf[Reg::RIP] -= 1;       // decrement program counter
    KASSERT0(*(unsigned char*)getReg64(Reg::RIP) == 0xCC);
  }

  DBG::outl(DBG::GDBDebug, "GDB(", cTID(),"): vec=", vec,
    " eflags=", FmtHex(getReg32(Reg::EFLAGS)),
    " rip=", FmtHex(getReg64(Reg::RIP)));

  if (vec == APIC::StopIPI) {               // allstop signal
    MappedAPIC()->sendEOI();                // must confirm IPI to APIC

  } else {                                  // else report to frontend
    // obtain ticket, if current ticket old
    lock.acquire();
    if (serving - getCPU().ticket > 0) {
      getCPU().ticket = tcounter;
      ++tcounter;
    }
    lock.release();

    // wait my turn
    while (getCPU().ticket != serving) CPU::Pause();

    // default allstop mode: stop everybody
    allstop = true;
    for (mword i = 0; i < Machine::getProcessorCount(); i++) {
      if (cpus[i].state == GdbCpu::Running) {
        Machine::sendIPI(i, APIC::StopIPI);
        while (cpus[i].state != GdbCpu::Stopped) CPU::Pause();
      }
    }

    // if previous vCont packet processed, send stop reply
    if (vCont.action != 0) {
      char buf[128];
      if (handleReasonTargetHalted(nullptr, buf, sigval)) {
        putpacket(buf);
      }
      vCont.action = 0;
    }

    // handle query packets until next vCont packet arrives
    do handlePacket(sigval); while (vCont.action == 0);
  } // keep ticket valid for next section

  while (allstop) {                     // while under debugger control

    // obtain ticket, if current ticket old
    lock.acquire();
    if (serving - getCPU().ticket > 0) {
      getCPU().ticket = tcounter;
      ++tcounter;
    }
    lock.release();

    // wait my turn
    while (getCPU().ticket != serving) CPU::Pause();
    
    if (vCont.pending()) {              // vCont packet for this thread?

      // clear TF in saved eflags - new value installed during iret
      reg32_t eflags = getReg32(Reg::EFLAGS) & ~TF;

      if (vCont.action == 's') {        // step: set TF
        KASSERT1(vCont.tid == cTID(), vCont.tid);
        eflags = eflags | TF;
        setGdbAffinity();
      } else if (vCont.action == 'c') {
        if (_Unwind_DebugHook_break) {  // next: wake all
          KASSERT1(vCont.tid == -1, vCont.tid);
          allstop = false;
          setGdbAffinity();
        } else {                        // cont: wake all
          KASSERT1(vCont.tid == -1, vCont.tid);
          allstop = false;
          resetGdbAffinity();
        }
      }

      setReg32(Reg::EFLAGS, eflags);
      ++serving;                        // trigger next core before resume
      break;
    }

    ++serving;                          // trigger next core, but keep waiting
  }

  getCPU().state = GdbCpu::Running;
  LocalProcessor::unlock();             // deflate lock count
  resumeExecution(&getCPU());           // resume to KOS, no return
}

// called from GdbAsm.S
extern "C" GdbCpu* getGdbCpu() {
  return &getCPU();
}

extern "C" mword getGdbStack(GdbCpu* cpu) {
  return mword(cpu) + GdbCpu::stackSize;
}
