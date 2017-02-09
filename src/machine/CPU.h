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
#ifndef _CPU_h_
#define _CPU_h_ 1

#include "generic/bitmanip.h"

namespace MSR {
  enum Register : uint32_t {
    APIC_BASE      = 0x0000001B,

    PMC0           = 0x000000C1, /* PMU event counters */

    SYSENTER_CS    = 0x00000174,
    SYSENTER_ESP   = 0x00000175,
    SYSENTER_EIP   = 0x00000176,

    PERFEVTSEL0    = 0x00000186, /* PMU event selectors */

    EFER           = 0xC0000080,

    TSC_DEADLINE   = 0x000006E0,

    SYSCALL_STAR   = 0xC0000081,
    SYSCALL_LSTAR  = 0xC0000082,
    SYSCALL_CSTAR  = 0xC0000083,
    SYSCALL_SFMASK = 0xC0000084,

    FS_BASE        = 0xC0000100,
    GS_BASE        = 0xC0000101,
    KERNEL_GS_BASE = 0xC0000102
  };

  static inline void read( Register msr, uint32_t& lo, uint32_t& hi ) {
    asm volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr));
  }
  static inline uint64_t read( Register msr ) {
    uint32_t lo, hi;
    read(msr, lo, hi);
    return (uint64_t(hi) << 32) | lo;
  }

  static inline void readpmc( Register msr, uint32_t& lo, uint32_t& hi ) {
    asm volatile("rdpmc" : "=a"(lo), "=d"(hi) : "c"(msr));
  }
  static inline uint64_t readpmc( Register msr ) {
    uint32_t lo, hi;
    readpmc(msr, lo, hi);
    return (uint64_t(hi) << 32) | lo;
  }

  static inline void write( Register msr, uint32_t lo, uint32_t hi ) {
    asm volatile("wrmsr" :: "a"(lo), "d"(hi), "c"(msr));
  }
  static inline void write( Register msr, uint64_t val ) {
    write(msr, val & 0xFFFFFFFF, val >> 32);
  }

  static inline bool isBSP() {
    return read(APIC_BASE) & bitmask<mword>(8,1);
  }
  static inline void enableAPIC() {
    write(APIC_BASE, read(APIC_BASE) | bitmask<mword>(11,1));
  }
  static inline void enableNX() {
    write(EFER, read(EFER) | bitmask<mword>(11,1));
  }
  static inline void enableSYSCALL() {
    write(EFER, read(EFER) | bitmask<mword>(0,1));
  }

  static inline void startPMC(uint32_t index, uint64_t arg) { // arg = 0 stops counting
    write( Register(PMC0 + index), 0 );
    write( Register(PERFEVTSEL0 + index), arg );
  }
  static inline uint64_t readPMC(uint32_t index) {
    return read( Register(PMC0 + index) );
//  return readPMC( Register(PMC0 + index) );
  }
};

namespace CPU {
  static inline void out8( uint16_t port, uint8_t val ) {
    asm volatile("outb %0, %1" :: "a"(val), "Nd"(port));
  }

  static inline uint8_t in8( uint16_t port ) {
    uint8_t ret;
    asm volatile("inb %1, %0" : "=a"(ret) : "Nd"(port));
    return ret;
  }

  static inline void out16( uint16_t port, uint16_t val ) {
    asm volatile("outw %0, %1" :: "a"(val), "Nd"(port));
  }

  static inline uint16_t in16( uint16_t port ) {
    uint16_t ret;
    asm volatile("inw %1, %0" : "=a"(ret) : "Nd"(port));
    return ret;
  }

  static inline void out32( uint16_t port, uint32_t val ) {
    asm volatile("outl %0, %1" :: "a"(val), "Nd"(port));
  }

  static inline uint32_t in32( uint16_t port ) {
    uint32_t ret;
    asm volatile("inl %1, %0" : "=a"(ret) : "Nd"(port));
    return ret;
  }

  static inline void Pause() {
    asm volatile("pause");
  }

  static inline void Halt() {
    asm volatile("hlt" ::: "memory");
  }

  static inline void SwapGS() {
    asm volatile("swapgs");
  }

  static inline void InvTLB(mword val) {
    // Somehow the extra move into a register seems to be necessary to
    // to make invlpg work reliably (e.g. bochs, but also observed on HW),
    // instead of just: asm volatile( "invlpg %0" :: "m"(val) : "memory" );
    asm volatile("invlpg (%0)" :: "r"(val) : "memory");
  }

  static inline mword readTSC() {
    mword a,d; asm volatile("rdtsc" : "=a"(a), "=d"(d)); return (d<<32)|a;
  }

  static inline mword readCR0() { // see Intel Vol 3, Section 2.5 "Control Registers"
    mword val; asm volatile("mov %%cr0, %0" : "=r"(val) :: "cc"); return val;
  }
  static inline void writeCR0( mword val ) {
    asm volatile("mov %0, %%cr0" :: "r"(val) : "memory", "cc");
  }
  static const BitString<mword, 0,1> PE;
  static const BitString<mword, 1,1> MP;
  static const BitString<mword, 2,1> EM;
  static const BitString<mword, 3,1> TS;
  static const BitString<mword, 4,1> ET;
  static const BitString<mword, 5,1> NE;
  static const BitString<mword,16,1> WP;
  static const BitString<mword,18,1> AM;
  static const BitString<mword,29,1> NW;
  static const BitString<mword,30,1> CD;
  static const BitString<mword,31,1> PG;

  static inline mword readCR2() {
    mword val; asm volatile("mov %%cr2, %0" : "=r"(val) :: "cc"); return val;
  }

  static inline mword readCR3() {
    mword val; asm volatile("mov %%cr3, %0" : "=r"(val) :: "cc"); return val;
  }
  static inline void writeCR3( mword val ) {
    asm volatile("mov %0, %%cr3" :: "r"(val) : "memory", "cc");
  }

  static inline mword readCR4() {
    mword val; asm volatile("mov %%cr4, %0" : "=r"(val) :: "cc"); return val;
  }
  static inline void writeCR4( mword val ) {
    asm volatile("mov %0, %%cr4" :: "r"(val) : "memory", "cc");
  }
  static const BitString<mword, 0,1> VME;
  static const BitString<mword, 1,1> PVI;
  static const BitString<mword, 2,1> TSD;
  static const BitString<mword, 3,1> DE;
  static const BitString<mword, 4,1> PSE;
  static const BitString<mword, 5,1> PAE;
  static const BitString<mword, 6,1> MCE;
  static const BitString<mword, 7,1> PGE;        // G bit for page entries
  static const BitString<mword, 8,1> PCE;        // RDPMC available at user-level
  static const BitString<mword, 9,1> OSFXSR;
  static const BitString<mword,10,1> OSXMMEXCPT;
  static const BitString<mword,13,1> VMXE;
  static const BitString<mword,14,1> SMXE;
  static const BitString<mword,16,1> FSGSBASE;
  static const BitString<mword,17,1> PCIDE;
  static const BitString<mword,18,1> OSXSAVE;
  static const BitString<mword,20,1> SMEP;

  static inline mword readCR8() {
    mword val; asm volatile("mov %%cr8, %0" : "=r"(val) :: "cc"); return val;
  }
  static inline void writeCR8( mword val ) {
    asm volatile("mov %0, %%cr8" :: "r"(val) : "cc");
  }
  static const BitString<mword, 0,4> TPR;        // task priority in CR8

  __finline static inline mword readRSP() {
    mword val; asm volatile("mov %%rsp, %0" : "=r"(val)); return val;
  }
  __finline static inline mword readRBP() {
    mword val; asm volatile("mov %%rbp, %0" : "=r"(val)); return val;
  }

  static inline mword readRFlags() {
    mword x; asm volatile("pushfq\n\tpop %0" : "=r"(x)); return x;
  }
  static inline void writeRFlags( mword x ) {
    asm volatile("push %0\n\tpopfq" :: "r"(x) : "cc");
  }
  namespace RFlags {
    static const BitString<mword, 0,1> CF;
    static const BitString<mword, 2,1> PF;
    static const BitString<mword, 4,1> AF;
    static const BitString<mword, 6,1> ZF;
    static const BitString<mword, 7,1> SF;
    static const BitString<mword, 8,1> TF;
    static const BitString<mword, 9,1> IF;
    static const BitString<mword,10,1> DF;
    static const BitString<mword,11,1> OF;
    static const BitString<mword,12,2> IOPL;
    static const BitString<mword,14,1> NT;
    static const BitString<mword,16,1> RF;
    static const BitString<mword,17,1> VN;
    static const BitString<mword,18,1> AC;
    static const BitString<mword,19,1> VIF;
    static const BitString<mword,20,1> VIP;
    static const BitString<mword,21,1> ID;
  };

  static inline bool CPUID() {
    mword rf = readRFlags();
    rf ^= RFlags::ID();
    writeRFlags(rf);
    mword rf2 = readRFlags();
    return ((rf ^ rf2) & RFlags::ID()) == 0;
  }

  class MachContext {
    mword fs;
    mword gs;
  public:
    MachContext() : fs(0), gs(0) {}
    void save() {
      fs = MSR::read(MSR::FS_BASE);
      gs = MSR::read(MSR::KERNEL_GS_BASE);
    }
    void restore() {
      MSR::write(MSR::KERNEL_GS_BASE, gs);
      MSR::write(MSR::FS_BASE, fs);
    }
  };

  static inline bool interruptsEnabled() { return readRFlags() & RFlags::IF(); }
  static inline void forceTrap() { writeRFlags(readRFlags() | RFlags::TF()); }
};

// TODO: handle unsupported CPUID requests...
class CPUID : public NoObject {
  friend class Processor;

  struct RetCode {
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
  };

  static inline RetCode cpuid(uint32_t ia) {
    RetCode r;
    asm volatile("cpuid" : "=a"(r.a),"=b"(r.b),"=c"(r.c),"=d"(r.d) : "a"(ia));
    return r;
  }
  static inline RetCode cpuid(uint32_t ia, uint32_t ic) {
    RetCode r;
    asm volatile("cpuid" : "=a"(r.a),"=b"(r.b),"=c"(r.c),"=d"(r.d) : "a"(ia),"c"(ic));
    return r;
  }

  static inline uint8_t APICID() { return cpuid(0x00000001).b & bitmask<uint32_t>(24,8) >> 24; }
  static inline bool MWAIT()     { return cpuid(0x00000001).c & bitmask<uint32_t>( 3,1); }
  static inline bool X2APIC()    { return cpuid(0x00000001).c & bitmask<uint32_t>(21,1); }
  static inline bool POPCNT()    { return cpuid(0x00000001).c & bitmask<uint32_t>(23,1); }
  static inline bool TSCD()      { return cpuid(0x00000001).c & bitmask<uint32_t>(24,1); }
  static inline bool MSR()       { return cpuid(0x00000001).d & bitmask<uint32_t>( 5,1); }
  static inline bool APIC()      { return cpuid(0x00000001).d & bitmask<uint32_t>( 9,1); }
  static inline bool ARAT()      { return cpuid(0x00000006).a & bitmask<uint32_t>( 2,1); }
  static inline bool FSGSBASE()  { return cpuid(0x00000007).b & bitmask<uint32_t>( 0,1); }
  static inline bool NX()        { return cpuid(0x80000001).d & bitmask<uint32_t>(20,1); }
  static inline bool SYSCALL()   { return cpuid(0x80000001).d & bitmask<uint32_t>(11,1); }
  static inline bool Page1G()    { return cpuid(0x80000001).d & bitmask<uint32_t>(26,1); }
  void getCacheInfo()                                 __section(".boot.text");
};

// TODO: query PMU capabilities using CPUID (see Intel Vol. 3, Chap 18)
union PerfEvent {
  uint64_t c;
  struct {
    uint64_t EventSelect       : 8;
    uint64_t UnitMask          : 8;
    uint64_t UserMode          : 1;
    uint64_t OSMode            : 1;
    uint64_t EdgeDetect        : 1;
    uint64_t PinControl        : 1;
    uint64_t ApicIntrEnable    : 1;
    uint64_t Reserved0         : 1;
    uint64_t EnableCounters    : 1;
    uint64_t InvertCounterMask : 1;
    uint64_t CounterMask       : 8;
    uint64_t Reserved1         : 32;
  };
  PerfEvent(uint64_t es, uint64_t um) : EventSelect(es), UnitMask(um) {}
} __packed;

// Intel Vol. 3, Section 18.2.3 "Pre-defined Architectural Performance Events"
static const PerfEvent UnhaltedCoreCycles(0x3C,0x00);
static const PerfEvent InstructionRetired(0xC0,0x00);
static const PerfEvent UnhaltedReferenceCycles(0x3C,0x01);
static const PerfEvent LLC_Reference(0x2E,0x4F);
static const PerfEvent LLC_Misses(0x2E,0x41);
static const PerfEvent BranchInstructionRetired(0xC4,0x00);
static const PerfEvent BranchMissRetired(0xC5,0x00);

#endif /* _CPU_h_ */
