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
#include "runtime/Scheduler.h"
#include "runtime/Thread.h"
#include "kernel/AddressSpace.h"
#include "kernel/Clock.h"
#include "kernel/FrameManager.h"
#include "kernel/MemoryManager.h"
#include "kernel/Multiboot.h"
#include "kernel/Process.h"
#include "machine/asmdecl.h"
#include "machine/APIC.h"
#include "machine/Machine.h"
#include "machine/Processor.h"
#include "machine/Paging.h"
#include "devices/Keyboard.h"
#include "devices/PCI.h"
#include "devices/PIT.h"
#include "devices/RTC.h"
#include "devices/Screen.h"
#include "devices/Serial.h"
#include "gdb/Gdb.h"
#include "syscalls.h"

#include "machine/ACPI.h"

#include <list>
#include <map>

// simple direct declarations in lieu of more header files
extern void initCdiDrivers();
extern bool findCdiDriver(const PCIDevice&);
extern void lwip_init_tcpip();
extern void kosMain();

// check various assumptions about data type sizes
static_assert(sizeof(uint64_t) == sizeof(mword), "mword != uint64_t" );
static_assert(sizeof(size_t) == sizeof(mword), "mword != size_t");
static_assert(sizeof(ptr_t) == sizeof(mword), "mword != ptr_t");
static_assert(sizeof(APIC) == 0x400, "sizeof(APIC) != 0x400" );
static_assert(sizeof(APIC) <= pagesize<1>(), "sizeof(APIC) <= pagesize<1>()" );
static_assert(sizeof(InterruptDescriptor) == 2 * sizeof(mword), "sizeof(InterruptDescriptor) != 2 * sizeof(mword)" );
static_assert(sizeof(SegmentDescriptor) == sizeof(mword), "sizeof(SegmentDescriptor) != sizeof(mword)" );

// pointers to 16-bit boot code location from boot.S
extern const char boot16Begin, boot16End;

// symbols set during linking, see linker.ld
extern const char __KernelBoot,  __KernelBootEnd;
extern const char __KernelCtors, __KernelCtorsEnd;
extern const char __KernelCode,  __KernelCodeEnd;
extern const char __KernelRO,    __KernelRO_End;
extern const char __KernelData,  __KernelDataEnd;
extern const char __KernelBss,   __KernelBssEnd;
extern const char __MultibootHdr;

// various helpers during bootstrap
static Processor dummyProc    __section(".boot.data"); // dummy processor object
static vaddr kernelEnd        __section(".boot.data"); // passed from initBSP to bootCleanup
static volatile mword apIndex __section(".boot.data"); // enumerate APs
static paddr pml4addr         __section(".boot.data"); // root of kernel AS

// global frame manager objects
static FrameManager frameManager;

// global device objects
Keyboard keyboard;
static RTC rtc;
static PIT pit;

// interrupt descriptor tables
static const unsigned int maxIDT = 256;
static InterruptDescriptor idt[maxIDT]                __aligned(pagesize<1>());

// CPU information
mword Machine::processorCount = 0;
static Processor* processorTable = nullptr;
static Scheduler* schedulerTable = nullptr;
static mword bspIndex = ~mword(0);
static mword bspApicID = ~mword(0);

// simple IPI test during bootstrap
void (*tipiHandler)(void) = nullptr;
static volatile bool tipiTest __section(".boot.data");
static void tipiReceiver()    __section(".boot.text");
static void tipiReceiver() {
  KERR::out1(" TIPI ");
  tipiTest = true;
}

// IRQ handling
static const int MaxIrqCount = 192;
struct IrqInfo {
  paddr    ioApicAddr;
  uint8_t  ioApicIrq;
  uint8_t  globalIrq;
  uint16_t overrideFlags;
  typedef pair<funcvoid1_t,ptr_t> Handler;
  list<Handler,KernelAllocator<Handler>> handlers;
} irqTable[MaxIrqCount];
static Bitmap<MaxIrqCount> irqMask;     // IRQ bitmap
static Semaphore asyncIrqSem;

// init routine for APs: on boot stack and using identity paging
void Machine::initAP(mword idx) {
  KASSERT1(idx == apIndex, idx);
  processorTable[apIndex].init(pml4addr, idt, sizeof(idt), initAP2);
}

// on proper stack, processor initialized
void Machine::initAP2() {
  startGdbCpu(apIndex);             // tell GDB that this CPU is running
  apIndex = bspIndex;               // sync with BSP
  DBG::outl(DBG::Boot, "Enabling AP interrupts...");
  LocalProcessor::initInterrupts(false); // enable interrupts (off boot stack)
  DBG::outl(DBG::Boot, "Finishing AP boot thread...");
  LocalProcessor::getScheduler()->terminate(); // idle thread takes over
}

// init routine for BSP, on boot stack and identity paging
void Machine::initBSP(mword magic, vaddr mbiAddr, mword idx) {

  // initialize bss
  memset((char*)&__KernelBss, 0, &__KernelBssEnd - &__KernelBss);

  // static memory to back temporary heap during bootstrap
  static buf_t bootHeap[bootHeapSize]                  __section(".boot.data");

  // create temporary boot heap, so that ctors & RegionMap can use malloc/free
  MemoryManager::init0( (vaddr)bootHeap, sizeof(bootHeap) );

  // set up dummy processor for lock counter -> output/malloc uses spinlock
  dummyProc.install();

  // initialize multiboot & debugging: no debug options before this point!
  vaddr mbiEnd = Multiboot::init(magic, mbiAddr);
  // determine end addresses of kernel overall (including multiboot & modules)
  kernelEnd = align_up(mbiEnd, kernelps) + kernelBase;

  // initialize basic devices -> needed for printing
  if (!Screen::init(kernelBase)) Reboot();        // no identity mapping later
  DebugDevice::init();                            // init qemu debug device
  SerialDevice::init(DBG::test(DBG::GDBEnable));  // must come after multiboot/debug init

  // call global constructors: can use temporary dynamic memory
  // %rbx is callee-saved; explicitly protect all caller-saved registers
  for ( const char* x = &__KernelCtors; x != &__KernelCtorsEnd; x += sizeof(char*)) {
    asm volatile( "call *(%0)" : : "b"(x) : "memory", "cc", "rax", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "r11" );
  }

  // can print now: install global IDT entries - the earlier the better...
  setupIDTable();

  // check processor, set paging bits, print results
  dummyProc.check(true);

  // print all MBI info, re-initialize debugging to print debug options
  Multiboot::init2();

  // double-check that BSP received index 0
  KASSERT1(idx == 0, idx);

  // print boot memory information
  DBG::outl(DBG::Boot, "*********** MEM INFO ***********");
  DBG::outl(DBG::Boot, "Boot16:    ", FmtHex(&boot16Begin),  " - ", FmtHex(&boot16End), " -> ", FmtHex(BOOTAP16));
  DBG::outl(DBG::Boot, "PageTbls:  ", FmtHex(Paging::ptprefix<1>()), " - ", FmtHex(Paging::ptend()));
  DBG::outl(DBG::Boot, "Kernel:    ", FmtHex(kernelBase),    " - ", FmtHex(kernelEnd));
  DBG::outl(DBG::Boot, "Boot Seg:  ", FmtHex(&__KernelBoot), " - ", FmtHex(&__KernelBootEnd));
  DBG::outl(DBG::Boot, "Code Seg:  ", FmtHex(&__KernelCode), " - ", FmtHex(&__KernelCodeEnd));
  DBG::outl(DBG::Boot, "RO Seg:    ", FmtHex(&__KernelRO),   " - ", FmtHex(&__KernelRO_End));
  DBG::outl(DBG::Boot, "Data Seg:  ", FmtHex(&__KernelData), " - ", FmtHex(&__KernelDataEnd));
  DBG::outl(DBG::Boot, "Bss Seg:   ", FmtHex(&__KernelBss),  " - ", FmtHex(&__KernelBssEnd));
  DBG::outl(DBG::Boot, "MB/MBI:    ", FmtHex(&__MultibootHdr + kernelBase), " / ", FmtHex(mbiAddr + kernelBase));
  DBG::outl(DBG::Boot, "Vid/APIC:  ", FmtHex(videoAddr), " / ", FmtHex(apicAddr));

  DBG::outl(DBG::Boot, "*********** BOOTING ************");

  // collect available and used memory
  RegionSet<Region<paddr>> memtmp, mem;
  Multiboot::getMemory( memtmp );
  for (auto it = memtmp.begin(); it != memtmp.end(); ) {
    mem.insert( Region<paddr>(align_up(it->start, pagesize<1>()), align_down(it->end, pagesize<1>())) );
    it = memtmp.erase(it);
  }
  bool check = mem.remove( Region<paddr>(vaddr(&__KernelBoot) - kernelBase, kernelEnd - kernelBase) );
  KASSERT0(check);
  size_t boot16Size = align_up( size_t(&boot16End - &boot16Begin), pagesize<1>());
  check = mem.remove( Region<paddr>(BOOTAP16, BOOTAP16 + boot16Size) );
  KASSERT0(check);
  KASSERT0(!mem.empty());

  // copy boot code segment -> easy with identiy mapping
  memcpy( bufptr_t(BOOTAP16), &boot16Begin, boot16Size );

  // bootstrap paging -> afterwards: identity mapping is gone
  pml4addr = Paging::bootstrap(kernelEnd);
  Multiboot::remap(kernelBase);

  // allocate and map memory for frame manager <- need paging bootstrapped
  paddr endphysmem = (--mem.end())->end;
  size_t fmStart = kerneltop - FrameManager::getSize( endphysmem );
  vaddr initStart = kerneltop;
  while ( initStart + bootHeapSize > fmStart ) { // ensure leftover for kernel memory
    paddr start = mem.retrieve(kernelps);
    KASSERT0(start != topaddr);
    initStart -= kernelps;
    // frameManager useless at this point, but not needed - page tables bootstrap
    bool check = Paging::map<kernelpl,true>(initStart, start, Paging::KernelData, frameManager);
    KASSERT1(check, initStart);
  }

  // re-init kernel heap, discard boot heap <- use leftover from FM init
  MemoryManager::reinit(initStart, fmStart - initStart);

  // rerun all global constructors: can now use proper dynamic memory
  // %rbx is callee-saved; explicitly protect all caller-saved registers
  DBG::out1(DBG::Boot, "constructors:");
  for ( const char* x = &__KernelCtors; x != &__KernelCtorsEnd; x += sizeof(char*)) {
    DBG::out1(DBG::Boot, ' ', FmtHex(x));
    asm volatile( "call *(%0)" : : "b"(x) : "memory", "cc", "rax", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "r11" );
  }
  DBG::outl(DBG::Boot);

  // initialize frame manager <- need dynamic memory for internal container
  frameManager.init( (bufptr_t)fmStart, endphysmem );
  for ( auto it = mem.begin(); it != mem.end(); ++it ) {
    frameManager.releaseRegion( it->start, it->end - it->start );
  }
  DBG::outl(DBG::Boot, "FM/init: ", frameManager);

  // set up top-level kernel page tables (for later AS cloning) <- need FM
  Paging::bootstrap2(frameManager);
  DBG::outl(DBG::Boot, "Paging & FM initialized");

  // set frame manager in dummy processor -> needed for page mappings
  dummyProc.frameManager = &frameManager;

  // init kernel address space <- need dynamic memory for internal container
  kernelSpace.initKernel(kernelbot, initStart, pml4addr);
  DBG::outl(DBG::Boot, "AS/init: ", kernelSpace);

  // parse ACPI tables: find/initialize CPUs, APICs, IOAPICs
  map<uint32_t,uint32_t> apicMap;
  map<uint32_t,paddr> ioApicMap;
  map<uint8_t,pair<uint32_t,uint16_t>> ioOverrideMap;
  paddr rsdp = Multiboot::getRSDP() - kernelBase;
  paddr apicPhysAddr = initACPI(rsdp, apicMap, ioApicMap, ioOverrideMap);

  // process IOAPIC/IRQ information -> mask all IOAPIC interrupts for now
  for (const pair<uint32_t,paddr>&iop : ioApicMap) {
    kernelSpace.mapDirect<1>(iop.second, ioApicAddr, pagesize<1>(), Paging::MMapIO);
    mword rdr = MappedIOAPIC()->getRedirects() + 1;
    for (mword x = 0; x < rdr; x += 1 ) {
      MappedIOAPIC()->maskIRQ(x);
      mword irqnum = iop.first + x;
      KASSERT1(irqnum < MaxIrqCount, irqnum);
      irqTable[irqnum].ioApicAddr = iop.second;
      irqTable[irqnum].ioApicIrq  = x;
      if (ioOverrideMap.count(irqnum) > 0) {
        pair<uint32_t,uint16_t>& override = ioOverrideMap.at(irqnum);
        irqTable[irqnum].globalIrq     = override.first;
        irqTable[irqnum].overrideFlags = override.second;
      } else {
        irqTable[irqnum].globalIrq     = irqnum;
      }
    }
    kernelSpace.unmapDirect<1>(ioApicAddr, pagesize<1>());
  }

  // NOTE: could use broadcast and ticket lock sequencing in Machine::initBSP2()
  // determine processorCount and create processorTable
  KASSERT0(apicMap.size());
  processorCount = apicMap.size();
  processorTable = knewN<Processor>(processorCount);
  schedulerTable = knewN<Scheduler>(processorCount);
  mword coreIdx = 0;
  for (const pair<uint32_t,uint32_t>& ap : apicMap) {
    DBG::outl( DBG::Scheduler, "Scheduler ", coreIdx, " at ", FmtHex(schedulerTable + coreIdx));
    schedulerTable[coreIdx].setPartner(schedulerTable[(coreIdx + 1) % processorCount]);
    processorTable[coreIdx].setup(kernelSpace, kernelSpace.initProcessor(),
      schedulerTable[coreIdx], frameManager, coreIdx, ap.second, ap.first);
    coreIdx += 1;
  }

  // map APIC page, use APIC ID to determine bspIndex
  kernelSpace.mapDirect<1>(apicPhysAddr, apicAddr, pagesize<1>(), Paging::MMapIO);
  bspApicID = MappedAPIC()->getID();
  for (mword i = 0; i < processorCount; i += 1) {
    if (processorTable[i].apicID == bspApicID) {
      bspIndex = i;
      break;
    }
  }
  DBG::outl(DBG::Boot, "CPUs: ", processorCount, '/', bspIndex, '/', bspApicID);
  KASSERT0(bspIndex != ~mword(0));

  // init and install processor object (need bspIndex) -> start main/idle loop
  processorTable[bspIndex].init(pml4addr, idt, sizeof(idt), bootMain);
}

// on proper stack, processor initialized
void Machine::initBSP2() {
  DBG::outl(DBG::Boot, "********** NEW STACK ***********");
  DBG::outl(DBG::Boot, "BSP: ", LocalProcessor::getIndex(), '/', LocalProcessor::getSystemID(), '/', LocalProcessor::getApicID());

  // initialize GDB object -> after ACPI init & IDT installed
  initGdb(bspIndex);

  DBG::outl(DBG::Boot, "Initializing basic devices...");
  // init RTC timer; used for preemption & sleeping
  rtc.init();
  // init PIT timer; used for waiting
  pit.init();
  // init keyboard; must init RTC first (HW req)?
  keyboard.init();

  DBG::outl(DBG::Boot, "********** MULTI CORE **********");

  // enable interrupts (off boot stack); needed for timer waiting
  DBG::outl(DBG::Boot, "Enabling BSP interrupts...");
  LocalProcessor::initInterrupts(true);

  // send test IPI to self <- reception needs interrupts enabled
  tipiTest = false;
  tipiHandler = tipiReceiver;
  sendIPI(bspIndex, APIC::TestIPI);
  while (!tipiTest) CPU::Pause();

  // NOTE: could use broadcast and ticket lock sequencing
  // start up APs one by one (on boot stack): APs go into long mode and halt
  StdOut.print("AP init (", FmtHex(BOOTAP16 / 0x1000), "):");
  for (mword idx = 0; idx < processorCount; idx += 1) {
    if (idx != bspIndex) {
      apIndex = idx;
      for (;;) {
        mword ai = processorTable[idx].apicID;
        StdOut.print(' ', ai);
        MappedAPIC()->sendInitIPI(ai);
        StdOut.print('I');
        Clock::wait(100);                // wait for HW init
        MappedAPIC()->sendInitDeassertIPI(ai);
        StdOut.print('D');
        Clock::wait(100);                // wait for HW init
        for (int i = 0; i < 10; i += 1) {
          MappedAPIC()->sendStartupIPI(ai, BOOTAP16 / 0x1000);
          StdOut.print('S');
          for (int j = 0; j < 1000; j += 1) {
            Clock::wait(1);
            if (apIndex != idx) goto apDone;
          }
        }
      }
apDone:
      StdOut.print('|', idx);
    }
  }
  StdOut.print(kendl);

  DBG::outl(DBG::Boot, "Building kernel filesystem...");
  // initialize kernel file system with boot modules
  Multiboot::readModules(kernelBase);

  // more info from ACPI; could find IOAPIC interrupt pins for PCI devices
  initACPI2(); // needs "current thread"

  // initialize CDI drivers
  initCdiDrivers();
  DBG::outl(DBG::Boot, "CDI drivers initialized.");

  // probe for PCI devices
  list<PCIDevice> pciDevList;
  PCI::sanityCheck();
  PCI::checkAllBuses(pciDevList);

  // initialize TCP/IP stack - needed to start network devices
  DBG::outl(DBG::Boot, "Starting network subsystem...");
  lwip_init_tcpip();

  DBG::outl(DBG::Boot, "Starting CDI devices...");
  // find and install CDI drivers for PCI devices - need interrupts for sleep
  for (const PCIDevice& pd : pciDevList) findCdiDriver(pd);

  // start irq thread after cdi init -> avoid interference from device irqs
  DBG::outl(DBG::Boot, "Creating IRQ thread...");
  Thread::create()->setPriority(topPriority)->setAffinity(processorTable[0].scheduler)->start((ptr_t)asyncIrqLoop);
}

void Machine::bootCleanup() {
  DBG::outl(DBG::Boot, "********* MEMORY CLEANUP *********");

  // free AP boot code
  frameManager.releaseRegion(BOOTAP16, pagesize<1>());
  DBG::outl(DBG::Boot, "FM/free16:", frameManager);

#if 0
  // unmap & free multiboot memory
  for ( vaddr x = kernelBase + vaddr(&__MultibootHdr); x < kernelEnd; x += kernelps ) {
    Paging::unmap<kernelpl>(x);
    frameManager.releaseRegion(x - kernelBase, kernelps );
  }
  DBG::outl(DBG::Boot, "FM/mbi:", frameManager);
#endif

  // remap screen page high, before low memory paging entries disappear
  kernelSpace.mapDirect<1>(Paging::vtop(Screen::getAddress()), videoAddr, pagesize<1>(), Paging::MMapIO);
  Screen::setAddress(videoAddr);

  // unmap & free kernel boot memory
  Paging::unmap<kernelpl>(kernelBase);
  frameManager.releaseRegion( vaddr(&__KernelBoot) - kernelBase, kernelBase + kernelps - vaddr(&__KernelBoot) );
  for ( vaddr x = kernelBase + kernelps; x < vaddr(&__KernelCode); x += kernelps / 2 ) {
    Paging::unmap<kernelpl>(x);
    frameManager.releaseRegion(x - kernelBase, kernelps);
  }

  // VM addresses from above are not reused, thus no TLB invalidation needed
  DBG::outl(DBG::Boot, "FM/boot:", frameManager);
  DBG::outl(DBG::Boot, "AS/boot: ", kernelSpace);
}

void Machine::bootMain() {
  Machine::initBSP2();
  Machine::bootCleanup();
  Thread::create()->start((ptr_t)kosMain);
  LocalProcessor::getScheduler()->terminate(); // explicitly terminate boot thread
}

void Machine::setAffinity(Thread& t, mword idx) {
  KASSERT1(idx < processorCount, idx);
  t.setAffinity(processorTable[idx].scheduler);
}

void Machine::sendIPI(mword idx, uint8_t vec) {
  MappedAPIC()->sendIPI(processorTable[idx].apicID, vec);
}

void Machine::sendWakeIPI(Scheduler* scheduler) {
  MappedAPIC()->sendIPI(processorTable[scheduler - schedulerTable].apicID, APIC::WakeIPI);
}

void Machine::rrPreemptIPI(mword tick) {
  sendIPI(processorTable[tick % processorCount].apicID, APIC::PreemptIPI);
}

/*********************** IRQ / Exception Handling Code ***********************/

void Machine::asyncIrqLoop() {
  for (;;) {
    asyncIrqSem.P();
    for (;;) {
      mword idx = irqMask.findset();
      if slowpath(idx >= MaxIrqCount) break;
#if TESTING_REPORT_INTERRUPTS
      StdErr.out1(" AH:", FmtHex(idx));
#endif
      irqMask.clear<true>(idx);
      for (IrqInfo::Handler f : irqTable[idx].handlers) f.first(f.second);
    }
  }
};

void Machine::mapIrq(mword irq, mword vector) {
  static SpinLock ioapicLock;
  mword irqmod = irqTable[irq].globalIrq;
  DBG::outl(DBG::Basic, "IRQ mapping: ", FmtHex(irq), '/', FmtHex(irqTable[irqmod].ioApicIrq), " -> ", FmtHex(vector));
  kernelSpace.mapDirect<1>(irqTable[irqmod].ioApicAddr, ioApicAddr, pagesize<1>(), AddressSpace::MMapIO);
  if (vector) {
    ScopedLock<> sl(ioapicLock);
    // TODO: program IOAPIC with polarity/trigger (ACPI flags), if necessary
    MappedIOAPIC()->mapIRQ( irqTable[irqmod].ioApicIrq, vector, bspApicID );
  } else {
    ScopedLock<> sl(ioapicLock);
    MappedIOAPIC()->maskIRQ( irqTable[irqmod].ioApicIrq );
  }
  kernelSpace.unmapDirect<1>(ioApicAddr, pagesize<1>());
}

void Machine::registerIrqSync(mword irq, mword vector) {
  ScopedLock<LocalProcessor> sl;
  KASSERT0(irqTable[irq].handlers.empty());
  mapIrq(irq, vector);
}

void Machine::registerIrqAsync(mword irq, funcvoid1_t handler, ptr_t ctx) {
  mword vector = irq + 0x20;
  DBG::outl(DBG::Basic, "register async IRQ handler: ", FmtHex(ptr_t(handler)), " for irq/vector ", FmtHex(irq), '/', FmtHex(vector));
  ScopedLock<LocalProcessor> sl;
  if (irqTable[irq].handlers.empty()) mapIrq(irq, vector);
  irqTable[irq].handlers.push_back( {handler, ctx} );
}

void Machine::deregisterIrqAsync(mword irq, funcvoid1_t handler) {
  DBG::outl(DBG::Basic, "deregister async IRQ handler: ", FmtHex(ptr_t(handler)), " for irq ", FmtHex(irq));
  ScopedLock<LocalProcessor> sl;
  auto it = irqTable[irq].handlers.begin();
  for ( ; it != irqTable[irq].handlers.end(); ++it ) {
    if (it->first == handler) {
      irqTable[irq].handlers.erase(it);
  break;
    }
  }
  if (irqTable[irq].handlers.empty()) mapIrq(irq, 0);
}

void Machine::setupIDT(uint32_t number, paddr address, uint32_t ist) {
  KASSERT1(number < maxIDT, number);
  idt[number].Offset00 = (address & 0x000000000000FFFF);
  idt[number].Offset16 = (address & 0x00000000FFFF0000) >> 16;
  idt[number].Offset32 = (address & 0xFFFFFFFF00000000) >> 32;
  idt[number].SegmentSelector = Processor::kernCS * sizeof(SegmentDescriptor);
  idt[number].IST = ist;
  idt[number].Type = 0x0E; // 64-bit interrupt gate (trap gate does not disable interrupts)
  idt[number].P = 1;
}

void Machine::setupIDTable() {
  KASSERT0(!CPU::interruptsEnabled());

  for (size_t i = 0; i < MaxIrqCount; i += 1) {
    irqTable[i].ioApicAddr    = 0;
    irqTable[i].ioApicIrq     = 0;
    irqTable[i].globalIrq     = i; 
    irqTable[i].overrideFlags = 0;
  }

  memset(idt, 0, sizeof(idt));

  // first 32 vectors are architectural or reserved
  setupIDT(0x00, (vaddr)&isr_wrapper_0x00);
  setupIDT(0x01, (vaddr)&isr_wrapper_0x01);
  setupIDT(0x02, (vaddr)&isr_wrapper_0x02, Processor::nmiIST);
  setupIDT(0x03, (vaddr)&isr_wrapper_0x03);
  setupIDT(0x04, (vaddr)&isr_wrapper_0x04);
  setupIDT(0x05, (vaddr)&isr_wrapper_0x05);
  setupIDT(0x06, (vaddr)&isr_wrapper_0x06);
  setupIDT(0x07, (vaddr)&isr_wrapper_0x07);
  setupIDT(0x08, (vaddr)&isr_wrapper_0x08, Processor::dbfIST); // double fault
  setupIDT(0x09, (vaddr)&isr_wrapper_0x09);
  setupIDT(0x0a, (vaddr)&isr_wrapper_0x0a);
  setupIDT(0x0b, (vaddr)&isr_wrapper_0x0b);
  setupIDT(0x0c, (vaddr)&isr_wrapper_0x0c, Processor::stfIST); // stack fault
  setupIDT(0x0d, (vaddr)&isr_wrapper_0x0d); // general protection fault
  setupIDT(0x0e, (vaddr)&isr_wrapper_0x0e, Processor::pgfIST); // page fault
  setupIDT(0x0f, (vaddr)&isr_wrapper_0x0f);
  setupIDT(0x10, (vaddr)&isr_wrapper_0x10);
  setupIDT(0x11, (vaddr)&isr_wrapper_0x11);
  setupIDT(0x12, (vaddr)&isr_wrapper_0x12);
  setupIDT(0x13, (vaddr)&isr_wrapper_0x13);
  setupIDT(0x14, (vaddr)&isr_wrapper_0x14);
  setupIDT(0x15, (vaddr)&isr_wrapper_0x15);
  setupIDT(0x16, (vaddr)&isr_wrapper_0x16);
  setupIDT(0x17, (vaddr)&isr_wrapper_0x17);
  setupIDT(0x18, (vaddr)&isr_wrapper_0x18);
  setupIDT(0x19, (vaddr)&isr_wrapper_0x19);
  setupIDT(0x1a, (vaddr)&isr_wrapper_0x1a);
  setupIDT(0x1b, (vaddr)&isr_wrapper_0x1b);
  setupIDT(0x1c, (vaddr)&isr_wrapper_0x1c);
  setupIDT(0x1d, (vaddr)&isr_wrapper_0x1d);
  setupIDT(0x1e, (vaddr)&isr_wrapper_0x1e);
  setupIDT(0x1f, (vaddr)&isr_wrapper_0x1f);

  // remaining vectors are programmable via IO-APIC
  setupIDT(0x20, (vaddr)&isr_wrapper_0x20);
  setupIDT(0x21, (vaddr)&isr_wrapper_0x21);
  setupIDT(0x22, (vaddr)&isr_wrapper_0x22);
  setupIDT(0x23, (vaddr)&isr_wrapper_0x23);
  setupIDT(0x24, (vaddr)&isr_wrapper_0x24);
  setupIDT(0x25, (vaddr)&isr_wrapper_0x25);
  setupIDT(0x26, (vaddr)&isr_wrapper_0x26);
  setupIDT(0x27, (vaddr)&isr_wrapper_0x27);
  setupIDT(0x28, (vaddr)&isr_wrapper_0x28);
  setupIDT(0x29, (vaddr)&isr_wrapper_0x29);
  setupIDT(0x2a, (vaddr)&isr_wrapper_0x2a);
  setupIDT(0x2b, (vaddr)&isr_wrapper_0x2b);
  setupIDT(0x2c, (vaddr)&isr_wrapper_0x2c);
  setupIDT(0x2d, (vaddr)&isr_wrapper_0x2d);
  setupIDT(0x2e, (vaddr)&isr_wrapper_0x2e);
  setupIDT(0x2f, (vaddr)&isr_wrapper_0x2f);
  setupIDT(0x30, (vaddr)&isr_wrapper_0x30);
  setupIDT(0x31, (vaddr)&isr_wrapper_0x31);
  setupIDT(0x32, (vaddr)&isr_wrapper_0x32);
  setupIDT(0x33, (vaddr)&isr_wrapper_0x33);
  setupIDT(0x34, (vaddr)&isr_wrapper_0x34);
  setupIDT(0x35, (vaddr)&isr_wrapper_0x35);
  setupIDT(0x36, (vaddr)&isr_wrapper_0x36);
  setupIDT(0x37, (vaddr)&isr_wrapper_0x37);
  setupIDT(0x38, (vaddr)&isr_wrapper_0x38);
  setupIDT(0x39, (vaddr)&isr_wrapper_0x39);
  setupIDT(0x3a, (vaddr)&isr_wrapper_0x3a);
  setupIDT(0x3b, (vaddr)&isr_wrapper_0x3b);
  setupIDT(0x3c, (vaddr)&isr_wrapper_0x3c);
  setupIDT(0x3d, (vaddr)&isr_wrapper_0x3d);
  setupIDT(0x3e, (vaddr)&isr_wrapper_0x3e);
  setupIDT(0x3f, (vaddr)&isr_wrapper_0x3f);
  setupIDT(0x40, (vaddr)&isr_wrapper_0x40);
  setupIDT(0x41, (vaddr)&isr_wrapper_0x41);
  setupIDT(0x42, (vaddr)&isr_wrapper_0x42);
  setupIDT(0x43, (vaddr)&isr_wrapper_0x43);
  setupIDT(0x44, (vaddr)&isr_wrapper_0x44);
  setupIDT(0x45, (vaddr)&isr_wrapper_0x45);
  setupIDT(0x46, (vaddr)&isr_wrapper_0x46);
  setupIDT(0x47, (vaddr)&isr_wrapper_0x47);
  setupIDT(0x48, (vaddr)&isr_wrapper_0x48);
  setupIDT(0x49, (vaddr)&isr_wrapper_0x49);
  setupIDT(0x4a, (vaddr)&isr_wrapper_0x4a);
  setupIDT(0x4b, (vaddr)&isr_wrapper_0x4b);
  setupIDT(0x4c, (vaddr)&isr_wrapper_0x4c);
  setupIDT(0x4d, (vaddr)&isr_wrapper_0x4d);
  setupIDT(0x4e, (vaddr)&isr_wrapper_0x4e);
  setupIDT(0x4f, (vaddr)&isr_wrapper_0x4f);
  setupIDT(0x50, (vaddr)&isr_wrapper_0x50);
  setupIDT(0x51, (vaddr)&isr_wrapper_0x51);
  setupIDT(0x52, (vaddr)&isr_wrapper_0x52);
  setupIDT(0x53, (vaddr)&isr_wrapper_0x53);
  setupIDT(0x54, (vaddr)&isr_wrapper_0x54);
  setupIDT(0x55, (vaddr)&isr_wrapper_0x55);
  setupIDT(0x56, (vaddr)&isr_wrapper_0x56);
  setupIDT(0x57, (vaddr)&isr_wrapper_0x57);
  setupIDT(0x58, (vaddr)&isr_wrapper_0x58);
  setupIDT(0x59, (vaddr)&isr_wrapper_0x59);
  setupIDT(0x5a, (vaddr)&isr_wrapper_0x5a);
  setupIDT(0x5b, (vaddr)&isr_wrapper_0x5b);
  setupIDT(0x5c, (vaddr)&isr_wrapper_0x5c);
  setupIDT(0x5d, (vaddr)&isr_wrapper_0x5d);
  setupIDT(0x5e, (vaddr)&isr_wrapper_0x5e);
  setupIDT(0x5f, (vaddr)&isr_wrapper_0x5f);
  setupIDT(0x60, (vaddr)&isr_wrapper_0x60);
  setupIDT(0x61, (vaddr)&isr_wrapper_0x61);
  setupIDT(0x62, (vaddr)&isr_wrapper_0x62);
  setupIDT(0x63, (vaddr)&isr_wrapper_0x63);
  setupIDT(0x64, (vaddr)&isr_wrapper_0x64);
  setupIDT(0x65, (vaddr)&isr_wrapper_0x65);
  setupIDT(0x66, (vaddr)&isr_wrapper_0x66);
  setupIDT(0x67, (vaddr)&isr_wrapper_0x67);
  setupIDT(0x68, (vaddr)&isr_wrapper_0x68);
  setupIDT(0x69, (vaddr)&isr_wrapper_0x69);
  setupIDT(0x6a, (vaddr)&isr_wrapper_0x6a);
  setupIDT(0x6b, (vaddr)&isr_wrapper_0x6b);
  setupIDT(0x6c, (vaddr)&isr_wrapper_0x6c);
  setupIDT(0x6d, (vaddr)&isr_wrapper_0x6d);
  setupIDT(0x6e, (vaddr)&isr_wrapper_0x6e);
  setupIDT(0x6f, (vaddr)&isr_wrapper_0x6f);
  setupIDT(0x70, (vaddr)&isr_wrapper_0x70);
  setupIDT(0x71, (vaddr)&isr_wrapper_0x71);
  setupIDT(0x72, (vaddr)&isr_wrapper_0x72);
  setupIDT(0x73, (vaddr)&isr_wrapper_0x73);
  setupIDT(0x74, (vaddr)&isr_wrapper_0x74);
  setupIDT(0x75, (vaddr)&isr_wrapper_0x75);
  setupIDT(0x76, (vaddr)&isr_wrapper_0x76);
  setupIDT(0x77, (vaddr)&isr_wrapper_0x77);
  setupIDT(0x78, (vaddr)&isr_wrapper_0x78);
  setupIDT(0x79, (vaddr)&isr_wrapper_0x79);
  setupIDT(0x7a, (vaddr)&isr_wrapper_0x7a);
  setupIDT(0x7b, (vaddr)&isr_wrapper_0x7b);
  setupIDT(0x7c, (vaddr)&isr_wrapper_0x7c);
  setupIDT(0x7d, (vaddr)&isr_wrapper_0x7d);
  setupIDT(0x7e, (vaddr)&isr_wrapper_0x7e);
  setupIDT(0x7f, (vaddr)&isr_wrapper_0x7f);
  setupIDT(0x80, (vaddr)&isr_wrapper_0x80);
  setupIDT(0x81, (vaddr)&isr_wrapper_0x81);
  setupIDT(0x82, (vaddr)&isr_wrapper_0x82);
  setupIDT(0x83, (vaddr)&isr_wrapper_0x83);
  setupIDT(0x84, (vaddr)&isr_wrapper_0x84);
  setupIDT(0x85, (vaddr)&isr_wrapper_0x85);
  setupIDT(0x86, (vaddr)&isr_wrapper_0x86);
  setupIDT(0x87, (vaddr)&isr_wrapper_0x87);
  setupIDT(0x88, (vaddr)&isr_wrapper_0x88);
  setupIDT(0x89, (vaddr)&isr_wrapper_0x89);
  setupIDT(0x8a, (vaddr)&isr_wrapper_0x8a);
  setupIDT(0x8b, (vaddr)&isr_wrapper_0x8b);
  setupIDT(0x8c, (vaddr)&isr_wrapper_0x8c);
  setupIDT(0x8d, (vaddr)&isr_wrapper_0x8d);
  setupIDT(0x8e, (vaddr)&isr_wrapper_0x8e);
  setupIDT(0x8f, (vaddr)&isr_wrapper_0x8f);
  setupIDT(0x90, (vaddr)&isr_wrapper_0x90);
  setupIDT(0x91, (vaddr)&isr_wrapper_0x91);
  setupIDT(0x92, (vaddr)&isr_wrapper_0x92);
  setupIDT(0x93, (vaddr)&isr_wrapper_0x93);
  setupIDT(0x94, (vaddr)&isr_wrapper_0x94);
  setupIDT(0x95, (vaddr)&isr_wrapper_0x95);
  setupIDT(0x96, (vaddr)&isr_wrapper_0x96);
  setupIDT(0x97, (vaddr)&isr_wrapper_0x97);
  setupIDT(0x98, (vaddr)&isr_wrapper_0x98);
  setupIDT(0x99, (vaddr)&isr_wrapper_0x99);
  setupIDT(0x9a, (vaddr)&isr_wrapper_0x9a);
  setupIDT(0x9b, (vaddr)&isr_wrapper_0x9b);
  setupIDT(0x9c, (vaddr)&isr_wrapper_0x9c);
  setupIDT(0x9d, (vaddr)&isr_wrapper_0x9d);
  setupIDT(0x9e, (vaddr)&isr_wrapper_0x9e);
  setupIDT(0x9f, (vaddr)&isr_wrapper_0x9f);
  setupIDT(0xa0, (vaddr)&isr_wrapper_0xa0);
  setupIDT(0xa1, (vaddr)&isr_wrapper_0xa1);
  setupIDT(0xa2, (vaddr)&isr_wrapper_0xa2);
  setupIDT(0xa3, (vaddr)&isr_wrapper_0xa3);
  setupIDT(0xa4, (vaddr)&isr_wrapper_0xa4);
  setupIDT(0xa5, (vaddr)&isr_wrapper_0xa5);
  setupIDT(0xa6, (vaddr)&isr_wrapper_0xa6);
  setupIDT(0xa7, (vaddr)&isr_wrapper_0xa7);
  setupIDT(0xa8, (vaddr)&isr_wrapper_0xa8);
  setupIDT(0xa9, (vaddr)&isr_wrapper_0xa9);
  setupIDT(0xaa, (vaddr)&isr_wrapper_0xaa);
  setupIDT(0xab, (vaddr)&isr_wrapper_0xab);
  setupIDT(0xac, (vaddr)&isr_wrapper_0xac);
  setupIDT(0xad, (vaddr)&isr_wrapper_0xad);
  setupIDT(0xae, (vaddr)&isr_wrapper_0xae);
  setupIDT(0xaf, (vaddr)&isr_wrapper_0xaf);
  setupIDT(0xb0, (vaddr)&isr_wrapper_0xb0);
  setupIDT(0xb1, (vaddr)&isr_wrapper_0xb1);
  setupIDT(0xb2, (vaddr)&isr_wrapper_0xb2);
  setupIDT(0xb3, (vaddr)&isr_wrapper_0xb3);
  setupIDT(0xb4, (vaddr)&isr_wrapper_0xb4);
  setupIDT(0xb5, (vaddr)&isr_wrapper_0xb5);
  setupIDT(0xb6, (vaddr)&isr_wrapper_0xb6);
  setupIDT(0xb7, (vaddr)&isr_wrapper_0xb7);
  setupIDT(0xb8, (vaddr)&isr_wrapper_0xb8);
  setupIDT(0xb9, (vaddr)&isr_wrapper_0xb9);
  setupIDT(0xba, (vaddr)&isr_wrapper_0xba);
  setupIDT(0xbb, (vaddr)&isr_wrapper_0xbb);
  setupIDT(0xbc, (vaddr)&isr_wrapper_0xbc);
  setupIDT(0xbd, (vaddr)&isr_wrapper_0xbd);
  setupIDT(0xbe, (vaddr)&isr_wrapper_0xbe);
  setupIDT(0xbf, (vaddr)&isr_wrapper_0xbf);
  setupIDT(0xc0, (vaddr)&isr_wrapper_0xc0);
  setupIDT(0xc1, (vaddr)&isr_wrapper_0xc1);
  setupIDT(0xc2, (vaddr)&isr_wrapper_0xc2);
  setupIDT(0xc3, (vaddr)&isr_wrapper_0xc3);
  setupIDT(0xc4, (vaddr)&isr_wrapper_0xc4);
  setupIDT(0xc5, (vaddr)&isr_wrapper_0xc5);
  setupIDT(0xc6, (vaddr)&isr_wrapper_0xc6);
  setupIDT(0xc7, (vaddr)&isr_wrapper_0xc7);
  setupIDT(0xc8, (vaddr)&isr_wrapper_0xc8);
  setupIDT(0xc9, (vaddr)&isr_wrapper_0xc9);
  setupIDT(0xca, (vaddr)&isr_wrapper_0xca);
  setupIDT(0xcb, (vaddr)&isr_wrapper_0xcb);
  setupIDT(0xcc, (vaddr)&isr_wrapper_0xcc);
  setupIDT(0xcd, (vaddr)&isr_wrapper_0xcd);
  setupIDT(0xce, (vaddr)&isr_wrapper_0xce);
  setupIDT(0xcf, (vaddr)&isr_wrapper_0xcf);
  setupIDT(0xd0, (vaddr)&isr_wrapper_0xd0);
  setupIDT(0xd1, (vaddr)&isr_wrapper_0xd1);
  setupIDT(0xd2, (vaddr)&isr_wrapper_0xd2);
  setupIDT(0xd3, (vaddr)&isr_wrapper_0xd3);
  setupIDT(0xd4, (vaddr)&isr_wrapper_0xd4);
  setupIDT(0xd5, (vaddr)&isr_wrapper_0xd5);
  setupIDT(0xd6, (vaddr)&isr_wrapper_0xd6);
  setupIDT(0xd7, (vaddr)&isr_wrapper_0xd7);
  setupIDT(0xd8, (vaddr)&isr_wrapper_0xd8);
  setupIDT(0xd9, (vaddr)&isr_wrapper_0xd9);
  setupIDT(0xda, (vaddr)&isr_wrapper_0xda);
  setupIDT(0xdb, (vaddr)&isr_wrapper_0xdb);
  setupIDT(0xdc, (vaddr)&isr_wrapper_0xdc);
  setupIDT(0xdd, (vaddr)&isr_wrapper_0xdd);
  setupIDT(0xde, (vaddr)&isr_wrapper_0xde);
  setupIDT(0xdf, (vaddr)&isr_wrapper_0xdf);
  setupIDT(0xe0, (vaddr)&isr_wrapper_0xe0);
  setupIDT(0xe1, (vaddr)&isr_wrapper_0xe1);
  setupIDT(0xe2, (vaddr)&isr_wrapper_0xe2);
  setupIDT(0xe3, (vaddr)&isr_wrapper_0xe3);
  setupIDT(0xe4, (vaddr)&isr_wrapper_0xe4);
  setupIDT(0xe5, (vaddr)&isr_wrapper_0xe5);
  setupIDT(0xe6, (vaddr)&isr_wrapper_0xe6);
  setupIDT(0xe7, (vaddr)&isr_wrapper_0xe7);
  setupIDT(0xe8, (vaddr)&isr_wrapper_0xe8);
  setupIDT(0xe9, (vaddr)&isr_wrapper_0xe9);
  setupIDT(0xea, (vaddr)&isr_wrapper_0xea);
  setupIDT(0xeb, (vaddr)&isr_wrapper_0xeb);
  setupIDT(0xec, (vaddr)&isr_wrapper_0xec);
  setupIDT(0xed, (vaddr)&isr_wrapper_0xed);
  setupIDT(0xee, (vaddr)&isr_wrapper_0xee);
  setupIDT(0xef, (vaddr)&isr_wrapper_0xef);
  setupIDT(0xf0, (vaddr)&isr_wrapper_0xf0);
  setupIDT(0xf1, (vaddr)&isr_wrapper_0xf1);
  setupIDT(0xf2, (vaddr)&isr_wrapper_0xf2);
  setupIDT(0xf3, (vaddr)&isr_wrapper_0xf3);
  setupIDT(0xf4, (vaddr)&isr_wrapper_0xf4);
  setupIDT(0xf5, (vaddr)&isr_wrapper_0xf5);
  setupIDT(0xf6, (vaddr)&isr_wrapper_0xf6);
  setupIDT(0xf7, (vaddr)&isr_wrapper_0xf7);
  setupIDT(0xf8, (vaddr)&isr_wrapper_0xf8);
  setupIDT(0xf9, (vaddr)&isr_wrapper_0xf9);
  setupIDT(0xfa, (vaddr)&isr_wrapper_0xfa);
  setupIDT(0xfb, (vaddr)&isr_wrapper_0xfb);
  setupIDT(0xfc, (vaddr)&isr_wrapper_0xfc);
  setupIDT(0xfd, (vaddr)&isr_wrapper_0xfd);
  setupIDT(0xfe, (vaddr)&isr_wrapper_0xfe);
  setupIDT(0xff, (vaddr)&isr_wrapper_0xff);
}

template<bool irq>
class IsrEntry {
  mword* frame;
  constexpr mword* rip()    const { return frame;   }
  constexpr mword* cs()     const { return frame+1; }
  constexpr mword* rflags() const { return frame+2; }
  constexpr mword* rsp()    const { return frame+3; }
  constexpr mword* ss()     const { return frame+4; }
public:
  IsrEntry(mword* is) : frame(is) {
    if (irq) MappedAPIC()->sendEOI();
    if (Processor::userSegment(*cs())) CPU::SwapGS();
    LocalProcessor::lockFake();
    kernelSpace.runKernelInvalidation();
  }
  ~IsrEntry() {
    LocalProcessor::unlockFake();
    if (Processor::userSegment(*cs())) {
      checkSignals();
      LocalProcessor::setKernelStack();
      CPU::SwapGS();
    }
  }
  void checkSignals() {
    Process& p = CurrProcess();
    if (p.getSignalHandler()) {         // TODO: only if signal posted
      mword** userSP = (mword**)rsp();  // access user stack
      *userSP -= 1;                     // make room for return address
      **userSP = *rip();                // put return address on user stack
      *userSP -= 1;                     // make room for signal number
      **userSP = 0xdeadbeef;            // put signal number on user stack
      *rip() = p.getSignalHandler();    // return to sighandler from ISR
    }                                   // TODO: else kill thread/process
  }
};

extern "C" void exception_handler_0x00(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("DIVIDE ERROR @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x01(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("DEBUG @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x02(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("NMI @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x03(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("BREAKPOINT @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x04(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("OVERFLOW @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x05(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("BOUND RANGE EXCEEDED @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x06(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("INVALID OPCODE @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x07(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("DEVICE NOT AVAILABLE @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x08(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("DOUBLE FAULT @ ", FmtHex(*isrFrame), " / error: ", FmtHex(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x09(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("COPROCESSOR SEGMENT OVERRUN @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x0a(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("INVALID TSS @ ", FmtHex(*isrFrame), " / error: ", FmtHex(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x0b(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("SEGMENT NOT PRESENT @ ", FmtHex(*isrFrame), " / error: ", FmtHex(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x0c(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("STACK FAULT @ ", FmtHex(*isrFrame), " / error: ", FmtHex(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x0d(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("GENERAL PROTECTION FAULT @ ", FmtHex(*isrFrame), " / error: ", FmtHex(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x0e(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  vaddr da = CPU::readCR2();
  if (Paging::fault(da, *LocalProcessor::getFrameManager())) return;
  KERR::outl("PAGE FAULT @ ", FmtHex(*isrFrame), " / data: ", FmtHex(da), " / flags:", Paging::PageFaultFlags(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x10(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("FPU ERROR @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_errcode_0x11(mword* isrFrame, mword ec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("ALIGNMENT CHECK @ ", FmtHex(*isrFrame), " / error: ", FmtHex(ec));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x12(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("MACHINE CHECK @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x13(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("SIMD FP @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_0x14(mword* isrFrame) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("VIRTUALIZATION @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void exception_handler_undefined(mword* isrFrame, mword vec) {
  IsrEntry<false> ie(isrFrame);
  KERR::outl("UNDEFINED EXCEPTION ", FmtHex(vec), " @ ", FmtHex(*isrFrame));
  Reboot(*isrFrame);
}

extern "C" void irq_handler_async(mword* isrFrame, mword idx) {
  IsrEntry<true> ie(isrFrame);
  irqMask.set<true>(idx);
#if TESTING_REPORT_INTERRUPTS
  KERR::out1(" AI:", FmtHex(idx));
#endif
}

extern "C" void irq_handler_0xe0(mword* isrFrame) { // APIC::WakeIPI
  IsrEntry<true> ie(isrFrame);
  LocalProcessor::getScheduler()->preempt();
}

extern "C" void irq_handler_0xed(mword* isrFrame) { // APIC::PreemptIPI
  IsrEntry<true> ie(isrFrame);
  LocalProcessor::getScheduler()->preempt();
}

extern "C" void irq_handler_0xee(mword* isrFrame) { // APIC::TestIPI
  IsrEntry<true> ie(isrFrame);
  if (tipiHandler) {
    tipiHandler();
  } else {
    KERR::outl("NO HANDLER FOR TEST IPI @ ", FmtHex(*isrFrame));
    Reboot();
  }
}

extern "C" void irq_handler_0xef(mword* isrFrame) { // APIC::StopIPI
  for (;;) CPU::Halt();
}

extern "C" void irq_handler_0xf0(mword* isrFrame) { // PIT interrupt
  IsrEntry<true> ie(isrFrame);
  Clock::ticker();
#if TESTING_REPORT_INTERRUPTS
  KERR::out1(" PIT");
#endif
}

extern "C" void irq_handler_0xf7(mword* isrFrame) { // parallel interrupt, spurious no problem
  IsrEntry<true> ie(isrFrame);
  KERR::out1(" parallel");
}

extern "C" void irq_handler_0xf8(mword* isrFrame) { // RTC interrupt
  IsrEntry<true> ie(isrFrame);
  rtc.staticInterruptHandler();          // RTC processing
#if TESTING_REPORT_INTERRUPTS
  KERR::out1(" RTC");
#endif
  if (!irqMask.empty()) asyncIrqSem.V(); // check interrupts
  Timeout::checkExpiry(Clock::now());    // check timeout queue
  Machine::rrPreemptIPI(rtc.tick());     // simulate APIC timer interrupts
}

extern "C" void irq_handler_0xf9(mword* isrFrame) { // spuriously seen
  IsrEntry<true> ie(isrFrame);
  KERR::out1(" IRQ-F9");
}

extern "C" void irq_handler_0xfc(mword* isrFrame) { // mouse interrupt
  IsrEntry<true> ie(isrFrame);
  KERR::out1(" mouse");
}

extern "C" void irq_handler_0xff(mword* isrFrame) { // bochs quirk?
  IsrEntry<true> ie(isrFrame);
  KERR::out1(" IRQ-FF");
}

void Breakpoint2(vaddr ia) {
  asm volatile("nop");
}

void Steppoint() {
  while (CPU::in8(0x64) & 0x01) CPU::in8(0x60); // clear read buffer
  while (!(CPU::in8(0x64) & 0x01));        // wait for any key press
}

void Reboot(vaddr ia) {
  asm volatile("cli");                // disable interrupts
  Breakpoint(ia);
#if 1
  for (mword i = 0; i < Machine::getProcessorCount(); i++) {
    if (i != LocalProcessor::getIndex()) Machine::sendIPI(i, APIC::StopIPI);
  }
  mword rbp;
  asm volatile("mov %%rbp, %0" : "=r"(rbp));
  KOUT::outl();
  for (int i = 0; i < 20 && rbp != 0; i += 1) {
    KOUT::outl("XBT: ", FmtHex(*(mword*)(rbp + sizeof(mword))));
    rbp = *(mword*)(rbp);
  }
  KOUT::outl();
#endif
  Steppoint();
  loadIDT(0,0);                       // load empty IDT
  asm volatile("int $0xff");          // trigger triple fault
  unreachable();
}

extern "C" void KosReboot() { Reboot(); }
