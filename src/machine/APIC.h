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
#ifndef _APIC_h_
#define _APIC_h_ 1

#include "generic/basics.h"
#include "machine/Memory.h"

struct PIC {
  enum IRQs { // see http://en.wikipedia.org/wiki/Intel_8259 and
              //     http://wiki.osdev.org/Interrupts#Standard_ISA_IRQs
    PIT       =  0,
    Keyboard  =  1,
    Cascade   =  2,
    Serial1   =  3,
    Serial0   =  4,
    Parallel2 =  5,
    Floppy    =  6,
    Parallel1 =  7,
    RTC       =  8,
    SCI       =  9, // ACPI system control interrupt; power management
    Mouse     = 12,
    FPU       = 13,
    ATA1      = 14,
    ATA2      = 15,
    Max       = 16
  };
  static void disable();
};

// Note that APIC registers have to be written as a complete 32-bit word
// see Intel Vol 3, Section 10.4 "Local APIC"
class APIC : public NoObject {
  friend class IOAPIC;

  __aligned(0x10) volatile uint32_t Reserved0x000;     // 0x000
  __aligned(0x10) volatile uint32_t Reserved0x010;
  __aligned(0x10) volatile uint32_t ID;
    const BitString<uint32_t,24,8> APIC_ID;

  __aligned(0x10) volatile uint32_t Version;           // 0x030
  __aligned(0x10) volatile uint32_t Reserved0x040;
  __aligned(0x10) volatile uint32_t Reserved0x050;
  __aligned(0x10) volatile uint32_t Reserved0x060;
  __aligned(0x10) volatile uint32_t Reserved0x070;
  __aligned(0x10) volatile uint32_t TaskPriority;      // 0x080
  __aligned(0x10) volatile uint32_t ArbitrationPriority;
  __aligned(0x10) volatile uint32_t ProcessorPriority;
  __aligned(0x10) volatile uint32_t EOI;
  __aligned(0x10) volatile uint32_t RemoteRead;
  __aligned(0x10) volatile uint32_t LogicalDest;
  __aligned(0x10) volatile uint32_t DestFormat;
  __aligned(0x10) volatile uint32_t SpuriosInterruptVector;
    const BitString<uint32_t, 0,8> SpuriousVector;
    const BitString<uint32_t, 8,1> SoftwareEnable;
    const BitString<uint32_t, 9,1> FocusProcCheck;
    const BitString<uint32_t,12,1> BroadcastSupp;

  __aligned(0x10) volatile uint32_t InService0;        // 0x100
  __aligned(0x10) volatile uint32_t InService1;
  __aligned(0x10) volatile uint32_t InService2;
  __aligned(0x10) volatile uint32_t InService3;
  __aligned(0x10) volatile uint32_t InService4;
  __aligned(0x10) volatile uint32_t InService5;
  __aligned(0x10) volatile uint32_t InService6;
  __aligned(0x10) volatile uint32_t InService7;
  __aligned(0x10) volatile uint32_t TriggerMode0;      // 0x180
  __aligned(0x10) volatile uint32_t TriggerMode1;
  __aligned(0x10) volatile uint32_t TriggerMode2;
  __aligned(0x10) volatile uint32_t TriggerMode3;
  __aligned(0x10) volatile uint32_t TriggerMode4;
  __aligned(0x10) volatile uint32_t TriggerMode5;
  __aligned(0x10) volatile uint32_t TriggerMode6;
  __aligned(0x10) volatile uint32_t TriggerMode7;
  __aligned(0x10) volatile uint32_t InterruptRequest0; // 0x200
  __aligned(0x10) volatile uint32_t InterruptRequest1;
  __aligned(0x10) volatile uint32_t InterruptRequest2;
  __aligned(0x10) volatile uint32_t InterruptRequest3;
  __aligned(0x10) volatile uint32_t InterruptRequest4;
  __aligned(0x10) volatile uint32_t InterruptRequest5;
  __aligned(0x10) volatile uint32_t InterruptRequest6;
  __aligned(0x10) volatile uint32_t InterruptRequest7;
  __aligned(0x10) volatile uint32_t ErrorStatus;       // 0x280
    const BitString<uint32_t, 0,1> SendChecksum;
    const BitString<uint32_t, 1,1> ReceiveChecksum;
    const BitString<uint32_t, 2,1> SendAccept;
    const BitString<uint32_t, 3,1> ReceiveAccept;
    const BitString<uint32_t, 4,1> RedirectableIPI;
    const BitString<uint32_t, 5,1> SendIllegalVector;
    const BitString<uint32_t, 6,1> ReceiveIllegalVector;
    const BitString<uint32_t, 7,1> IllegalRegisterAddress;

  __aligned(0x10) volatile uint32_t Reserved0x290;
  __aligned(0x10) volatile uint32_t Reserved0x2a0;
  __aligned(0x10) volatile uint32_t Reserved0x2b0;
  __aligned(0x10) volatile uint32_t Reserved0x2c0;
  __aligned(0x10) volatile uint32_t Reserved0x2d0;
  __aligned(0x10) volatile uint32_t Reserved0x2e0;
  __aligned(0x10) volatile uint32_t LVT_CMCI;
  __aligned(0x10) volatile uint32_t ICR_LOW;           // 0x300
    const BitString<uint32_t, 0,8> Vector;
    const BitString<uint32_t, 8,3> DeliveryMode;          // see below
    const BitString<uint32_t,11,1> DestinationMode;       // 0: Physical, 1: Logical
    const BitString<uint32_t,12,1> DeliveryPending;       // 0: Idle, 1: Pending
    const BitString<uint32_t,14,1> LevelUp;
    const BitString<uint32_t,15,1> TriggerModeLevel;      // 0: Edge, 1: Level
    const BitString<uint32_t,18,2> DestinationShorthand;

  __aligned(0x10) volatile uint32_t ICR_HIGH;          // 0x310
    const BitString<uint32_t,24,8> DestField;

  __aligned(0x10) volatile uint32_t LVT_Timer;         // 0x320
    const BitString<uint32_t,16,1> MaskTimer;
    const BitString<uint32_t,17,2> TimerMode;

  __aligned(0x10) volatile uint32_t LVT_ThermalSensor; // 0x330
  __aligned(0x10) volatile uint32_t LVT_PMCs;
  __aligned(0x10) volatile uint32_t LVT_LINT0;
  __aligned(0x10) volatile uint32_t LVT_LINT1;
  __aligned(0x10) volatile uint32_t LVT_Error;
  __aligned(0x10) volatile uint32_t InitialCount;      // 0x380
  __aligned(0x10) volatile uint32_t CurrentCount;
  __aligned(0x10) volatile uint32_t Reserved0x3a0;
  __aligned(0x10) volatile uint32_t Reserved0x3b0;
  __aligned(0x10) volatile uint32_t Reserved0x3c0;
  __aligned(0x10) volatile uint32_t Reserved0x3d0;
  __aligned(0x10) volatile uint32_t DivideConfiguration;
  __aligned(0x10) volatile uint32_t Reserved0x3f0;

  enum DeliveryModes { // Intel Vol. 3, Section 10.6.1 "Interrupt Command Register"
    Fixed          = 0b000,
    LowestPriority = 0b001,
    SMI            = 0b010,
    NMI            = 0b100,
    Init           = 0b101,
    Startup        = 0b110
  };

  enum DestinationShorthands { // Intel Vol. 3, Section 10.6.1 "Interrupt Command Register"
    None        = 0b00,
    Self        = 0b01,
    AllInclSelf = 0b10,
    AllExclSelf = 0b11
  };

  enum TimerModes { //
    OneShot  = 0b00,
    Periodic = 0b01,
    Deadline = 0b10
  };

  void ipi(uint32_t high, uint32_t low, bool broadcast) {
    if (broadcast) low |= DestinationShorthand.put(AllExclSelf);
    ICR_HIGH = high;
    ICR_LOW = low;
    if (!broadcast) while (ICR_LOW & DeliveryPending());
    mword err = ErrorStatus;
    KASSERT1(err == 0, FmtHex(err));
  }

public:
  uint8_t getID() {
    return APIC_ID.get(ID);
  }
  void setTaskPriority(uint32_t p) {
    TaskPriority = p;
  }
  void sendEOI() {
    EOI = 0;
  }
  void enable() {
    SpuriosInterruptVector |= SoftwareEnable();
  }
  void enable(uint8_t sv) {
    SpuriosInterruptVector |= SoftwareEnable() | SpuriousVector.put(sv);
  }
  void disable() {
    SpuriosInterruptVector &= ~SoftwareEnable();
  }
  void setLogicalDest(uint8_t group) {
    LogicalDest = APIC_ID.put(group);
  }
  void setFlatMode() {
    DestFormat = 0xFFFFFFFF;
  }
  void maskTimer() {
    LVT_Timer |= MaskTimer();
  }
  void sendInitIPI(uint8_t dest, bool broadcast = false) {
    ipi(DestField.put(dest), DeliveryMode.put(Init), broadcast);
  }
  void sendInitDeassertIPI(uint8_t dest, bool broadcast = false) {
    ipi(DestField.put(dest), DeliveryMode.put(Init) | TriggerModeLevel(), broadcast);
  }
  void sendStartupIPI(uint8_t dest, uint8_t vec, bool broadcast = false) {
    ipi(DestField.put(dest), DeliveryMode.put(Startup) | Vector.put(vec), broadcast);
  }
  void sendIPI(uint8_t dest, uint8_t vec, bool broadcast = false) {
    ipi(DestField.put(dest), DeliveryMode.put(Fixed) | Vector.put(vec), broadcast);
  }
  static const uint8_t WakeIPI    = 0xe0; // remote wakeup
  static const uint8_t PreemptIPI = 0xed; // preemption
  static const uint8_t TestIPI    = 0xee; // test IPI: bootstrap & experiment
  static const uint8_t StopIPI    = 0xef; // stop, used for GDB or reboot
} __packed;


// Intel 82093AA IOAPIC, Section 3.1 "Memory Mapped Registers..."
class IOAPIC : public NoObject {
  friend class Machine;

  __aligned(0x10) volatile uint32_t RegisterSelect;    // 0x000
  __aligned(0x10) volatile uint32_t Window;

  enum Registers : uint8_t {
    IOAPICID =  0x00,
    IOAPICVER = 0x01,
    IOAPICARB = 0x02,
    IOREDTBL =  0x10
  };

  uint32_t read( uint8_t reg ) {
    RegisterSelect = reg;
    return Window;
  }
  void write( uint8_t reg, uint32_t val ) {
    RegisterSelect = reg;
    Window = val;
  }

  const BitString<uint32_t, 0, 8> Version;
  const BitString<uint32_t,16, 8> MaxRedirectionEntry;
  const BitString<uint32_t,24, 4> ArbitrationID;
  const BitString<uint32_t,24, 4> ID;

  const BitString<uint64_t, 0, 8> Vector;           // interrupt number
  const BitString<uint64_t, 8, 3> DeliveryMode;     // see Intel spec, cf. APIC
  const BitString<uint64_t,11, 1> DestinationMode;  // 0: Physical, 1: Logical
  const BitString<uint64_t,12, 1> DeliveryPending;  // 0: Idle, 1: Pending
  const BitString<uint64_t,13, 1> Polarity;         // 0: High, 1: Low
  const BitString<uint64_t,14, 1> RemoteIRR;        // 0: EOI received
  const BitString<uint64_t,15, 1> TriggerModeLevel; // 0: Edge, 1: Level
  const BitString<uint64_t,16, 1> Mask;             // 1: interrupt masked
  const BitString<uint64_t,56, 4> DestinationID;    // APIC ID
  const BitString<uint64_t,56, 8> DestinationSet;   // group, cf APIC spec

  uint8_t getVersion();
  uint8_t getRedirects();
  void maskIRQ(uint8_t irq);
  // TODO: all IRQs are send to APIC logical group 0x01 for now...
  void mapIRQ(uint8_t irq, uint8_t intr, bool low = false, bool level = false);
} __packed;

static   APIC*   MappedAPIC() { return   (APIC*)apicAddr; }
static IOAPIC* MappedIOAPIC() { return (IOAPIC*)ioApicAddr; }

#endif /* APIC */
