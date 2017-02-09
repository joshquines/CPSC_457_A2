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
#ifndef _PCI_h_
#define _PCI_h_ 1

#include "kernel/Output.h"
#include "machine/CPU.h"

#include <list>

// details taken from http://wiki.osdev.org/PCI

class PCIDevice {
  uint8_t bus;
  uint8_t dev;
  uint8_t func;
  uint8_t irq;
  void setIrq(uint8_t i) { irq = i; }
public:
  PCIDevice(uint8_t b, uint8_t d, uint8_t f, uint8_t i) : bus(b), dev(d), func(f), irq(i) {}
  uint8_t getBus() const { return bus; }
  uint8_t getDevice() const { return dev; }
  uint8_t getFunction() const { return func; }
  uint8_t getIrq() const { return irq; }
  inline uint32_t getBARSize(uint8_t idx) const;
};

namespace PCI {
  static const uint16_t AddressPort = 0xCF8;
  static const uint16_t DataPort    = 0xCFC;
  static const uint8_t  MaxBus       = 8;
  static const uint8_t  MaxDevice    = 32;
  static const uint8_t  MaxFunction  = 8;
  static const uint32_t DefaultWidth = 32;

  union Config {
    uint32_t c;
    struct {
      uint32_t Zero     : 2;
      uint32_t Register : 6;
      uint32_t Function : 3;
      uint32_t Device   : 5;
      uint32_t Bus      : 8;
      uint32_t Reserved : 7;
      uint32_t Enable   : 1;
    };
    Config(uint8_t b, uint8_t d, uint8_t f, uint8_t r)
      : Zero(0), Register(r), Function(f), Device(d), Bus(b), Enable(1) {}
    operator uint32_t() const { return c; }
  } __packed;

  static inline void sanityCheck() {
    CPU::out32(AddressPort, 0x80000000);
    KASSERT1(CPU::in32(AddressPort) == 0x80000000, "No PCI controller detected!");
  }

  template<mword width=32>
  static inline uint32_t readConfig(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg) {
    static_assert(width == 8 || width == 16 || width == 32, "unsupported width");
    uint32_t shift = (reg & 3) * 8;
    KASSERTN(width + shift <= 32, width, ' ', shift);
    CPU::out32(AddressPort, Config(bus,dev,func,reg / 4));
    return (CPU::in32(DataPort) >> shift) & bitmask<uint32_t>(width);
  }

  template<mword width=32>
  static inline uint32_t readConfig(const PCIDevice& pd, uint8_t reg) {
    return readConfig<width>(pd.getBus(), pd.getDevice(), pd.getFunction(), reg);
  }

  template<mword width=32>
  static inline void writeConfig(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t value) {
    static_assert(width == 8 || width == 16 || width == 32, "unsupported width");
    uint32_t shift = (reg & 3) * 8;
    KASSERTN(width + shift <= 32, width, ' ', shift);
    CPU::out32(AddressPort, Config(bus,dev,func,reg / 4));
    CPU::out32(DataPort, (value & bitmask<uint32_t>(width)) << shift);
  }

  template<mword width=32>
  static inline void writeConfig(const PCIDevice& pd, uint8_t reg, uint32_t val) {
    writeConfig<width>(pd.getBus(), pd.getDevice(), pd.getFunction(), reg, val);
  }

  template<int reg,mword width>
  struct Register {
    uint32_t c;
    void read(uint8_t bus, uint8_t dev, uint8_t func, uint8_t off = 0) {
      c = PCI::readConfig<width*8>(bus, dev, func, reg + off*width);
    }
    void read(const PCIDevice& pd, uint8_t off = 0) {
      read(pd.getBus(), pd.getDevice(), pd.getFunction(), off);
    }
    void write(uint8_t bus, uint8_t dev, uint8_t func, uint8_t off = 0) {
      PCI::writeConfig<width*8>(bus, dev, func, reg + off*width, c);
    }
    void write(const PCIDevice& pd, uint8_t off = 0) {
      write(pd.getBus(), pd.getDevice(), pd.getFunction(), off);
    }
    static void write(uint8_t bus, uint8_t dev, uint8_t func, uint32_t val, uint8_t off = 0) {
      PCI::writeConfig<width*8>(bus, dev, func, reg + off*width, val);
    }
    static void write(const PCIDevice& pd, uint32_t val, uint8_t off = 0) {
      write(pd.getBus(), pd.getDevice(), pd.getFunction(), val, off);
    }
    Register() = default;
    Register(uint8_t bus, uint8_t dev, uint8_t func, uint8_t off = 0) { read(bus, dev, func, off); }
    Register(const PCIDevice& pd, uint8_t off = 0) { read(pd, off); }
    operator uint32_t() const { return c; }
  };

  static const BitString<uint16_t, 0,1> IOSpace;
  static const BitString<uint16_t, 1,1> MemorySpace;
  static const BitString<uint16_t, 2,1> BusMaster;
  static const BitString<uint16_t, 3,1> SpecialCycles;
  static const BitString<uint16_t, 4,1> MemoryWriteAndInvalidate;
  static const BitString<uint16_t, 5,1> VGAPaletteSnoop;
  static const BitString<uint16_t, 6,1> ParityErrorResponse;
  static const BitString<uint16_t, 8,1> SERR;
  static const BitString<uint16_t, 9,1> FastBackToBack;
  static const BitString<uint16_t,10,1> InterruptDisable;

  static const BitString<uint16_t, 3,1> InterruptStatus;
  static const BitString<uint16_t, 4,1> CapabilitiesList;
  static const BitString<uint16_t, 5,1> MHz66Cap;
  static const BitString<uint16_t, 7,1> FastBackToBackCap;
  static const BitString<uint16_t, 8,1> MasterDataParityError;
  static const BitString<uint16_t, 9,2> DEVSELTiming;
  static const BitString<uint16_t,11,1> SignaledTargetAbort;
  static const BitString<uint16_t,12,1> ReceivedTargetAbort;
  static const BitString<uint16_t,13,1> ReceivedMasterAbort;
  static const BitString<uint16_t,14,1> SignaledSystemError;
  static const BitString<uint16_t,15,1> DetectedParityError;

  // Common registers
  typedef Register< 0,2> VendorID;
  typedef Register< 2,2> DeviceID;
  typedef Register< 4,2> Command;
  typedef Register< 6,2> Status;
  typedef Register< 8,1> RevisionID;
  typedef Register< 9,1> ProgIF;
  typedef Register<10,1> SubClass;
  typedef Register<11,1> ClassCode;
  typedef Register<12,1> CacheLineSize;
  typedef Register<13,1> LatencyTimer;
  typedef Register<14,1> HeaderType;
  typedef Register<15,1> BIST;

  // Device registers
  typedef Register<16,4> BAR;                // 6 BARs, read/write with offset
  typedef Register<40,4> CardbusCIS;
  typedef Register<44,2> SubVendorID;
  typedef Register<46,2> SubSystemID;
  typedef Register<48,4> ExpansionROM;
  typedef Register<52,1> Capabilities;
  typedef Register<60,1> InterruptLine;
  typedef Register<61,1> InterruptPin;
  typedef Register<62,1> MinGrant;
  typedef Register<63,1> MaxLatency;

  // BridgePCI registers - not complete (not needed)
  // reuse BAR from Device registers         // 2 BARs, read/write with offset
  typedef Register<24,1> PrimaryBus;
  typedef Register<25,1> SecondaryBus;
  typedef Register<26,1> SubordinateBus;
  typedef Register<27,1> SecondaryLatencyTimer;

  // BridgeCardbus registers - not needed

  void checkAllBuses(list<PCIDevice>& pciDevList);
  void checkBus(uint8_t bus, list<PCIDevice>& pciDevList);
  void checkDevice(uint8_t bus, uint8_t dev, list<PCIDevice>& pciDevList);
  void checkFunction(uint8_t bus, uint8_t dev, uint8_t func, list<PCIDevice>& pciDevList);
};

uint32_t PCIDevice::getBARSize(uint8_t idx) const {
  uint32_t bar = PCI::BAR(*this,idx);      // save original BAR
  PCI::BAR::write(*this, 0xffffffff, idx); // write full mask
  uint32_t mask = PCI::BAR(*this,idx);     // read back
  PCI::BAR::write(*this, bar, idx);        // restore original BAR
  return ~mask + 1;                        // compute & return size
}

#endif /* _PCI_h_ */
