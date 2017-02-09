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
#include "devices/PCI.h"

// handle multiple PCI host controllers
void PCI::checkAllBuses(list<PCIDevice>& pciDevList) {
  if ((HeaderType(0,0,0) & 0x80) == 0) checkBus(0, pciDevList); // only one PCI host controller
  else for (uint8_t func = 0; func < MaxBus; func++) {  // check all possible controllers;
    if (VendorID(0,0,func) == 0xffff) checkBus(func, pciDevList);
  }
}

// scans one bus
void PCI::checkBus(uint8_t bus, list<PCIDevice>& pciDevList) {
  DBG::outl(DBG::PCI, "Checking BUS ", FmtHex(bus));
  for (uint8_t dev = 0; dev < MaxDevice; dev++) {
    checkDevice(bus, dev, pciDevList);
  }
}

// check if a specific device on a specific bus is present
void PCI::checkDevice(uint8_t bus, uint8_t dev, list<PCIDevice>& pciDevList) {
  if (VendorID(bus,dev,0) == 0xffff) return;    // device doesn't exist
  checkFunction(bus, dev, 0, pciDevList);
  if ((HeaderType(bus,dev,0) & 0x80) != 0) {    // multi-function
    for (uint8_t func = 1; func < MaxFunction; func++) {
      if (VendorID(bus,dev,func) != 0xffff) checkFunction(bus, dev, func, pciDevList);
    }
  }
}

// detects if the function is a PCI to PCI bridge; also add the detected PCI device
void PCI::checkFunction(uint8_t bus, uint8_t dev, uint8_t func, list<PCIDevice>& pciDevList) {
  DBG::outl(DBG::PCI, "PCI b/d/f: ", FmtHex(bus,2), '/', FmtHex(dev,2), '/', FmtHex(func,2),
                      " v/d: ", FmtHex(VendorID(bus,dev,func),4), '/', FmtHex(DeviceID(bus,dev,func),4),
                      " cmd: ", FmtHex(Command(bus,dev,func),4), " st: ", FmtHex(Status(bus,dev,func),4));
  list<uint32_t> busList;
  uint32_t ht = HeaderType(bus,dev,func);
  switch (ht & 0x7f) {
    case 0x00: {
      uint8_t il = InterruptLine(bus,dev,func);
      uint8_t ip = InterruptPin(bus,dev,func);
      DBG::out1(DBG::PCI, "IRQ L/P: ", FmtHex(il,2), '/', ip ? char('A' + ip - 1) : 'X');
      DBG::out1(DBG::PCI, " BARs:");
      for (int i = 0; i < 6; i += 1) {
        DBG::out1(DBG::PCI, ' ', FmtHex(BAR(bus,dev,func,i)));
      }
      pciDevList.push_back( {bus, dev, func, il} );
    } break;
    case 0x01: {
      uint32_t sb = SecondaryBus(bus,dev,func);
      DBG::out1(DBG::PCI, "SecondaryBus: ", FmtHex(sb));
      DBG::out1(DBG::PCI, " BARs:");
      for (int i = 0; i < 2; i += 1) {
        DBG::out1(DBG::PCI, ' ', FmtHex(BAR(bus,dev,func,i)));
      }
      if (ClassCode(bus,dev,func) == 0x06 && SubClass(bus,dev,func) == 0x04 ) {
        busList.push_back(sb);
      }
    } break;
    case 0x02: {
      break;
    }
    default: KABORT1(FmtHex(ht));
  }
  DBG::outl(DBG::PCI);
  for (uint32_t sb : busList) checkBus(sb, pciDevList);
}
