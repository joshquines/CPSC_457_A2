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
#include "runtime/BlockingSync.h"
#include "kernel/MemoryManager.h"
#include "kernel/Output.h"
#include "machine/Machine.h"
#include "machine/Paging.h"
#include "machine/Processor.h"
#include "devices/PCI.h"

#include <cstdarg>
#include <list>
#undef __STRICT_ANSI__  // to get access to vsnprintf
#include <cstdio>

extern "C" void cdi_printf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  ExternDebugPrintf(DBG::CDI, fmt, args);
  va_end(args);
}

#include "cdi.h"
#include "cdi/pci.h"

// see http://stackoverflow.com/questions/16552710/how-do-you-get-the-start-and-end-addresses-of-a-custom-elf-section-in-c-gcc
extern struct cdi_driver* __start_cdi_drivers;  // first driver
extern struct cdi_driver* __stop_cdi_drivers;   // 1 past last driver

list<cdi_driver*> driverList;

struct netif;
static netif* lwip_netif = nullptr;
extern netif* lwip_add_netif(void *ethif);
extern void lwip_net_receive(netif*, bufptr_t buffer, size_t size);

void initCdiDrivers() {
  for (cdi_driver** pdrv = &__start_cdi_drivers; pdrv < &__stop_cdi_drivers; pdrv += 1) {
    cdi_driver* drv = *pdrv;
    if (drv->init) drv->init();
    cdi_driver_register(drv);
  }
}

bool findCdiDriver(const PCIDevice &pciDev) {
  // create CDI-style pci device object from KOS' PCIDevice
  cdi_pci_device* cpd = knew<cdi_pci_device>();
  cpd->bus_data.bus_type = CDI_PCI;

  cpd->vendor_id    = PCI::VendorID(pciDev);
  cpd->device_id    = PCI::DeviceID(pciDev);
  cpd->class_id     = PCI::ClassCode(pciDev);
  cpd->subclass_id  = PCI::SubClass(pciDev);
  cpd->interface_id = PCI::ProgIF(pciDev);
  cpd->rev_id       = PCI::RevisionID(pciDev);
  cpd->bus          = pciDev.getBus();
  cpd->dev          = pciDev.getDevice();
  cpd->function     = pciDev.getFunction();
  cpd->irq          = pciDev.getIrq();
  cpd->resources    = cdi_list_create();

  // enable I/O ports, MMIO, and Bus Mastering for this device
#if 0
  PCI::writeConfig<32>(pciDev, 4, PCI::readConfig<32>(pciDev, 4) | 0x7);
#else
  PCI::Command c(pciDev);
  PCI::Command::write(pciDev, c | PCI::IOSpace() | PCI::MemorySpace() | PCI::BusMaster());
#endif

  // add BARs as resources
  uint8_t numBars = ((PCI::HeaderType(pciDev) & 0x7f) == 0) ? 6 : 2;
  for (int i = 0; i < numBars; i++) {
    uint32_t bar = PCI::BAR(pciDev,i);
    if (bar == 0) continue;
    cdi_pci_resource* res = knew<cdi_pci_resource>();
    res->length = pciDev.getBARSize(i);
    res->index = i;
    res->address = 0;                     // not yet mapped to vaddr
    if (bar & 0x1) {
      res->type = CDI_PCI_IOPORTS;
      res->start = (bar & 0xfffffffc);
      res->length &= 0xffff;              // port number range 2^16
    } else {
      res->type = CDI_PCI_MEMORY;
      res->start = (bar & 0xfffffff0);
      switch ((bar & 0x6) >> 1) {         // memory width?
        case 0x00: break;                 // 32 bit
        case 0x01: KABORT1("BAR"); break;  // reserved for PCI LocalBus 3.0
        case 0x02:                        // 64 bit -> consumes 2 registers
          res->start += (uint64_t(PCI::BAR(pciDev,i+1)) << 32);
          res->length += (uint64_t(pciDev.getBARSize(i+1)) << 32);
          i += 1;
          break;
        default: KABORT1(FmtHex(bar)); break;
      }
    }
    cdi_list_push(cpd->resources, res);
  }

  // find driver
  for (auto it = driverList.begin(); it != driverList.end(); ++it) {
    if ((*it)->bus == CDI_PCI && (*it)->init_device) {
      cdi_device* device = (*it)->init_device(&cpd->bus_data);
      if (device) {
        device->driver = (*it);
        cdi_list_push((*it)->devices, device);
        cdi_printf("PCI device %02X:%02X:%02X - driver found: %s\n", cpd->bus, cpd->dev, cpd->function, (*it)->name);
        lwip_netif = lwip_add_netif(device);
        return true;
      }
    }
  }

  // no driver found -> remove pci device object
  cdi_pci_resource* res;
  for (int i = 0;; ++i) {
    res = (cdi_pci_resource*)cdi_list_get(cpd->resources,i);
    if (!res) break;
    kdelete(res);
  }
  kdelete(cpd);
  return false;
}

void cdi_driver_init(cdi_driver* driver) {
  driver->devices = cdi_list_create();
}

void cdi_driver_destroy(cdi_driver* driver) {
  cdi_list_destroy(driver->devices);
}

// might be called from driver or via initCdiDrivers
void cdi_driver_register(cdi_driver* driver) {
  driverList.push_back(driver);
}

#include "cdi/mem.h"

cdi_mem_area* cdi_mem_alloc(size_t size, cdi_mem_flags_t flags) {
  vaddr vAddr;
  mword sgSize;
  KASSERT1(!(flags & CDI_MEM_DMA_16M), flags); // low 16M only needed for floppy or other old drivers?
  if ( flags & CDI_MEM_PHYS_CONTIGUOUS ) {
    paddr align = pow2<paddr>(flags & CDI_MEM_ALIGN_MASK);
    paddr limit = (flags & CDI_MEM_DMA_4G) ? pow2<paddr>(32) : topaddr;
    vAddr = MemoryManager::allocContig(size, align, limit);
    sgSize = size;
  } else if (size >= kernelps) {
    size = align_up(size, kernelps);
    vAddr = MemoryManager::map(size);
    sgSize = kernelps;
  } else {
    size = align_up(size, pagesize<1>());
    vAddr = MemoryManager::map(size);
    sgSize = pagesize<1>();
  }
  KASSERT0(vAddr != topaddr);
  cdi_mem_area* area = knew<cdi_mem_area>();
  area->size = size;
  area->vaddr = (ptr_t)vAddr;
  area->osdep.allocated = true;
  area->paddr.num = divup(size, sgSize);
  area->paddr.items = knewN<cdi_mem_sg_item>(area->paddr.num);
  for (mword i = 0; i < area->paddr.num; i += 1) {
    DBG::outl(DBG::CDI, "cdi_mem_alloc(): ", FmtHex(Paging::vtop(vAddr)), '/', FmtHex(sgSize), " -> ", FmtHex(vAddr));
    area->paddr.items[i].start = Paging::vtop(vAddr);
    area->paddr.items[i].size = sgSize;
    vAddr += sgSize;
  }
  return area;
}

cdi_mem_area* cdi_mem_map(uintptr_t pAddr, size_t size) {
  // NOTE: no need to reserve physical address (from PCI BAR) in FrameManager?
  vaddr vAddr = MemoryManager::map(size, paddr(pAddr));
  KASSERT0(vAddr != topaddr);
  DBG::outl(DBG::CDI, "cdi_mem_map(): ", FmtHex(pAddr), '/', FmtHex(size), " -> ", FmtHex(vAddr));
  cdi_mem_area* area = knew<cdi_mem_area>();
  area->size = size;
  area->vaddr = (ptr_t)vAddr;
  area->osdep.allocated = false;
  area->paddr.num = 1;
  area->paddr.items = knew<cdi_mem_sg_item>();
  area->paddr.items->start = pAddr;
  area->paddr.items->size = size;
  return area;
}

void cdi_mem_free(cdi_mem_area* area) {
  KASSERT0(area && area->vaddr);
  DBG::outl(DBG::CDI, "cdi_mem_free(): ", FmtHex(Paging::vtop(vaddr(area->vaddr))), '/', FmtHex(area->size), " -> ", FmtHex(area->vaddr));
  MemoryManager::unmap( vaddr(area->vaddr), area->size, area->osdep.allocated );
  kdelete(area->paddr.items, area->paddr.num);
  kdelete(area);
}

#include "cdi/misc.h"

void cdi_register_irq(uint8_t irq, void (*handler)(cdi_device *), cdi_device* device) {
  Machine::registerIrqAsync(irq, (funcvoid1_t)handler, (void*)device);
}

void cdi_sleep_ms(uint32_t ms) {
  Timeout::sleep(ms);
}

#include "cdi/net.h"

static unsigned long netcard_highest_id = 0;
static cdi_list_t netcard_list = nullptr;

void cdi_net_driver_init(struct cdi_net_driver* driver) {
  driver->drv.type = CDI_NETWORK;
  cdi_driver_init((cdi_driver*)driver);
  if (netcard_list == nullptr) netcard_list = cdi_list_create();
}

void cdi_net_driver_destroy(struct cdi_net_driver* driver) {
  cdi_driver_destroy((cdi_driver*)driver);
}

void cdi_net_device_init(struct cdi_net_device* device) {
  device->number = netcard_highest_id;
  cdi_list_push(netcard_list, device);
  netcard_highest_id += 1;
}

void cdi_net_receive(cdi_net_device* device, ptr_t buffer, size_t size) {
  //DBG::outl(DBG::CDI, "packet received");
  if (lwip_netif) lwip_net_receive(lwip_netif, (bufptr_t)buffer, size);
}

// just send via first available interface
void cdi_net_send(ptr_t buffer, size_t size) {
  cdi_net_device* dev = (cdi_net_device*)cdi_list_get(netcard_list, 0);
  cdi_net_driver* driver = (cdi_net_driver*)dev->dev.driver;
  driver->send_packet(dev, buffer, size);
  //DBG::outl(DBG::CDI, "packet sent: ", size);
}

#include "cdi/pci.h"

void cdi_pci_alloc_ioports(cdi_pci_device* device) {
  cdi_pci_resource* res;
  for (int i = 0; (res = (cdi_pci_resource *)cdi_list_get(device->resources, i)); i++) {
    if (res->type == CDI_PCI_IOPORTS) {
      // TODO: allocate res->start, res->length
    }
  }
}

void cdi_pci_free_ioports(cdi_pci_device* device) {
  cdi_pci_resource* res;
  for (int i = 0; (res = (cdi_pci_resource *)cdi_list_get(device->resources, i)); i++) {
    if (res->type == CDI_PCI_IOPORTS) {
      // TODO: free res->start, res->length
    }
  }
}

struct cdi_list_implementation {
  list<ptr_t> impl;
};

#include "cdi/lists.h"

cdi_list_t cdi_list_create() {
  return knew<cdi_list_implementation>();
}

void cdi_list_destroy(cdi_list_t list) {
  kdelete(list);
}

cdi_list_t cdi_list_push(cdi_list_t list, ptr_t value) {
  list->impl.push_front(value);
  return list;
}

ptr_t cdi_list_pop(cdi_list_t list) {
  if (list->impl.empty()) return nullptr;
  ptr_t elem = list->impl.front();
  list->impl.pop_front();
  return elem;
}

size_t cdi_list_empty(cdi_list_t list) {
  return list->impl.empty();
}

ptr_t cdi_list_get(cdi_list_t list, size_t index) {
  for (auto it = list->impl.begin(); it != list->impl.end(); ++it) {
    if (index == 0) return *it;
    index -= 1;
  }
  return nullptr;
}

cdi_list_t cdi_list_insert(cdi_list_t list, size_t index, ptr_t value) {
  for (auto it = list->impl.begin(); it != list->impl.end(); ++it) {
    if (index == 0) {
      list->impl.insert(it, value);
      return list;
    }
    index -= 1;
  }
  return nullptr;
}

ptr_t cdi_list_remove(cdi_list_t list, size_t index) {
  for (auto it = list->impl.begin(); it != list->impl.end(); ++it) {
    if (index == 0) {
      ptr_t elem = *it;
      list->impl.erase(it);
      return elem;
    }
    index -= 1;
  }
  return nullptr;
}

size_t cdi_list_size(cdi_list_t list) {
  return list->impl.size();
}
