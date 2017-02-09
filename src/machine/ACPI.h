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
extern "C" {
#include "extern/acpica/source/include/acpi.h"
}
#include "runtime/BlockingSync.h"

#include <set>

static_assert( bool(ACPI_INTERRUPT_HANDLED) == true, "ACPI_INTERRUPT_HANDLED" );
static_assert( bool(ACPI_INTERRUPT_NOT_HANDLED) == false, "ACPI_INTERRUPT_NOT_HANDLED" );

static vaddr rsdp = 0;                // AcpiOsGetRootPointer callback

// double-check ACPI dynamic allocations & mappings
static set<vaddr> allocations;
static set<pair<vaddr,size_t>> mappings;

#define ACPICALL(func) {\
  ACPI_STATUS status = func;\
  KASSERT1(status == AE_OK, status);\
}

static paddr initACPI(vaddr r, map<uint32_t,uint32_t>&, map<uint32_t,paddr>&,
  map<uint8_t,pair<uint32_t,uint16_t>>&)                                __section(".boot.text");
static void initACPI2()                                                 __section(".boot.text");
static ACPI_DEVICE_INFO* acpiGetInfo(ACPI_HANDLE)                       __section(".boot.text");
static ACPI_STATUS walkHandler(ACPI_HANDLE, UINT32, void*, void**)      __section(".boot.text");
static ACPI_STATUS initHandler(ACPI_HANDLE, UINT32)                     __section(".boot.text");
static ACPI_STATUS pciRootBridge(ACPI_HANDLE, UINT32, void*, void**)    __section(".boot.text");
static ACPI_STATUS displayResource(ACPI_RESOURCE*, void*)               __section(".boot.text");
static ACPI_STATUS pciInterruptLink(ACPI_HANDLE, UINT32, void*, void**) __section(".boot.text");

static paddr initACPI(vaddr r, map<uint32_t,uint32_t>& apicMap,
  map<uint32_t,paddr>& ioApicMap,
  map<uint8_t,pair<uint32_t,uint16_t>>& ioOverrideMap ) {

  rsdp = r;                           // set up for acpica callback

  ACPICALL(AcpiInitializeTables(0, 0, true)); // get tables w/o dynamic memory

  acpi_table_fadt* fadt;              // FADT reports PS/2 keyboard
  ACPICALL(AcpiGetTable((char*)ACPI_SIG_FADT, 0, (ACPI_TABLE_HEADER**)&fadt));
  DBG::out1(DBG::Acpi, "FADT: ", fadt->Header.Length, '/', FmtHex(fadt->BootFlags), '/', FmtHex(fadt->Flags));
  if (fadt->BootFlags & ACPI_FADT_8042) DBG::out1(DBG::Acpi, " - 8042");
  // PS/2 driver is enabled in initBSP2, regardless of what ACPI reports
  DBG::outl(DBG::Acpi);

  acpi_table_srat* srat;
  if (AcpiGetTable((char*)ACPI_SIG_SRAT, 0, (ACPI_TABLE_HEADER**)&srat ) == AE_OK) {
    DBG::outl(DBG::Acpi, "SRAT: ", srat->Header.Length);
  }

  acpi_table_slit* slit;
  if (AcpiGetTable((char*)ACPI_SIG_SLIT, 0, (ACPI_TABLE_HEADER**)&slit ) == AE_OK) {
    DBG::outl(DBG::Acpi, "SLIT: ", slit->Header.Length);
  }

  acpi_table_madt* madt;              // MADT reports PIC, APICs, IOAPICS
  ACPICALL(AcpiGetTable((char*)ACPI_SIG_MADT, 0, (ACPI_TABLE_HEADER**)&madt));
  mword madtLength = madt->Header.Length - sizeof(acpi_table_madt);
  paddr apicPhysAddr = madt->Address;
  DBG::out1(DBG::Acpi, "MADT: ", madtLength, '/', FmtHex(apicPhysAddr));
  if (madt->Flags & ACPI_MADT_PCAT_COMPAT) DBG::out1(DBG::Acpi, " - 8259");
  DBG::outl(DBG::Acpi);
  PIC::disable();                     // disable 8259 PIC unconditionally

  DBG::out1(DBG::Acpi, "MADT:");
  // walk through subtables and gather information in temporary containers
  acpi_subtable_header* subtable = (acpi_subtable_header*)(madt + 1);
  
  while (madtLength > 0) {
    KASSERTN(subtable->Length <= madtLength, subtable->Length, '/', madtLength);
    switch (subtable->Type) {
    case ACPI_MADT_TYPE_LOCAL_APIC: {
      acpi_madt_local_apic* la = (acpi_madt_local_apic*)subtable;
      DBG::out1(DBG::Acpi, " CPU:", mword(la->ProcessorId), '/', mword(la->Id));
      if (!(la->LapicFlags & 0x1)) DBG::out1(DBG::Acpi, 'X');
      if (la->LapicFlags & 0x1) apicMap.insert( {la->ProcessorId, la->Id} );
    } break;
    case ACPI_MADT_TYPE_IO_APIC: {
      acpi_madt_io_apic* io = (acpi_madt_io_apic*)subtable;
      DBG::out1(DBG::Acpi, " IO_APIC:", io->GlobalIrqBase, '@', FmtHex(io->Address));
      ioApicMap.insert( {io->GlobalIrqBase, (paddr)io->Address} );
    } break;
    case ACPI_MADT_TYPE_INTERRUPT_OVERRIDE: {
      acpi_madt_interrupt_override* io = (acpi_madt_interrupt_override*)subtable;
      DBG::out1(DBG::Acpi, " INTERRUPT_OVERRIDE:", FmtHex(io->SourceIrq), "->", FmtHex(io->GlobalIrq), '/', FmtHex(io->IntiFlags));
      ioOverrideMap.insert( {io->SourceIrq, {io->GlobalIrq, io->IntiFlags}} );
    } break;
    case ACPI_MADT_TYPE_NMI_SOURCE:                // TODO: mark irq as unavailable
      DBG::out1(DBG::Acpi, " NMI_SOURCE"); break;
    case ACPI_MADT_TYPE_LOCAL_APIC_NMI: {          // list of APICs that receive NMIs
      acpi_madt_local_apic_nmi* ln = (acpi_madt_local_apic_nmi*)subtable;
      DBG::out1(DBG::Acpi, " LOCAL_APIC_NMI:", int(ln->ProcessorId), '/', int(ln->IntiFlags), '/', int(ln->Lint) );
    } break;
    case ACPI_MADT_TYPE_LOCAL_APIC_OVERRIDE: {
      acpi_madt_local_apic_override* ao = (acpi_madt_local_apic_override*)subtable;
      DBG::out1(DBG::Acpi, " LOCAL_APIC_OVERRIDE:", FmtHex(ao->Address) );
      apicPhysAddr = ao->Address;
      } break;
    case ACPI_MADT_TYPE_IO_SAPIC:
      DBG::out1(DBG::Acpi, " IO_SAPIC"); break;
    case ACPI_MADT_TYPE_LOCAL_SAPIC:
      DBG::out1(DBG::Acpi, " LOCAL_SAPIC"); break;
    case ACPI_MADT_TYPE_INTERRUPT_SOURCE:
      DBG::out1(DBG::Acpi, " INTERRUPT_SOURCE"); break;
    case ACPI_MADT_TYPE_LOCAL_X2APIC:
      DBG::out1(DBG::Acpi, " LOCAL_X2APIC"); break;
    case ACPI_MADT_TYPE_LOCAL_X2APIC_NMI:
      DBG::out1(DBG::Acpi, " LOCAL_X2APIC_NMI"); break;
    case ACPI_MADT_TYPE_GENERIC_INTERRUPT:
      DBG::out1(DBG::Acpi, " GENERIC_INTERRUPT"); break;
    case ACPI_MADT_TYPE_GENERIC_DISTRIBUTOR:
      DBG::out1(DBG::Acpi, " GENERIC_DISTRIBUTOR"); break;
    default: KABORT1("unknown ACPI MADT subtable");
    }
    madtLength -= subtable->Length;
    subtable = (acpi_subtable_header*)(((char*)subtable) + subtable->Length);
  }
  DBG::outl(DBG::Acpi);
  return apicPhysAddr;
}

static ACPI_DEVICE_INFO* acpiGetInfo(ACPI_HANDLE object) {
  ACPI_DEVICE_INFO* info;
  ACPICALL(AcpiGetObjectInfo(object, &info));

  char buffer[256];
  ACPI_BUFFER path;
  path.Pointer = buffer;
  path.Length = sizeof(buffer);
  ACPICALL(AcpiGetName(object, ACPI_FULL_PATHNAME, &path));

  AcpiOsPrintf("%s HID: %s, ADR: %08X, Status: %08X\n", (char*)path.Pointer,
    info->HardwareId.String, info->Address, info->CurrentStatus);

  return info;
}

static ACPI_STATUS walkHandler(ACPI_HANDLE object, UINT32 level, void* ctx, void** retval) {
  AcpiOsPrintf("WALK ");
  ACPI_DEVICE_INFO* info = acpiGetInfo(object);
  AcpiOsFree(info);
  return AE_OK;
}

static ACPI_STATUS initHandler(ACPI_HANDLE object, UINT32 func) {
  AcpiOsPrintf("INIT ");
  ACPI_DEVICE_INFO* info = acpiGetInfo(object);
  AcpiOsFree(info);
  return AE_OK;
}

static ACPI_STATUS pciRootBridge(ACPI_HANDLE object, UINT32 level, void* ctx, void** retval) {
  AcpiOsPrintf("BRDG ");
  ACPI_DEVICE_INFO* info = acpiGetInfo(object);
  KASSERT1(info->Flags == ACPI_PCI_ROOT_BRIDGE, info->Flags);
  ACPI_BUFFER buffer;
  buffer.Length = ACPI_ALLOCATE_BUFFER;
  buffer.Pointer = nullptr;
  ACPI_STATUS status = AcpiGetIrqRoutingTable(object, &buffer);
  if (status == AE_OK) {
    ACPI_PCI_ROUTING_TABLE* rt = (ACPI_PCI_ROUTING_TABLE*)buffer.Pointer;
    while (rt->Length != 0) {
      mword devnum = (rt->Address & 0xFFFF0000) >> 16; // ACPI spec Sec 6.1.1
      mword func = (rt->Address & 0x0000FFFF);
      KASSERT1(func == 0xFFFF, FmtHex(func));
      AcpiOsPrintf("IRQ RT: Dev 0x%02X/%c -> %s/%d\n", devnum, 'A'+rt->Pin, rt->Source, rt->SourceIndex);
      rt = (ACPI_PCI_ROUTING_TABLE*)((char*)rt + rt->Length);
    }
    AcpiOsFree(buffer.Pointer);
  } else {
    AcpiOsPrintf("no IRQ routing info: %d\n", status);
  }
  AcpiOsFree(info);
  return AE_OK;
}

static ACPI_STATUS displayResource(ACPI_RESOURCE* resource, void* ctx) {
  if (resource->Type == ACPI_RESOURCE_TYPE_IRQ) {
    ACPI_RESOURCE_IRQ& r = resource->Data.Irq;
    AcpiOsPrintf("Len 0x%X Trig 0x%X Pol 0x%X Shar 0x%X Wake 0x%X Cnt 0x%X",
      r.DescriptorLength, r.Triggering, r.Polarity, r.Sharable, r.WakeCapable, r.InterruptCount);
    AcpiOsPrintf(" IRQs:");
    for (int i = 0; i < r.InterruptCount; i++) {
      AcpiOsPrintf(" 0x%02X", r.Interrupts[i]);
    }
    AcpiOsPrintf("\n");
  } else if (resource->Type == ACPI_RESOURCE_TYPE_EXTENDED_IRQ) {
    ACPI_RESOURCE_EXTENDED_IRQ& r = resource->Data.ExtendedIrq;
    AcpiOsPrintf("P/C 0x%X Trig 0x%X Pol 0x%X Shar 0x%X Wake 0x%X Cnt 0x%X",
      r.ProducerConsumer, r.Triggering, r.Polarity, r.Sharable, r.WakeCapable, r.InterruptCount);
    if (r.ResourceSource.StringLength) AcpiOsPrintf(" Idx %d Source %s", r.ResourceSource.Index, r.ResourceSource.StringPtr);
    AcpiOsPrintf(" IRQs:");
    for (int i = 0; i < r.InterruptCount; i++) {
      AcpiOsPrintf(" 0x%02X", r.Interrupts[i]);
    }
    AcpiOsPrintf("\n");
  } else {
    AcpiOsPrintf("ACPI_RESOURCE_TYPE: %d\n", resource->Type);
  }
  return AE_OK;
}

static ACPI_STATUS pciInterruptLink(ACPI_HANDLE object, UINT32 level, void* ctx, void** retval) {
  AcpiOsPrintf("INTL ");
  ACPI_DEVICE_INFO* info = acpiGetInfo(object);
#if 0
  ACPICALL(AcpiWalkResources(object, (char*)"_CRS", displayResource, NULL));
#else
  ACPI_BUFFER buffer;
  buffer.Length = ACPI_ALLOCATE_BUFFER;
  buffer.Pointer = nullptr;
  ACPICALL(AcpiGetCurrentResources(object, &buffer));
  ACPI_RESOURCE* resource = (ACPI_RESOURCE*)buffer.Pointer;
  while (resource->Type != ACPI_RESOURCE_TYPE_END_TAG) {
    displayResource(resource, ctx);
    resource = (ACPI_RESOURCE*)((char*)resource + resource->Length);
  }
  AcpiOsFree(buffer.Pointer);
#endif
  AcpiOsFree(info);
  return AE_OK;
}

static void initACPI2() { // init sequence from Sec 10.1 of ACPICA documentation
  ACPICALL(AcpiInitializeSubsystem());
//  ACPICALL(AcpiReallocateRootTable(); // gives AE_SUPPORT, not supported
  ACPICALL(AcpiLoadTables());
  ACPICALL(AcpiEnableSubsystem(ACPI_FULL_INITIALIZATION));
  ACPICALL(AcpiInstallInitializationHandler(initHandler, 0));
  ACPICALL(AcpiInitializeObjects(ACPI_FULL_INITIALIZATION));

//  ACPICALL(AcpiWalkNamespace(ACPI_TYPE_ANY, ACPI_ROOT_OBJECT, ACPI_UINT32_MAX, walkHandler, NULL, NULL, NULL));
  ACPICALL(AcpiGetDevices((char*)"PNP0C0F", pciInterruptLink, NULL, NULL));
  ACPICALL(AcpiGetDevices((char*)"PNP0A03", pciRootBridge, NULL, NULL)); // or PCI_ROOT_HID_STRING

  // TODO: in principle, the ACPI subsystem should keep running...
  ACPICALL(AcpiTerminate());

  // check for ACPI memory leaks
  KASSERT0(allocations.empty());
  KASSERT0(mappings.empty());
}

ACPI_STATUS AcpiOsInitialize(void) { return AE_OK; }

ACPI_STATUS AcpiOsTerminate(void) { return AE_OK; }

ACPI_PHYSICAL_ADDRESS AcpiOsGetRootPointer(void) { return rsdp; }

ACPI_STATUS AcpiOsPredefinedOverride(const ACPI_PREDEFINED_NAMES* InitVal,
  ACPI_STRING* NewVal) {
  *NewVal = nullptr;
  return AE_OK;
}

ACPI_STATUS AcpiOsTableOverride(ACPI_TABLE_HEADER* ExistingTable,
  ACPI_TABLE_HEADER** NewTable) {
  *NewTable = nullptr;
  return AE_OK;
}

ACPI_STATUS AcpiOsPhysicalTableOverride(ACPI_TABLE_HEADER* ExistingTable,
  ACPI_PHYSICAL_ADDRESS* NewAddress, UINT32* NewTableLength) {
  *NewAddress = 0;
  *NewTableLength = 0;
  return AE_OK;
}

ACPI_STATUS AcpiOsCreateLock(ACPI_SPINLOCK* OutHandle) {
  *OutHandle = knew<SpinLock>();
  return AE_OK;
}

void AcpiOsDeleteLock(ACPI_SPINLOCK Handle) {
  kdelete(reinterpret_cast<SpinLock*>(Handle));
}

ACPI_CPU_FLAGS AcpiOsAcquireLock(ACPI_SPINLOCK Handle) {
  reinterpret_cast<SpinLock*>(Handle)->acquire();
  return 0;
}

void AcpiOsReleaseLock(ACPI_SPINLOCK Handle, ACPI_CPU_FLAGS Flags) {
  reinterpret_cast<SpinLock*>(Handle)->release();
}

ACPI_STATUS AcpiOsCreateSemaphore(UINT32 MaxUnits, UINT32 InitialUnits,
  ACPI_SEMAPHORE* OutHandle) {
  *OutHandle = knew<Semaphore>(InitialUnits);
  return AE_OK;
}

ACPI_STATUS AcpiOsDeleteSemaphore(ACPI_SEMAPHORE Handle) {
  kdelete(reinterpret_cast<Semaphore*>(Handle));
  return AE_OK;
}

ACPI_STATUS AcpiOsWaitSemaphore(ACPI_SEMAPHORE Handle, UINT32 Units,
  UINT16 Timeout) {

  KASSERT1(Timeout == 0xFFFF || Timeout == 0, Timeout);
  for (UINT32 x = 0; x < Units; x += 1) reinterpret_cast<Semaphore*>(Handle)->P();
  return AE_OK;
}

ACPI_STATUS AcpiOsSignalSemaphore(ACPI_SEMAPHORE Handle, UINT32 Units) {
  for (UINT32 x = 0; x < Units; x += 1) reinterpret_cast<Semaphore*>(Handle)->V();
  return AE_OK;
}

#if 0
ACPI_STATUS AcpiOsCreateMutex(ACPI_MUTEX* OutHandle) {
  KABORT1(false,"");
  return AE_ERROR;
}

void AcpiOsDeleteMutex(ACPI_MUTEX Handle) { KABORT1(false,""); }

ACPI_STATUS AcpiOsAcquireMutex(ACPI_MUTEX Handle, UINT16 Timeout) {
  KABORT1(false,"");
  return AE_ERROR;
}

void AcpiOsReleaseMutex(ACPI_MUTEX Handle) { KABORT1(false,""); }
#endif

void* AcpiOsAllocate(ACPI_SIZE Size) {
  DBG::outl(DBG::MemAcpi, "acpi alloc: ", Size);
  void* ptr = malloc(Size);
  memset(ptr, 0, Size); // zero out memory to play it safe...
  allocations.insert( vaddr(ptr) );
  return ptr;
}

void AcpiOsFree(void* Memory) {
  DBG::outl(DBG::MemAcpi, "acpi free: ", Memory);
  allocations.erase( vaddr(Memory) );
  free(Memory);
}

void* AcpiOsMapMemory(ACPI_PHYSICAL_ADDRESS Where, ACPI_SIZE Length) {
  DBG::outl(DBG::MemAcpi, "acpi map: ", FmtHex(Where), '/', FmtHex(Length));
  paddr pma = align_down(paddr(Where), pagesize<1>());
  size_t size = align_up(paddr(Where) + Length, pagesize<1>()) - pma;
  vaddr vma = kernelSpace.kmap<1,false>( 0, size, pma );
  mappings.insert( { vma,size} );
  return (void*)(vma + Where - pma);
}

void AcpiOsUnmapMemory(void* LogicalAddress, ACPI_SIZE Size) {
  DBG::outl(DBG::MemAcpi, "acpi unmap: ", FmtHex(LogicalAddress), '/', FmtHex(Size));
  vaddr vma = align_down(vaddr(LogicalAddress), pagesize<1>());
  size_t size = align_up(Size + vaddr(LogicalAddress) - vma, pagesize<1>());
  kernelSpace.unmap<1,false>( vma, size );
  mappings.erase( {vma, size} );
}

ACPI_STATUS AcpiOsGetPhysicalAddress(void* LogicalAddress,
  ACPI_PHYSICAL_ADDRESS* PhysicalAddress) {
  *PhysicalAddress = Paging::vtop(vaddr(LogicalAddress));
  return AE_OK;
}

// the cache mechanism is bypassed in favour of regular allocation
// the object size is encoded in fake cache pointer
ACPI_STATUS AcpiOsCreateCache(char* CacheName, UINT16 ObjectSize,
  UINT16 MaxDepth, ACPI_CACHE_T** ReturnCache) {
  *ReturnCache = (ACPI_CACHE_T*)(uintptr_t)ObjectSize;
  return AE_OK;
}

ACPI_STATUS AcpiOsDeleteCache(ACPI_CACHE_T* Cache) {
  return AE_OK;
}

ACPI_STATUS AcpiOsPurgeCache(ACPI_CACHE_T* Cache) {
  return AE_OK;
}

void* AcpiOsAcquireObject(ACPI_CACHE_T* Cache) {
  void* addr = (void*)MemoryManager::alloc((UINT16)(uintptr_t)Cache);
  memset((void*)addr, 0, (UINT16)(uintptr_t)Cache);
  return (void*)addr;
}

ACPI_STATUS AcpiOsReleaseObject(ACPI_CACHE_T* Cache, void* Object) {
  MemoryManager::release((vaddr)Object, (UINT16)(uintptr_t)Cache);
  return AE_OK;
}

ACPI_STATUS AcpiOsInstallInterruptHandler(UINT32 InterruptNumber,
  ACPI_OSD_HANDLER ServiceRoutine, void* Context) {

  DBG::outl(DBG::Acpi, "ACPI install intr handler: ", InterruptNumber);
  Machine::registerIrqAsync(InterruptNumber, (funcvoid1_t)ServiceRoutine, Context);
  return AE_OK;
}

ACPI_STATUS AcpiOsRemoveInterruptHandler(UINT32 InterruptNumber,
  ACPI_OSD_HANDLER ServiceRoutine) {

  DBG::outl(DBG::Acpi, "ACPI remove intr handler: ", InterruptNumber);
  Machine::deregisterIrqAsync(InterruptNumber, (funcvoid1_t)ServiceRoutine);
  return AE_OK;
}

ACPI_THREAD_ID AcpiOsGetThreadId(void) {
  return (ACPI_THREAD_ID)LocalProcessor::getCurrThread();
}

ACPI_STATUS AcpiOsExecute(ACPI_EXECUTE_TYPE Type,
  ACPI_OSD_EXEC_CALLBACK Function, void* Context) {

  KABORT0();
  return AE_ERROR;
}

void AcpiOsWaitEventsComplete(void* Context) { KABORT0(); }

void AcpiOsSleep(UINT64 Milliseconds) { Timeout::sleep(Milliseconds); } // HP netbook

void AcpiOsStall(UINT32 Microseconds) { Clock::wait(Microseconds/1000); } // AMD 2427

void AcpiOsWaitEventsComplete() { KABORT0(); }

ACPI_STATUS AcpiOsReadPort(ACPI_IO_ADDRESS Address, UINT32* Value, UINT32 Width) {
  DBG::outl(DBG::Acpi, "AcpiOsReadPort: ", FmtHex(Address), '/', Width);
  switch (Width) {
    case  8: *Value = CPU::in8 (Address); break;
    case 16: *Value = CPU::in16(Address); break;
    case 32: *Value = CPU::in32(Address); break;
    default: return AE_ERROR;
  }
  return AE_OK;
}

ACPI_STATUS AcpiOsWritePort(ACPI_IO_ADDRESS Address, UINT32 Value, UINT32 Width) {
  DBG::outl(DBG::Acpi, "AcpiOsWritePort: ", FmtHex(Address), '/', Width, ':', FmtHex(Value));
  switch (Width) {
    case  8: CPU::out8 (Address,Value); break;
    case 16: CPU::out16(Address,Value); break;
    case 32: CPU::out32(Address,Value); break;
    default: return AE_ERROR;
  }
  return AE_OK;
}

ACPI_STATUS AcpiOsReadMemory(ACPI_PHYSICAL_ADDRESS Address, UINT64* Value, UINT32 Width) {
  KABORT0();
  return AE_ERROR;
}

ACPI_STATUS AcpiOsWriteMemory(ACPI_PHYSICAL_ADDRESS Address, UINT64 Value, UINT32 Width) {
  KABORT0();
  return AE_ERROR;
}

ACPI_STATUS AcpiOsReadPciConfiguration( ACPI_PCI_ID* PciId, UINT32 Reg, UINT64* Value, UINT32 Width) {
  switch (Width) {
    case  8: *Value = PCI::readConfig< 8>(PciId->Bus, PciId->Device, PciId->Function, Reg); break;
    case 16: *Value = PCI::readConfig<16>(PciId->Bus, PciId->Device, PciId->Function, Reg); break;
    case 32: *Value = PCI::readConfig<32>(PciId->Bus, PciId->Device, PciId->Function, Reg); break;
    default: KABORT1(Width);
  }
  return AE_OK;
}

ACPI_STATUS AcpiOsWritePciConfiguration(ACPI_PCI_ID* PciId, UINT32 Reg, UINT64 Value, UINT32 Width) {
  switch (Width) {
    case  8: PCI::writeConfig< 8>(PciId->Bus, PciId->Device, PciId->Function, Reg, Value); break;
    case 16: PCI::writeConfig<16>(PciId->Bus, PciId->Device, PciId->Function, Reg, Value); break;
    case 32: PCI::writeConfig<32>(PciId->Bus, PciId->Device, PciId->Function, Reg, Value); break;
    default: KABORT1(Width);
  }
  return AE_OK;
}

BOOLEAN AcpiOsReadable(void* Pointer, ACPI_SIZE Length) {
  KABORT0();
  return false;
}

BOOLEAN AcpiOsWritable(void* Pointer, ACPI_SIZE Length) {
  KABORT0();
  return false;
}

UINT64 AcpiOsGetTimer(void) {
  KABORT0();
  return 0;
}

ACPI_STATUS AcpiOsSignal(UINT32 Function, void* Info) {
  KABORT0();
  return AE_ERROR;
}

void ACPI_INTERNAL_VAR_XFACE AcpiOsPrintf(const char* Format, ...) {
  va_list args;
  va_start(args, Format);
  AcpiOsVprintf(Format, args);
  va_end(args);
}

void AcpiOsVprintf(const char* fmt, va_list args) {
  ExternDebugPrintf(DBG::Acpi, fmt, args);
}

void AcpiOsRedirectOutput(void* Destination) { KABORT0(); }

ACPI_STATUS AcpiOsGetLine(char* Buffer, UINT32 BufferLength, UINT32* BytesRead) {
  KABORT0();
  return AE_ERROR;
}

void* AcpiOsOpenDirectory(char* Pathname, char* WildcardSpec, char RequestedFileType) {
  KABORT0();
  return nullptr;
}

char* AcpiOsGetNextFilename(void* DirHandle) {
  KABORT0();
  return nullptr;
}

void AcpiOsCloseDirectory(void* DirHandle) { KABORT0(); }
