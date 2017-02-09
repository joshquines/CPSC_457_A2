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
#include "kernel/Multiboot.h"
#include "kernel/Output.h"
#include "world/Access.h"

#include "extern/multiboot/multiboot2.h"

// cf. 'multiboot_mmap_entry' in extern/multiboot/multiboot2.h
static const char* memtype[] __section(".boot.data") = {
  "unknown", "free", "resv", "acpi", "nvs", "bad"
};

#define FORALLTAGS(tag,start,end) \
  for (multiboot_tag* tag = (multiboot_tag*)(start); \
  vaddr(tag) < (end) && tag->type != MULTIBOOT_TAG_TYPE_END; \
  tag = (multiboot_tag*)(vaddr(tag) + ((tag->size + 7) & ~7)))

vaddr Multiboot::mbiStart = 0;
vaddr Multiboot::mbiEnd = 0;

void Multiboot::initDebug( bool msg ) {
  FORALLTAGS(tag,mbiStart,mbiEnd) {
    if (tag->type == MULTIBOOT_TAG_TYPE_CMDLINE) {
      DBG::init( ((multiboot_tag_string*)tag)->string, msg );
    }
  }
}

vaddr Multiboot::init( mword magic, vaddr mbi ) {
  KASSERT1(magic == MULTIBOOT2_BOOTLOADER_MAGIC, magic);
  KASSERT1( !(mbi & 7), FmtHex(mbi) );
  // mbiEnd = mbi + ((multiboot_header_tag*)mbi)->size;
  mbiStart = mbi + sizeof(multiboot_header_tag);
  mbiEnd = mbi + *(uint32_t*)mbi;
  initDebug(false);
  vaddr modEnd = 0;
  FORALLTAGS(tag,mbiStart,mbiEnd) {
    if (tag->type == MULTIBOOT_TAG_TYPE_MODULE) {
      multiboot_tag_module* tm = (multiboot_tag_module*)tag;
      if (tm->mod_end > modEnd) modEnd = tm->mod_end;
    }
  }
  return max(mbiEnd, modEnd);
}

void Multiboot::init2() {
  DBG::outl(DBG::Basic, "************ DEBUG *************");
  initDebug(true);
  DBG::outl(DBG::Basic, "************* MBI **************");
  FORALLTAGS(tag,mbiStart,mbiEnd) {
    switch (tag->type) {
    case MULTIBOOT_TAG_TYPE_CMDLINE:
      DBG::outl(DBG::Boot, "command line: ", ((multiboot_tag_string*)tag)->string);
      break;
    case MULTIBOOT_TAG_TYPE_BOOT_LOADER_NAME:
      DBG::outl(DBG::Boot, "boot loader: ", ((multiboot_tag_string*)tag)->string);
      break;
    case MULTIBOOT_TAG_TYPE_MODULE: {
      multiboot_tag_module* tm = (multiboot_tag_module*)tag;
      DBG::outl(DBG::Boot, "module at ", FmtHex(tm->mod_start), '-', FmtHex(tm->mod_end), ": ", tm->cmdline);
    } break;
    case MULTIBOOT_TAG_TYPE_BASIC_MEMINFO: {
      multiboot_tag_basic_meminfo* tm = (multiboot_tag_basic_meminfo*)tag;
      DBG::outl(DBG::Boot, "memory low: ", tm->mem_lower, " / high: ", tm->mem_upper);
    } break;
    case MULTIBOOT_TAG_TYPE_BOOTDEV: {
      multiboot_tag_bootdev* tb = (multiboot_tag_bootdev*)tag;
      DBG::outl(DBG::Boot, "boot device: ", FmtHex(tb->biosdev), ',', tb->slice, ',', tb->part);
    } break;
    case MULTIBOOT_TAG_TYPE_MMAP: {
      multiboot_tag_mmap* tmm = (multiboot_tag_mmap*)tag;
      vaddr end = vaddr(tmm) + tmm->size;
      DBG::out1(DBG::Boot, "mmap:");
      for (mword mm = (mword)tmm->entries; mm < end; mm += tmm->entry_size ) {
        multiboot_memory_map_t* mmap = (multiboot_memory_map_t*)mm;
        KASSERT1(mmap->type <= MULTIBOOT_MEMORY_BADRAM, mmap->type);
        DBG::out1(DBG::Boot, ' ', FmtHex(mmap->addr), '/', FmtHex(mmap->len), '/', memtype[mmap->type]);
      }
      DBG::outl(DBG::Boot);
    } break;
    case MULTIBOOT_TAG_TYPE_FRAMEBUFFER:
      DBG::outl(DBG::Boot, "framebuffer info present");
      break;
    case MULTIBOOT_TAG_TYPE_VBE:
      DBG::outl(DBG::Boot, "vbe info present");
      break;
    case MULTIBOOT_TAG_TYPE_ELF_SECTIONS:
      DBG::outl(DBG::Boot, "elf section info present");
      break;
    case MULTIBOOT_TAG_TYPE_APM:
      DBG::outl(DBG::Boot, "APM info present");
      break;
    case MULTIBOOT_TAG_TYPE_EFI32:
      DBG::outl(DBG::Boot, "efi32 info present");
      break;
    case MULTIBOOT_TAG_TYPE_EFI64:
      DBG::outl(DBG::Boot, "efi64 info present");
      break;
    case MULTIBOOT_TAG_TYPE_SMBIOS:
      DBG::outl(DBG::Boot, "smbios info present");
      break;
    case MULTIBOOT_TAG_TYPE_ACPI_OLD: {
      multiboot_tag_old_acpi* ta = (multiboot_tag_old_acpi*)tag;
      DBG::outl(DBG::Boot, "acpi/old: ", FmtHex(ta->rsdp), '/', FmtHex(ta->size));
    } break;
    case MULTIBOOT_TAG_TYPE_ACPI_NEW: {
      multiboot_tag_new_acpi* ta = (multiboot_tag_new_acpi*)tag;
      DBG::outl(DBG::Boot, "acpi/new: ", FmtHex(ta->rsdp), '/', FmtHex(ta->size));
    } break;
    case MULTIBOOT_TAG_TYPE_NETWORK:
      DBG::outl(DBG::Boot, "network info present");
      break;
    default:
      DBG::outl(DBG::Boot, "unknown tag: ", tag->type);
    }
  }
}

void Multiboot::remap(vaddr disp) {
  mbiStart += disp;
  mbiEnd += disp;
}

vaddr Multiboot::getRSDP() {
  FORALLTAGS(tag,mbiStart,mbiEnd) {
    switch (tag->type) {
    case MULTIBOOT_TAG_TYPE_ACPI_OLD:
      return vaddr(((multiboot_tag_old_acpi*)tag)->rsdp);
    case MULTIBOOT_TAG_TYPE_ACPI_NEW:
      return vaddr(((multiboot_tag_new_acpi*)tag)->rsdp);
    }
  }
  KABORT1("RSDP not found");
  return 0;
}

void Multiboot::getMemory(RegionSet<Region<paddr>>& rs) {
  FORALLTAGS(tag,mbiStart,mbiEnd) {
    if (tag->type == MULTIBOOT_TAG_TYPE_MMAP) {
      multiboot_tag_mmap* tm = (multiboot_tag_mmap*)tag;
      vaddr end = vaddr(tm) + tm->size;
      for (vaddr mm = (paddr)tm->entries; mm < end; mm += tm->entry_size ) {
        multiboot_memory_map_t* mmap = (multiboot_memory_map_t*)mm;
        if (mmap->type == MULTIBOOT_MEMORY_AVAILABLE && mmap->len > 0) {
          rs.insert( Region<paddr>(mmap->addr, mmap->addr + mmap->len) );
        }
      }
    }
  }
}

void Multiboot::readModules(vaddr disp) {
  FORALLTAGS(tag,mbiStart,mbiEnd) {
    if (tag->type == MULTIBOOT_TAG_TYPE_MODULE) {
      multiboot_tag_module* tm = (multiboot_tag_module*)tag;
      string cmd = tm->cmdline;
      string name = cmd.substr(0, cmd.find_first_of(' '));
      kernelFS.insert( {name, {tm->mod_start + disp, tm->mod_start, tm->mod_end - tm->mod_start}} );
    }
  }
}
