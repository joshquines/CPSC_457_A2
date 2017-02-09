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
#ifndef _Memory_h_
#define _Memory_h_ 1

#include "generic/bitmanip.h"
#include "machine/asmshare.h"

/* Typical (old style) PC memory map
      0		  3FF 	Interrupt Vectors (IVT)
    400		  500	BIOS data area(BDA)
    500		9FBFF	free (INT 0x12 with return AX KB blocks, or use E820)
  9FC00		9FFFF	extended BIOS data area (EBDA)
  A0000 	BFFFF	VGA framebuffers
  C0000		C7FFF	video BIOS
  C8000		EFFFF	nothing
  F0000		FFFFF	motherboard BIOS
 100000 .....		mostly free (1MB and above), some exceptions apply */

#define __caligned   __attribute__((__aligned__(64)))

static const mword topaddr = limit<mword>();

static const size_t pageoffsetbits   = 12;
static const size_t pagetablebits    = 9;
static const size_t pagetableentries = 1 << pagetablebits;
static const size_t pagelevels       = 4;
static const size_t pagetablepl      = 1;
static const size_t framebits        = pageoffsetbits + 40;
static const size_t pagebits         = pageoffsetbits + pagetablebits * pagelevels;

template<unsigned int N>
static constexpr size_t pagesizebits() {
  static_assert( N <= pagelevels + 1, "page level template violation" );
  return pageoffsetbits + (N-1) * pagetablebits;
}

template<unsigned int N>
static constexpr size_t pagesize() {
  static_assert( N <= pagelevels, "page level template violation" );
  return pow2<size_t>(pagesizebits<N>());
}

template<unsigned int N>
static constexpr size_t pageoffset(mword addr) {
  static_assert( N <= pagelevels, "page level template violation" );
  return addr & bitmask<mword>(pagesizebits<N>());
}

/**** Memory Constants and Layout ****/

// kernel code/data needs to be at upper 2G for mcmodel=kernel
// use first entry in upper half of page tables for recursive page directories
// use start of last 512G paging region for device mappings
// use previous three 512G paging regions for kernel dynamic memory
static const mword  usertindex   = pagetableentries / 2;
static const mword  recptindex   = pagetableentries / 2;
static const mword  kernbindex   = pagetableentries - 4;
static const mword  kerntindex   = pagetableentries - 1;
static const mword  devptindex   = pagetableentries - 1;
static const vaddr  kernelBase   = vaddr(KERNBASE);

// basic memory layout
static const mword  canonTest    =  pow2<mword>(pagebits - 1);
static const mword  canonPrefix  = ~bitmask<mword>(pagebits);
static const vaddr  userbot      =  pagesize<2>();
static const vaddr  usertop      = (pagesize<pagelevels>() * usertindex);
static const vaddr  kernelbot    = (pagesize<pagelevels>() * kernbindex) | canonPrefix;
static const vaddr  kerneltop    = (pagesize<pagelevels>() * kerntindex) | canonPrefix;
static const vaddr  deviceAddr   = (pagesize<pagelevels>() * devptindex) | canonPrefix;
static_assert(deviceAddr >= kerneltop, "deviceAddr < kerneltop");

// hard-coded device & special mappings -> at the very end virtual memory
static const vaddr   apicAddr    = deviceAddr + 0 * pagesize<1>();
static const vaddr ioApicAddr    = deviceAddr + 1 * pagesize<1>();
static const vaddr  videoAddr    = deviceAddr + 2 * pagesize<1>();
static const vaddr  cloneAddr    = deviceAddr + 3 * pagesize<1>();
static const vaddr  deviceEnd    = deviceAddr + 4 * pagesize<1>();
static_assert(kernelBase >= deviceEnd, "KERNBASE < deviceEnd");

// kernel and bootstrap constants
static const size_t kernelpl     = 2;
static const size_t kernelps     = pagesize<kernelpl>();
static const size_t bootHeapSize = 16 * pagesize<1>();
static const size_t bootHeapMap  = kerneltop - pagesize<3>();
static const size_t bootSizePT   = (4 + divup(mword(MAXKERNSIZE), pagesize<kernelpl+1>())) * pagesize<1>();

// thread stack constants
static const size_t stackpl          = 1;
static const size_t minimumStack     = 1 * pagesize<stackpl>();
static const size_t defaultStack     = 2 * pagesize<stackpl>();
static const size_t defaultUserStack = 2 * pagesize<stackpl>();
static const size_t stackGuardPage   = 1 * pagesize<stackpl>();

#endif /* _Memory_h_ */
