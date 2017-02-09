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
#ifndef _linuxtypes_h_
#define _linuxtypes_h_ 1

#include <cstddef>  
#include <cstdint>  
#if defined(__x86_64__)
typedef uint64_t mword;
typedef  int64_t sword;

typedef mword vaddr;
typedef mword paddr;

static const mword charbits = 8;
static const mword bytebits = 8;

typedef char  buf_t;
typedef char* bufptr_t;
typedef const char* cbufptr_t;
#else
#error unsupported architecture: only __x86_64__ supported at this time
#endif

#endif /* _linuxtypes_h_ */
