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
#ifndef _asmshare_h_
#define _asmshare_h_ 1

// TSSRSP must be consistent with Processor.h

#ifdef __ASSEMBLY__
.set BOOTAP16, 0x1000
.set TSSRSP, 0x54
#else
#define BOOTAP16 0x1000
#define TSSRSP 0x54
#define MAXKERNSIZE 0x40000000
#endif

#endif /* _asmshare_h_ */
