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
#include "machine/Paging.h"

const BitString<uint64_t, 0, 1> Paging::PageFaultFlags::P;
const BitString<uint64_t, 1, 1> Paging::PageFaultFlags::WR;
const BitString<uint64_t, 2, 1> Paging::PageFaultFlags::US;
const BitString<uint64_t, 3, 1> Paging::PageFaultFlags::RSVD;
const BitString<uint64_t, 4, 1> Paging::PageFaultFlags::ID;

ostream& operator<<(ostream& os, const Paging::PageFaultFlags& f) {
  if (f.t & Paging::PageFaultFlags::P())    os << " P";
  if (f.t & Paging::PageFaultFlags::WR())   os << " W/R";
  if (f.t & Paging::PageFaultFlags::US())   os << " U/S";
  if (f.t & Paging::PageFaultFlags::RSVD()) os << " RSVD";
  if (f.t & Paging::PageFaultFlags::ID())   os << " I/D";
  return os;
}

const BitString<uint64_t, 0, 1> Paging::P;
const BitString<uint64_t, 1, 1> Paging::RW;
const BitString<uint64_t, 2, 1> Paging::US;
const BitString<uint64_t, 3, 1> Paging::PWT;
const BitString<uint64_t, 4, 1> Paging::PCD;
const BitString<uint64_t, 5, 1> Paging::A;
const BitString<uint64_t, 6, 1> Paging::D;
const BitString<uint64_t, 7, 1> Paging::PS;
const BitString<uint64_t, 8, 1> Paging::G;
const BitString<uint64_t,12,40> Paging::ADDR;
const BitString<uint64_t,63, 1> Paging::XD;

ostream& operator<<(ostream& os, const Paging::FmtPE& f) {
  if (f.t & Paging::P())    os << " P";
  if (f.t & Paging::RW())   os << " RW";
  if (f.t & Paging::US())   os << " US";
  if (f.t & Paging::PWT())  os << " PWT";
  if (f.t & Paging::PCD())  os << " PCD";
  if (f.t & Paging::A())    os << " A";
  if (f.t & Paging::D())    os << " D";
  if (f.t & Paging::PS())   os << " PS";
  if (f.t & Paging::G())    os << " G";
  if (f.t & Paging::ADDR()) os << " ADDR:" << FmtHex(f.t & Paging::ADDR());
  if (f.t & Paging::XD())   os << " XD";
  return os;
}
