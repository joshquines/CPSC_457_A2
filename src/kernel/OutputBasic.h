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
#ifndef _OutputBasic_h_
#define _OutputBasic_h_ 1

// defined in Machine.cc
void Reboot(vaddr = 0) __noreturn;

// defined in Output.cc
extern void kassertprints(const char* const loc, int line, const char* const func);
extern void kassertprinte(const char* const msg);
extern void kassertprinte(const unsigned long long num);
extern void kassertprinte(const FmtHex& ptr);
extern void kassertprinte();

#define KABORT0()          {                      { kassertprints(  "KABORT: "       " in " __FILE__ ":", __LINE__, __func__); kassertprinte();    Reboot(); } }
#define KABORT1(msg)       {                      { kassertprints(  "KABORT: "       " in " __FILE__ ":", __LINE__, __func__); kassertprinte(msg); Reboot(); } }
#define KASSERT0(expr)     { if slowpath(!(expr)) { kassertprints( "KASSERT: " #expr " in " __FILE__ ":", __LINE__, __func__); kassertprinte();    Reboot(); } }
#define KASSERT1(expr,msg) { if slowpath(!(expr)) { kassertprints( "KASSERT: " #expr " in " __FILE__ ":", __LINE__, __func__); kassertprinte(msg); Reboot(); } }
#define KCHECK0(expr)      { if slowpath(!(expr)) { kassertprints(  "KCHECK: " #expr " in " __FILE__ ":", __LINE__, __func__); kassertprinte();    } }
#define KCHECK1(expr,msg)  { if slowpath(!(expr)) { kassertprints(  "KCHECK: " #expr " in " __FILE__ ":", __LINE__, __func__); kassertprinte(msg); } }

#endif /* _OutputBasic_h_ */
