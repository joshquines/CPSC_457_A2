/******************************************************************************
    Copyright © 2012-2014 Martin Karsten

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
#ifndef _JoinableThread_h_
#define _JoinableThread_h_ 1

#include "generic/Buffers.h"
#include "runtime/SyncQueues.h" 
#include "runtime/Thread.h" 

class JoinableThread : public Thread {
  Condition wait;
  ptr_t* result;
  enum State { Regular, Detached, Joining } state;
public:
  JoinableThread(vaddr sb, size_t ss) : Thread(sb, ss), state(Regular) {}
  void post(ptr_t res, BasicLock& lock) {
    if (state == Detached) return;
    if (wait.empty()) {
      result = &res;
      wait.wait(lock);
    } else {
      *result = res;
      wait.signal(lock);
    }
  }
  bool join(ptr_t& res, BasicLock& l) {
    if (state != Regular) return false;
    state = Joining;
    if (wait.empty()) {
      result = &res;
      wait.wait(l);
    } else {
      res = *result;
      wait.signal(l);
    };
    return true;
  }
};

#endif /* _JoinableThread_h_ */
