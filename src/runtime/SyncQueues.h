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
#ifndef _SyncQueues_h_
#define _SyncQueues_h_ 1

#include "runtime/BlockingSync.h"

template<typename Buffer>
class MessageQueue {
  typedef typename Buffer::Element Element;
  BasicLock lock;
  Buffer buffer;
  size_t sendSlots;                              // used for baton-passing
  size_t recvSlots;                              // used for baton-passing
  BlockingQueue sendQueue;
  BlockingQueue recvQueue;

  bool internalSend(const Element& elem, mword timeout = limit<mword>()) {
    lock.acquire();
    if fastpath(sendSlots == 0) {
      if slowpath(!sendQueue.block(lock, timeout)) return false;
      lock.acquire();
    } else {
      sendSlots -= 1;
    }
    buffer.push(elem);
    if slowpath(!recvQueue.resume(lock)) {     // try baton passing
      recvSlots += 1;                          // baton not passed
      lock.release();
    }
    return true;
  }

  bool internalRecv(Element& elem, mword timeout = limit<mword>()) {
    lock.acquire();
    if fastpath(recvSlots == 0) {
      if slowpath(!recvQueue.block(lock, timeout)) return false;
      lock.acquire();
    } else {
      recvSlots -= 1;
    }
    elem = buffer.front();
    buffer.pop();
    if slowpath(!sendQueue.resume(lock)) {     // try baton passing
      sendSlots += 1;                          // baton not passed
      lock.release();
    }
    return true;
  }

public:
  explicit MessageQueue(size_t N = 0) : buffer(N),
    sendSlots(buffer.max_size()), recvSlots(0) {}

  ~MessageQueue() {
    GENASSERT0(buffer.empty());
    GENASSERT1(sendSlots == buffer.max_size(), sendSlots);
    GENASSERT1(recvSlots == 0, recvSlots);
  }

  mword size() { return buffer.size(); }

  bool send(const Element& elem) {
    return internalSend(elem);
  }

  bool trySend(const Element& elem, mword t = 0) {
    return internalSend(elem, t);
  }

  bool recv(Element& elem) {
    return internalRecv(elem);
  }

  bool tryRecv(Element& elem, mword t = 0) {
    return internalRecv(elem, t);
  }

  Element recv() {
    Element e = Element();
    internalRecv(e);
    return e;
  }
};

#endif /* _SyncQueues_h_ */
