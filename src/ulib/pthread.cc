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
#include "syscalls.h"
#include "pthread.h"

#define SIGTERM 15

static void _pthread_start(void* (*func)(void*), void* data) {
  void* result = func(data);
  pthread_exit(result);
}

extern "C" int pthread_create(pthread_t*restrict tid, const pthread_attr_t*restrict attr, void* (*func)(void*), void*restrict data) {
  *tid = syscallStub(SyscallNum::_pthread_create, mword(_pthread_start), mword(func), mword(data));
  return 0;
}

extern "C" void pthread_exit(void* result) {
  syscallStub(SyscallNum::pthread_exit, mword(result));
}

extern "C" int pthread_join(pthread_t tid, void** result) {
  return syscallStub(SyscallNum::pthread_join, tid, mword(result));
}

extern "C" int pthread_kill(pthread_t tid, int sig) {
  return syscallStub(SyscallNum::pthread_kill, tid, sig);
}

extern "C" int pthread_cancel(pthread_t tid) {
  return syscallStub(SyscallNum::pthread_kill, tid, SIGTERM);
}

extern "C" pthread_t pthread_self(void) {
  return syscallStub(SyscallNum::pthread_self);
}

extern "C" int semCreate(mword* rsid, mword init) {
  return syscallStub(SyscallNum::semCreate, mword(rsid), init);
}

extern "C" int semDestroy(mword sid) {
  return syscallStub(SyscallNum::semDestroy, sid);
}

extern "C" int semP(mword sid) {
  return syscallStub(SyscallNum::semP, sid);
}

extern "C" int semV(mword sid) {
  return syscallStub(SyscallNum::semV, sid);
}

extern "C" int pthread_cond_broadcast(pthread_cond_t* c) {
  while (!c->ben.empty()) c->ben.V();
  return 0;
}

extern "C" int pthread_cond_destroy(pthread_cond_t* c) {
  c->ben.destroy();
  return 0;
}

extern "C" int pthread_cond_init(pthread_cond_t*restrict c, const pthread_condattr_t*restrict) {
  c->ben.init(0);
  return 0;
}

extern "C" int pthread_cond_signal(pthread_cond_t* c) {
  if (!c->ben.empty()) c->ben.V();
  return 0;
}

//extern "C" int pthread_cond_timedwait(pthread_cond_t*restrict c, pthread_mutex_t*restrict m, const struct timespec*restrict t) {
//  *__errno() = ENOTSUP;
//  return -1;
//}

extern "C" int pthread_cond_wait(pthread_cond_t*restrict c, pthread_mutex_t*restrict m) {
  c->ben.P();
  return 0;
}

extern "C" int pthread_mutex_destroy(pthread_mutex_t* m) {
  m->ben.destroy();
  return 0;
}

extern "C" int pthread_mutex_init(pthread_mutex_t*restrict m, const pthread_mutexattr_t*restrict a) {
  m->ben.init(1);
  return 0;
}

extern "C" int pthread_mutex_lock(pthread_mutex_t* m) {
  m->ben.P();
  return 0;
}

//extern "C" int pthread_mutex_timedlock(pthread_mutex_t*restrict m, const struct timespec*restrict t) {
//  *__errno() = ENOTSUP;
//  return -1;
//}

extern "C" int pthread_mutex_trylock(pthread_mutex_t* m) {
  if (m->ben.tryP()) return 0;
  *__errno() = EBUSY;
  return -1;
}

extern "C" int pthread_mutex_unlock(pthread_mutex_t* m) {
  m->ben.V();
  return 0;
}
