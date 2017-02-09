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
#include "runtime/Scheduler.h"
#include "runtime/Thread.h"
#include "kernel/Clock.h"
#include "kernel/Output.h"
#include "kernel/Process.h"
#include "world/Access.h"
#include "machine/Processor.h"

#include "syscalls.h"
#include "pthread.h"

/******* libc functions *******/

// for C-style 'assert' (e.g., from malloc.c)
extern "C" void __assert_func( const char* const file, size_t line,
  const char* const func, const char* const expr ) {
  KERR::outl("ASSERT: ", file, ':', line, ' ', func, ' ', expr);
  Reboot();
}

extern "C" void abort() {
  KABORT0();
  unreachable();
}

extern "C" void free(void* ptr) { MemoryManager::legacy_free(ptr); }
extern "C" void _free_r(_reent* r, void* ptr) { free(ptr); }
extern "C" void* malloc(size_t size) { return MemoryManager::legacy_malloc(size); }
extern "C" void* _malloc_r(_reent* r, size_t size) { return malloc(size); }

extern "C" void* calloc(size_t nmemb, size_t size) {
  void* ptr = malloc(nmemb * size);
  memset(ptr, 0, nmemb * size);
  return ptr;
}

extern "C" void* _calloc_r(_reent* r, size_t nmemb, size_t size) {
  return calloc(nmemb, size);
}

extern "C" void* realloc(void* ptr, size_t size) {
  KABORT1("realloc");
  return nullptr;
}

extern "C" void* _realloc_r(_reent* r, void* ptr, size_t size) {
  return realloc(ptr, size);
}

/******* syscall functions *******/

// libc exit calls atexit routines, then invokes _exit
extern "C" void _exit(int) {
  CurrProcess().exit();
}

extern "C" int open(const char *path, int oflag, ...) {
  Process& p = CurrProcess();
  auto it = kernelFS.find(path);
  if (it == kernelFS.end()) return -ENOENT;
  return p.ioHandles.store(knew<FileAccess>(it->second));
}

extern "C" int close(int fildes) {
  Process& p = CurrProcess();
  Access* access = p.ioHandles.remove(fildes);
  if (!access) return -EBADF;
  delete access;
  p.ioHandles.release(fildes);
  return 0;
}

extern "C" ssize_t read(int fildes, void* buf, size_t nbyte) {
  // TODO: validate buf/nbyte
  Process& p = CurrProcess();
  Access* access = p.ioHandles.access(fildes);
  if (!access) return -EBADF;
  ssize_t ret = access->read(buf, nbyte);
  p.ioHandles.done(fildes);
  return ret;
}

extern "C" ssize_t write(int fildes, const void* buf, size_t nbyte) {
  // TODO: validate buf/nbyte
  Process& p = CurrProcess();
  Access* access = p.ioHandles.access(fildes);
  if (!access) return -EBADF;
  ssize_t ret = access->write(buf, nbyte);
  p.ioHandles.done(fildes);
  return ret;
}

extern "C" off_t lseek(int fildes, off_t offset, int whence) {
  Process& p = CurrProcess();
  Access* access = p.ioHandles.access(fildes);
  if (!access) return -EBADF;
  ssize_t ret = access->lseek(offset, whence);
  p.ioHandles.done(fildes);
  return ret;
}

extern "C" pid_t getpid() {
  return CurrProcess().getID();
}

extern "C" pid_t getcid() {
  return LocalProcessor::getIndex();
}

extern "C" int usleep(useconds_t usecs) {
  Timeout::sleep(Clock::now() + usecs);
  return 0;
}

extern "C" int _mmap(void** addr, size_t len, int protflags, int fildes, off_t off) {
  // TODO: validate addr
  int prot = protflags & 0xf;
  int flags = protflags >> 4;
  vaddr va = CurrProcess().map<1>(vaddr(*addr), len, prot, flags, fildes, off);
  if (va == topaddr) return -ENOMEM;
  *addr = (void*)va;
  return 0;
}

extern "C" int _munmap(void* addr, size_t len) {
  CurrProcess().unmap<1>(vaddr(addr), len);
  return 0;
}

extern "C" pthread_t _pthread_create(funcvoid2_t invoke, funcvoid1_t func, void* data) {
  return CurrProcess().createThread(invoke, func, data);
}

extern "C" void pthread_exit(void* result) {
  CurrProcess().exitThread(result);
}

extern "C" int pthread_join(pthread_t tid, void** result) {
  // TODO: validate result
  return CurrProcess().joinThread(tid, *result);
}

extern "C" int pthread_kill(pthread_t tid, int signal) {
//  return CurrProcess().signalThread(tid, signal);
  KABORT1("pthread_kill");
}

extern "C" pthread_t pthread_self() {
  return Process::getCurrentThreadID();
}

extern "C" int semCreate(mword* rsid, mword init) {
  // TODO: validate rsid
  Process& p = CurrProcess();
  p.semStoreLock.acquire();
  Semaphore* s = knew<Semaphore>(init);
  *rsid = p.semStore.put(s);
  p.semStoreLock.release();
  return 0;
}

extern "C" int semDestroy(mword sid) {
  Process& p = CurrProcess();
  p.semStoreLock.acquire();
  if (!p.semStore.valid(sid) || !p.semStore.get(sid)->empty()) { p.semStoreLock.release(); return -1; }
  kdelete(p.semStore.get(sid));
  p.semStore.remove(sid);
  p.semStoreLock.release();
  return 0;
}

extern "C" int semP(mword sid) {
  Process& p = CurrProcess();
  p.semStoreLock.acquire();
  if (!p.semStore.valid(sid)) { p.semStoreLock.release(); return -1; }
  p.semStore.get(sid)->P(&p.semStoreLock);
  return 0;
}

extern "C" int semV(mword sid) {
  Process& p = CurrProcess();
  p.semStoreLock.acquire();
  if (!p.semStore.valid(sid)) { p.semStoreLock.release(); return -1; }
  p.semStore.get(sid)->V(&p.semStoreLock);
  return 0;
}

typedef int (*funcint4_t)(mword, mword, mword, mword);
extern "C" int privilege(ptr_t func, mword a1, mword a2, mword a3, mword a4) {
  return ((funcint4_t)func)(a1, a2, a3, a4);
}

extern "C" void _init_sig_handler(vaddr sighandler) {
  // TODO: validate sighandler
  CurrProcess().setSignalHandler(sighandler);
}

/******* dummy functions *******/

extern "C" int fstat(int fildes, struct stat *buf) {
  KABORT1("fstat"); return -ENOSYS;
}

//extern "C" char *getenv(const char *name) {
//  DBG::outl(DBG::Libc, "LIBC/getenv: ", name);
//  return nullptr;
//}

extern "C" int isatty(int fd) {
  KABORT1("isatty"); return -ENOSYS;
}

//extern "C" int kill(pid_t pid, int sig) {
//  KABORT1("kill"); return -ENOSYS;
//}

//extern "C" void* sbrk(intptr_t increment) {
//  KABORT1("sbrk"); return (void*)-1;
//}

void* __dso_handle = nullptr;

typedef ssize_t (*syscall_t)(mword a1, mword a2, mword a3, mword a4, mword a5);
static const syscall_t syscalls[] = {
  syscall_t(_exit),
  syscall_t(open),
  syscall_t(close),
  syscall_t(read),
  syscall_t(write),
  syscall_t(lseek),
  syscall_t(getpid),
  syscall_t(getcid),
  syscall_t(usleep),
  syscall_t(_mmap),
  syscall_t(_munmap),
  syscall_t(_pthread_create),
  syscall_t(pthread_exit),
  syscall_t(pthread_join),
  syscall_t(pthread_kill),
  syscall_t(pthread_self),
  syscall_t(semCreate),
  syscall_t(semDestroy),
  syscall_t(semP),
  syscall_t(semV),
  syscall_t(privilege),
  syscall_t(_init_sig_handler)
};

static_assert(sizeof(syscalls)/sizeof(syscall_t) == SyscallNum::max, "syscall list error");

extern "C" ssize_t syscall_handler(mword x, mword a1, mword a2, mword a3, mword a4, mword a5) {
  ssize_t retcode = -ENOSYS;
  if (x < SyscallNum::max) retcode = syscalls[x](a1, a2, a3, a4, a5);
  else DBG::outl(DBG::Tests, "syscall: ", x);
  // TODO: check for signals
  return retcode;
}
