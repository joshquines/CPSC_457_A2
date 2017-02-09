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

#include <string.h>

int signum = 0;

extern "C" void _KOS_sigwrapper();

extern "C" void _KOS_sighandler(mword s) {
  signum = s;
}

extern "C" void _initialize_KOS_standard_library() {
  syscallStub(SyscallNum::_init_sig_handler, (mword)_KOS_sigwrapper);
}

extern "C" void abort() { _exit(-1); }

extern "C" void _free_r(_reent* r, void* ptr) { free(ptr); }
extern "C" void* _malloc_r(_reent* r, size_t size) { return malloc(size); }
extern "C" void* _calloc_r(_reent* r, size_t nmemb, size_t size) { return calloc(nmemb, size); }
extern "C" void* _realloc_r(_reent* r, void* ptr, size_t size) { return realloc(ptr, size); }

extern "C" void _exit(int) {
  syscallStub(SyscallNum::_exit);
  for (;;); // never reached...
}

extern "C" int open(const char *path, int oflag, ...) {
  ssize_t ret = syscallStub(SyscallNum::open, mword(path), oflag);
  if (ret < 0) { *__errno() = -ret; return -1; } else return ret;
}

extern "C" int close(int fildes) {
  ssize_t ret = syscallStub(SyscallNum::close, fildes);
  if (ret < 0) { *__errno() = -ret; return -1; } else return ret;
}

extern "C" ssize_t read(int fildes, void* buf, size_t nbyte) {
  ssize_t ret = syscallStub(SyscallNum::read, fildes, mword(buf), nbyte);
  if (ret < 0) { *__errno() = -ret; return -1; } else return ret;
}

extern "C" ssize_t write(int fildes, const void* buf, size_t nbyte) {
  if (fildes == STDOUT_FILENO) {                      // copy stdout to stddbg
    syscallStub(SyscallNum::write, STDDBG_FILENO, mword(buf), nbyte);
  }
  ssize_t ret = syscallStub(SyscallNum::write, fildes, mword(buf), nbyte);
  if (ret < 0) { *__errno() = -ret; return -1; } else return ret;
}

extern "C" off_t lseek(int fildes, off_t offset, int whence) {
  ssize_t ret = syscallStub(SyscallNum::lseek, fildes, offset, whence);
  if (ret < 0) { *__errno() = -ret; return -1; } else return ret;
}

extern "C" pid_t getpid() {
  return syscallStub(SyscallNum::getpid);
}

extern "C" pid_t getcid() {
  return syscallStub(SyscallNum::getcid);
}

extern "C" int usleep(useconds_t usecs) {
  return syscallStub(SyscallNum::getcid, usecs);
}

extern "C" void* mmap(void* addr, size_t len, int prot, int flags, int filedes, off_t off) {
  void* newaddr = addr;
  ssize_t ret = syscallStub(SyscallNum::_mmap, mword(&newaddr), len, prot|(flags<<4), filedes, off);
  if (ret < 0) { *__errno() = -ret; return MAP_FAILED; } else return newaddr;
}

extern "C" int munmap(void* addr, size_t len) {
  ssize_t ret = syscallStub(SyscallNum::_munmap, mword(addr), len);
  if (ret < 0) { *__errno() = -ret; return -1; } else return ret;
}

extern "C" int privilege(void* func, mword a1, mword a2, mword a3, mword a4) {
  return syscallStub(SyscallNum::privilege, (mword)func, a1, a2, a3, a4);
}

/******* dummy functions *******/

extern "C" int fstat(int fildes, struct stat *buf) {
  memset(buf, 0, sizeof(struct stat));
  switch (fildes) {
    case STDIN_FILENO:  buf->st_mode = S_IFCHR;
    case STDOUT_FILENO: buf->st_mode = S_IFCHR;
    case STDERR_FILENO: buf->st_mode = S_IFCHR;
    default: buf->st_mode = S_IFREG;
  }
  return 0;
}

extern "C" char *getenv(const char *name) {
  return nullptr;
}

extern "C" int isatty(int fildes) {
  switch (fildes) {
    case STDIN_FILENO:  return 1;
    case STDOUT_FILENO: return 1;
    case STDERR_FILENO: return 1;
    default: return 0;
  }
}

extern "C" int kill(pid_t pid, int sig) {
  *__errno() = EINVAL;
  return -1;
}
