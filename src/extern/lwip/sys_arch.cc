extern "C" {
#include "lwip/sys.h"
}

#include "generic/Buffers.h"
#include "runtime/SyncQueues.h"
#include "kernel/Clock.h"
#include "kernel/MemoryManager.h"
#include "kernel/Output.h"

extern "C" err_t sys_sem_new(sys_sem_t *sem, u8_t count) {
  *sem = knew<Semaphore>(count);
  return ERR_OK;
}

extern "C" void sys_sem_signal(sys_sem_t *sem) {
  reinterpret_cast<Semaphore*>(*sem)->V();
}

extern "C" u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout) {
  mword before = Clock::now();
  if (timeout == 0) {
    reinterpret_cast<Semaphore*>(*sem)->P();
    return Clock::now() - before;
  } else if (reinterpret_cast<Semaphore*>(*sem)->tryP(timeout)) {
    return Clock::now() - before;
  } else {
    return SYS_ARCH_TIMEOUT;
  }
}

extern "C" void sys_sem_free(sys_sem_t *sem) {
  kdelete((Semaphore*)*sem);
}

extern "C" int sys_sem_valid(sys_sem_t *sem) {
  return *sem != nullptr;
}

extern "C" void sys_sem_set_invalid(sys_sem_t *sem) {
  *sem = nullptr;
}

typedef MessageQueue<RuntimeRingBuffer<void*,KernelAllocator<void*>>> MQ;

extern "C" err_t sys_mbox_new(sys_mbox_t *mbox, int size) {
  *mbox = knew<MQ>( max(size,128) );
  return ERR_OK;
}

extern "C" void sys_mbox_post(sys_mbox_t *mbox, void *msg) {
  reinterpret_cast<MQ*>(*mbox)->send(msg);
}

extern "C" err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg) {
  if (reinterpret_cast<MQ*>(*mbox)->trySend(msg)) return ERR_OK;
  return ERR_MEM;
}

extern "C" u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout) {
  mword before = Clock::now();
  if (timeout == 0) {
    reinterpret_cast<MQ*>(*mbox)->recv(*msg);
    return Clock::now() - before;
  } else if (reinterpret_cast<MQ*>(*mbox)->tryRecv(*msg, timeout)) {
    return Clock::now() - before;
  } else {
    return SYS_ARCH_TIMEOUT;
  }
}

extern "C" u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg) {
  mword before = Clock::now();
  if (reinterpret_cast<MQ*>(*mbox)->tryRecv(*msg, 0)) {
    return Clock::now() - before;
  } else {
    return SYS_ARCH_TIMEOUT;
  }
}

extern "C" void sys_mbox_free(sys_mbox_t *mbox) {
  kdelete((MQ*)*mbox);
}

extern "C" int sys_mbox_valid(sys_mbox_t *mbox) {
  return *mbox != nullptr;
}

extern "C" void sys_mbox_set_invalid(sys_mbox_t *mbox) {
  *mbox = nullptr;
}

extern "C" sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio) {
  Thread* t = Thread::create(stacksize + defaultStack);
  // t->setPriority(prio)
  t->start((ptr_t)thread, arg);
  return t;
}

// A lock that serializes all LWIP code
static OwnerLock* lwipLock;

extern "C" void sys_init(void) {
  lwipLock = knew<OwnerLock>();
}

extern "C" u32_t sys_jiffies(void) { KABORT0(); return 0; }

extern "C" u32_t sys_now(void) {
 return Clock::now();
} 

extern "C" sys_prot_t sys_arch_protect(void) {
  return lwipLock->acquire();
}

extern "C" void sys_arch_unprotect(sys_prot_t pval) {
  lwipLock->release();
}

extern "C" void lwip_assert(const char* const loc, int line, const char* const func, const char* const msg) {
  kassertprints(loc, line, func);
  kassertprinte(msg);
}

extern "C" void lwip_printf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  ExternDebugPrintf(DBG::Lwip, fmt, args);
  va_end(args);
}
