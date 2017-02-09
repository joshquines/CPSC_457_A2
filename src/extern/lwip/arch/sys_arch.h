#ifndef _LWIP_ARCH_SYS_ARCH_H_
#define _LWIP_ARCH_SYS_ARCH_H_ 1

void sys_init(void);
u32_t sys_now(void);

typedef void* sys_sem_t;
typedef void* sys_mbox_t;
typedef void* sys_thread_t;

typedef int sys_prot_t;

#endif /* _LWIP_ARCH_SYS_ARCH_H_ */
