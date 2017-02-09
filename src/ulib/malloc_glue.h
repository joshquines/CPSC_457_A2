#include <cstddef>
#include <sys/types.h> // _off64_t

// dlmalloc configurations

#define HAVE_MORECORE 0            // don't have sbrk
#define LACKS_SYS_MMAN_H
#define LACKS_TIME_H
#define malloc_getpagesize 4096    // avoid call to 'sysconf'

// mman.h

#define MAP_ANONYMOUS 0            // avoid call to 'open'
#define PROT_READ     0
#define PROT_WRITE    0
#define MAP_PRIVATE   0

#define MMAP(s)        mmap(0, (s), MMAP_PROT, MMAP_FLAGS, -1, 0)
#define MUNMAP(a,s)    munmap(a,s)
#define DIRECT_MMAP(s) MMAP(s)

extern "C" void *mmap(void*, size_t, int, int, int, _off64_t);
extern "C" int munmap(void*, size_t);
