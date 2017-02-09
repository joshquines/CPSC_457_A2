#include <cstddef>
#include <sys/types.h>

// dlmalloc configurations

#define ONLY_MSPACES 1
#define NO_MALLOC_STATS 1
#define LACKS_SYS_MMAN_H
#define LACKS_TIME_H
#define ABORT_ON_ASSERT_FAILURE 0
#define MMAP_CLEARS 0
#define malloc_getpagesize  2097152 // avoid call to 'sysconf'
#define DEFAULT_GRANULARITY 2097152
#if defined(__clang__)
#define NO_MALLINFO 1
#endif

extern void kassertprints(const char* const loc, int line, const char* const func);
extern void kassertprinte();
#define ABORT { kassertprints( "ABORT  : "       " in " __FILE__ ":", __LINE__, __func__); kassertprinte(); abort(); }

// mman.h

#define MAP_ANONYMOUS 0             // avoid call to 'open'
#define PROT_READ 0
#define PROT_WRITE 0
#define MAP_PRIVATE 0

#define MMAP(s) \
  dl_mmap(0, (s), MMAP_PROT, MMAP_FLAGS, -1, 0)

#define MUNMAP(a,s) \
  dl_munmap(a,s)

#define DIRECT_MMAP(s) MMAP(s)

extern "C" void *dl_mmap(void*, size_t, int, int, int, _off64_t);
extern "C" int dl_munmap(void*, size_t);
