#ifndef _LWIP_ARCH_CC_H_
#define _LWIP_ARCH_CC_H_ 1

#include <ctype.h>
#include <stdint.h>
#include <sys/param.h>
#include <sys/types.h>

/* Define generic types used in lwIP */
typedef  uint8_t  u8_t;
typedef   int8_t  s8_t;
typedef uint16_t u16_t;
typedef  int16_t s16_t;
typedef uint32_t u32_t;
typedef  int32_t s32_t;

typedef uintptr_t mem_ptr_t;

/* Define (sn)printf formatters for these lwIP types */
#define X8_F  "02x"
#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "u"
#define S32_F "d"
#define X32_F "x"
/* If only we could use C99 and get %zu */
#if defined(__x86_64__)
#define SZT_F "lu"
#else
#define SZT_F "u"
#endif

/* Define platform endianness */
#ifndef BYTE_ORDER
#define BYTE_ORDER LITTLE_ENDIAN
#endif /* BYTE_ORDER */

/* Compiler hints for packing structures */
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

/* Platform specific diagnostic output */
void lwip_printf(const char* fmt, ...);
void lwip_assert(const char* const loc, int line, const char* const func, const char* const msg);

#define LWIP_PLATFORM_DIAG(x)	lwip_printf x
#define LWIP_PLATFORM_ASSERT(x)	lwip_assert(__FILE__, __LINE__, __func__, x)

#endif /* _LWIP_ARCH_CC_H_ */
