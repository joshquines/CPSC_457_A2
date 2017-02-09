#ifndef _lwipopts_h_
#define _lwipopts_h_

#define NO_SYS                          0
#define LWIP_COMPAT_MUTEX               1
#define SYS_LIGHTWEIGHT_PROT            1

//#define LWIP_NETCONN                  	0
//#define LWIP_NETIF_API                  1
//#define LWIP_TCPIP_CORE_LOCKING         1

/* Minimal changes to opt.h required for tcp unit tests: */
#define MEM_SIZE                        16000
#define TCP_SND_QUEUELEN                40
#define MEMP_NUM_TCP_SEG                TCP_SND_QUEUELEN
#define TCP_SND_BUF                     (12 * TCP_MSS)
#define TCP_WND                         (10 * TCP_MSS)

#define LWIP_HAVE_LOOPIF              	1
#define LWIP_DHCP                     	1
#define LWIP_SOCKET                   	1
#define LWIP_COMPAT_SOCKETS             0
#define LWIP_POSIX_SOCKETS_IO_NAMES     0

#define LWIP_ICMP                       1
#define ICMP_STATS                      1

//DEBUG
#define DHCP_DEBUG                      LWIP_DBG_OFF
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_ON
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define IGMP_DEBUG                      LWIP_DBG_OFF
#define INET_DEBUG                      LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define IP_REASS_DEBUG                  LWIP_DBG_ON
#define RAW_DEBUG                       LWIP_DBG_OFF
#define MEM_DEBUG                       LWIP_DBG_OFF
#define MEMP_DEBUG                      LWIP_DBG_OFF
#define SYS_DEBUG                       LWIP_DBG_OFF
#define TIMERS_DEBUG                    LWIP_DBG_OFF
#define TCP_DEBUG                       LWIP_DBG_ON
#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF
#define LWIP_DEBUG                      LWIP_DBG_ON
#define MEMP_DEBUG                      LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define DBG_TYPES_ON                    LWIP_DBG_OFF

#endif /* _lwipopts_h_ */
