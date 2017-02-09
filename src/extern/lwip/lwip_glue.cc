extern "C" {
#include "lwip/def.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/mem.h"
#include "lwip/netbuf.h"
#include "lwip/netif.h"
#include "lwip/opt.h"
#include "lwip/pbuf.h"
#include "lwip/snmp.h"
#include "lwip/stats.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"
#include "netif/ppp_oe.h"
}

#include "kernel/MemoryManager.h"
#include "kernel/Output.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#define IFNAME0 'e'
#define IFNAME1 'n'

// see lwip/src/netif/ethernetif.c for explanations

struct cdi_net_device;
void cdi_net_send(ptr_t buffer, size_t size);

struct ethernetif {
  struct eth_addr *ethaddr;
  struct cdi_net_device* device;
};

void low_level_init(struct netif *netif) {
  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */// 52 54 00 12 34 56
  netif->hwaddr[0] = 0x52;
  netif->hwaddr[1] = 0x54;
  netif->hwaddr[2] = 0x00;
  netif->hwaddr[3] = 0x12;
  netif->hwaddr[4] = 0x34;
  netif->hwaddr[5] = 0x56;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
}

err_t low_level_output(struct netif *netif, struct pbuf *p) {
#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  // Send the data from the pbuf to the interface, one pbuf at a time. The
  // size of the data in each pbuf is kept in the ->len variable.
  for(struct pbuf *q = p; q != NULL; q = q->next) {
    cdi_net_send(q->payload, q->len);
  }
  // TODO: signal that packet should be sent

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  LINK_STATS_INC(link.xmit);

  return ERR_OK;
}

struct pbuf* low_level_input(struct netif *netif, bufptr_t buffer, size_t size) {
  u16_t len = size;

#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

  /* We allocate a pbuf chain of pbufs from the pool. */
  struct pbuf* p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

  if (p != NULL) {
#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

    /* We iterate over the pbuf chain until we have read the entire packet. */
    int counter = 0;
    for (struct pbuf* q = p; q != NULL; q = q->next) {
      /* Read enough bytes to fill this pbuf in the chain. The available
       * data in the pbuf is given by the q->len variable.
       * This does not necessarily have to be a memcpy, you can also
       * preallocate pbufs for a DMA-enabled MAC and after receiving
       * truncate it to the actually received size.  In this case, ensure
       * the tot_len member of the pbuf is the sum of the chained pbuf len
       * members.  */
      memcpy(q->payload, buffer + counter, q->len);
      counter += q->len;
    }
    // TODO: acknowledge that packet has been read
    LINK_STATS_INC(link.recv);

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
  } else {
    // TODO: drop packet
    LINK_STATS_INC(link.memerr);
    LINK_STATS_INC(link.drop);
  }
  return p;
}

void ethernetif_input(struct netif *netif, bufptr_t buffer, size_t size) {
  /* move received packet into a new pbuf */
  struct pbuf* p = low_level_input(netif, buffer, size);
  /* no packet could be read, silently ignore this */
  if (p == NULL) return;
  /* points to packet payload, which starts with an Ethernet header */
  struct eth_hdr* ethhdr = (struct eth_hdr *)p->payload;

  switch (htons(ethhdr->type)) {
  /* IP or ARP packet? */
  case ETHTYPE_IP:
  case ETHTYPE_ARP:
#if PPPOE_SUPPORT
  /* PPPoE packet? */
  case ETHTYPE_PPPOEDISC:
  case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
    /* full packet sent to tcpip_thread for processing */
    if (netif->input(p, netif) != ERR_OK) {
      DBG::outl(DBG::Lwip, "LWIP: IP input error");
      pbuf_free(p);
    }
    break;
  default:
    DBG::outl(DBG::Lwip, "LWIP: unknown Eth type");
    pbuf_free(p);
    break;
  }
}

err_t ethernetif_init(struct netif *netif) {
  KASSERT0(netif);
  struct ethernetif* ethernetif = kmalloc<struct ethernetif>();
  KASSERT0(ethernetif);

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /* Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second. */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->state = ethernetif;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* Directly use etharp_output() here to save a function call.  You can
   * instead declare your own function an call etharp_output() from it if
   * you have to do some checks before sending (e.g.  if link is available) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}
void lwip_net_receive(struct netif *nif, bufptr_t buffer, size_t size) {
  ethernetif_input(nif,buffer,size);
}

static const char *ip_to_string(uint32_t ip) {
  static char buf[32];
  sprintf(&buf[0], "%u.%u.%u.%u", (ip & 0xFF000000) >> 24,
      (ip & 0x00FF0000) >> 16, (ip & 0x0000FF00) >> 8, (ip & 0x000000FF));
  return &buf[0];
}

static void tcpip_init_done(void *arg) {
}

void lwip_init_tcpip() {
  tcpip_init(&tcpip_init_done, nullptr);
}

void* lwip_add_netif(void *ethif) {
  struct netif *nif = kmalloc<struct netif>();
  struct ip_addr ipaddr, netmask, gateway;

  // set defaults for qemu bridge setup, but use dhcp later anyway...
  IP4_ADDR(&gateway, 192,168,57,1);
  IP4_ADDR(&ipaddr, 192,168,57,200);
  IP4_ADDR(&netmask, 255,255,255,0);
//  ipaddr.addr = netmask.addr = gateway.addr = 0;

  if (!netif_add(nif, &ipaddr, &netmask, &gateway, ethif, ethernetif_init, tcpip_input)) {
    DBG::outl(DBG::Lwip, "LWIP: error in netif_add");
    kdelete(nif);
    return nullptr;
  } else {
    netif_set_default(nif);
    netif_set_up(nif);
    dhcp_start(nif);
    return nif;
  }
}
