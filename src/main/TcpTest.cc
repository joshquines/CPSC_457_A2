/*
 * TcpTest.cc
 *
 *  Created on: 2014-04-01
 *      Author: behrooz
 */

#include "kernel/MemoryManager.h"
#include "kernel/Output.h"

#include "extern/lwip/lwip/src/include/lwip/sockets.h"

#include <cstring>

int TcpTest() {
  int fd = lwip_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    DBG::outl(DBG::Lwip,"cannot create socket");
    return -1;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(80);

  ip_addr temp;
  int res = ipaddr_aton("74.125.226.146", &temp);

  if (res < 0) {
    DBG::outl(DBG::Lwip,"error: first parameter is not a valid address family");
    lwip_close(fd);
    return -1;
  } else if (res == 0) {
    DBG::outl(DBG::Lwip,"char string (second parameter does not contain valid ipaddress)");
    lwip_close(fd);
    return -1;
  }

  addr.sin_addr.s_addr = temp.addr;

  if (lwip_connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    DBG::outl(DBG::Lwip,"TCP connect failed");
    lwip_close(fd);
    return -1;
  }

  DBG::outl(DBG::Lwip,"Connected to 74.125.226.146");

  const char *msg = "Test Message\n";
  lwip_write(fd,msg,strlen(msg));
  static char buffer[1024];
  int len = lwip_read(fd,buffer,1023);
  buffer[len] = '\0';
  DBG::outl(DBG::Lwip,"TCP received: ",buffer);

  lwip_shutdown(fd, SHUT_RDWR);
  lwip_close(fd);
  return 0;
}
