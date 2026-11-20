#ifndef MOCK_NETIF_H
#define MOCK_NETIF_H
#include "lwip/ip4_addr.h"
struct netif { ip4_addr_t ip_addr; int up,link; };
#define netif_is_up(n) ((n)->up)
#define netif_is_link_up(n) ((n)->link)
#define netif_ip4_addr(n) (&((n)->ip_addr))
#endif
