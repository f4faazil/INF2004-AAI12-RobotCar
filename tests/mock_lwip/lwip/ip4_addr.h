#ifndef MOCK_IP4_H
#define MOCK_IP4_H
#include <stdint.h>
typedef struct { uint32_t addr; } ip4_addr_t;
typedef ip4_addr_t ip_addr_t;
int ip4addr_aton(const char *text, ip4_addr_t *out);
#define ip4_addr_isany_val(a) ((a).addr==0)
#endif
