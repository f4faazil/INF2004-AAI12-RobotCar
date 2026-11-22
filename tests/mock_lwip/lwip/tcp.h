#ifndef MOCK_LWIP_TCP_H
#define MOCK_LWIP_TCP_H
#include "lwip/pbuf.h"
#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include <stddef.h>
struct tcp_pcb { int x; };
#define TCP_WRITE_FLAG_COPY 1
struct tcp_pcb *tcp_new(void);
void tcp_arg(struct tcp_pcb *,void *);
void tcp_recv(struct tcp_pcb *,err_t (*)(void *,struct tcp_pcb *,struct pbuf *,err_t));
void tcp_sent(struct tcp_pcb *,err_t (*)(void *,struct tcp_pcb *,u16_t));
void tcp_err(struct tcp_pcb *,void (*)(void *,err_t));
void tcp_poll(struct tcp_pcb *,err_t (*)(void *,struct tcp_pcb *), unsigned char);
void tcp_abort(struct tcp_pcb *);
err_t tcp_close(struct tcp_pcb *);
err_t tcp_connect(struct tcp_pcb *,const ip_addr_t *,u16_t, err_t (*)(void *,struct tcp_pcb *,err_t));
err_t tcp_write(struct tcp_pcb *,const void *,u16_t,unsigned char);
u16_t tcp_sndbuf(const struct tcp_pcb *);
err_t tcp_output(struct tcp_pcb *);
void tcp_recved(struct tcp_pcb *,u16_t);
#endif
