#ifndef MOCK_LWIP_PBUF_H
#define MOCK_LWIP_PBUF_H
#include <stdint.h>
typedef uint16_t u16_t;
struct pbuf { u16_t tot_len; const uint8_t *data; };
u16_t pbuf_copy_partial(const struct pbuf *, void *, u16_t, u16_t);
void pbuf_free(struct pbuf *);
#endif
