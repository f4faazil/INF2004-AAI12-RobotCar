#if APP_BUDDY5
#include "b5.h"
#include <string.h>
/* Canonical little-endian bit numbering; no casts to packed C structs. */
static void put(uint8_t *p, unsigned *at, uint32_t v, unsigned n) {
    for (unsigned i=0;i<n;++i,++*at)
        if ((v>>i)&1u) p[*at/8u]|=(uint8_t)(1u<<(*at%8u));
}
static uint32_t get(const uint8_t *p, unsigned *at, unsigned n) {
    uint32_t v=0;
    for(unsigned i=0;i<n;++i,++*at) v|=(uint32_t)((p[*at/8u]>>(*at%8u))&1u)<<i;
    return v;
}
static void range_out(uint8_t *p,unsigned *at,b5_range r) {
    put(p,at,r.valid?r.mm:0,12); put(p,at,r.valid,1);
}
static b5_range range_in(const uint8_t *p,unsigned *at) {
    b5_range r={0}; r.mm=get(p,at,12); r.valid=get(p,at,1); return r;
}
void b5_encode_status(const b5_status *s,uint8_t out[B5_STATUS_BYTES]) {
    unsigned a=0; memset(out,0,B5_STATUS_BYTES);
    put(out,&a,s->state,3);put(out,&a,s->request,3);put(out,&a,s->faults,7);
    range_out(out,&a,s->front);range_out(out,&a,s->closest);
    range_out(out,&a,s->width);range_out(out,&a,s->left);range_out(out,&a,s->right);
    put(out,&a,s->slot,5);put(out,&a,s->count,5);put(out,&a,s->at_ms,32);put(out,&a,s->range_ms,32);
}
static bool measured_range_ok(b5_range r) {
    return r.valid?(r.mm>=20u && r.mm<=4000u):r.mm==0u;
}
bool b5_decode_status(const uint8_t in[B5_STATUS_BYTES],b5_status *s) {
    unsigned a=0;memset(s,0,sizeof(*s));
    s->state=get(in,&a,3);s->request=get(in,&a,3);s->faults=get(in,&a,7);
    s->front=range_in(in,&a);s->closest=range_in(in,&a);s->width=range_in(in,&a);
    s->left=range_in(in,&a);s->right=range_in(in,&a);
    s->slot=get(in,&a,5);s->count=get(in,&a,5);s->at_ms=get(in,&a,32);s->range_ms=get(in,&a,32);
    return s->request<=B5_RESUME && s->slot<B5_SCAN_SLOTS && s->count<=B5_SCAN_SLOTS &&
      measured_range_ok(s->front) && measured_range_ok(s->closest) &&
      measured_range_ok(s->left) && measured_range_ok(s->right) &&
      (s->width.valid || s->width.mm==0u);
}
void b5_encode_feedback(const b5_feedback *f,uint8_t out[B5_FEEDBACK_BYTES]) {
    unsigned a=0;memset(out,0,B5_FEEDBACK_BYTES);
    put(out,&a,f->mode,2);put(out,&a,f->stopped,1);put(out,&a,f->motion_valid,1);
    put(out,&a,f->line_valid,1);put(out,&a,f->line_found,1);
    put(out,&a,f->travel_mm,9);put(out,&a,f->at_ms,32);
}
bool b5_decode_feedback(const uint8_t in[B5_FEEDBACK_BYTES],b5_feedback *f) {
    unsigned a=0;memset(f,0,sizeof(*f));
    f->mode=get(in,&a,2);f->stopped=get(in,&a,1);f->motion_valid=get(in,&a,1);
    f->line_valid=get(in,&a,1);f->line_found=get(in,&a,1);
    f->travel_mm=get(in,&a,9);f->at_ms=get(in,&a,32);
    return (in[5]&0x80u)==0 && f->travel_mm<=500u; /* reject nonzero reserved bits */
}

#endif
