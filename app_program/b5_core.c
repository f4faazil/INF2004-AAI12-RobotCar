#if APP_BUDDY5
#include "b5.h"
#include <string.h>
/* Q15 sine values for -30..+30 degrees, in 2.5-degree steps.
 * Slot 0 = right, slot 12 = forward, slot 24 = left.
 */
static const int16_t sine_q15[25] = {
    -16384, -15131, -13848, -12540, -11207, -9854,
    -8481, -7092, -5690, -4277, -2856, -1429,
    0,
    1429, 2856, 4277, 5690, 7092, 8481,
    9854, 11207, 12540, 13848, 15131, 16384
};
_Static_assert(B5_INVALID_LIMIT >= 1 && B5_INVALID_LIMIT <= 3,
               "retry field supports 1..3 attempts");
_Static_assert(B5_STOP_MM >= 20 && B5_STOP_MM <= 4000,
               "stop threshold outside measurement range");
_Static_assert(B5_PING_PERIOD_MS >= 61,
               "ping interval must exceed 60 ms");
_Static_assert(B5_RECOVERY_MAX_MM <= 500,
               "travel range is 0..500 mm");
static uint32_t age(uint32_t now,uint32_t then) {return now-then;}
static void fault(b5_core *b,unsigned bits) {
    b->hw.cancel();b->waiting=0;b->s.state=B5_FAULT;b->s.request=B5_STOP;
    b->s.faults|=bits;
}
static void point(b5_core *b,unsigned slot,uint32_t now) {
    b->hw.cancel();b->waiting=0;b->s.slot=slot;b->hw.servo((uint8_t)slot);
    b->servo_ms=now;b->retries=0;
}
static void scan_put(b5_core *b,unsigned slot,b5_range r) {
    unsigned v=r.valid?(r.mm|0x1000u):0;
    for(unsigned i=0;i<13;++i){
        unsigned bit=slot*13u+i;uint8_t mask=(uint8_t)(1u<<(bit%8u));
        b->scan[bit/8u]=(uint8_t)((b->scan[bit/8u]&~mask)|(((v>>i)&1u)?mask:0u));
    }
}
b5_range b5_scan_get(const b5_core *b,unsigned slot) {
    b5_range r={0};if(slot>=B5_SCAN_SLOTS)return r;
    unsigned v=0;
    for(unsigned i=0;i<13;++i){unsigned bit=slot*13u+i;v|=((b->scan[bit/8u]>>(bit%8u))&1u)<<i;}
    r.mm=v&4095u;r.valid=(v>>12)&1u;return r;
}
void b5_init(b5_core *b,b5_hal hw,uint32_t now) {
    memset(b,0,sizeof(*b));b->hw=hw;b->s.state=B5_MONITOR;b->s.request=B5_STOP;
    b->phase_ms=now;b->ping_ms=now-B5_PING_PERIOD_MS;point(b,12,now);
}
/* Nonblocking echo acquisition; only a completed attempt changes results. */
static bool measure(b5_core *b,uint32_t now,b5_range *r,uint8_t *reason) {
    if(b->waiting) {
        uint16_t mm=0;uint8_t result=B5_MEASURE_BAD;
        if(!b->hw.poll(&mm,&result))return false;
        b->waiting=0;r->valid=(result==B5_MEASURE_OK && mm>=20 && mm<=4000);
        r->mm=r->valid?mm:0;*reason=result;
        if(result==B5_MEASURE_OK && !r->valid)*reason=B5_MEASURE_BAD;
        return true;
    }
    if(age(now,b->servo_ms)>=B5_SERVO_SETTLE_MS &&
       age(now,b->ping_ms)>=B5_PING_PERIOD_MS && b->hw.start()) {
        b->ping_ms=now;b->waiting=1;
    }
    return false;
}
static bool find_fine(b5_core *b,unsigned from,unsigned *next) {
    for(unsigned i=from;i<=b->last;++i)
        if(!b5_scan_get(b,i).valid){*next=i;return true;}
    return false;
}
static void finish_profile(b5_core *b,uint32_t now) {
    unsigned lo=25,hi=0,min=4001;int32_t xmin=0,xmax=0;
    b->s.left=b5_scan_get(b,24);b->s.right=b5_scan_get(b,0);
    /* side clearance = minimum of BOTH flank coarse rays, not furthest ray. */
    b5_range q=b5_scan_get(b,18);
    if(!q.valid)b->s.left.valid=0;
    else if(q.mm<b->s.left.mm)b->s.left.mm=q.mm;
    q=b5_scan_get(b,6);
    if(!q.valid)b->s.right.valid=0;
    else if(q.mm<b->s.right.mm)b->s.right.mm=q.mm;
    for(unsigned i=0;i<25;++i) {
        q=b5_scan_get(b,i);if(!q.valid)continue;
        if(q.mm<min)min=q.mm;
        if(q.mm<=B5_PROFILE_MM) {
            int32_t x=(int32_t)q.mm*sine_q15[i];
            if(lo==25){lo=i;xmin=x;xmax=x;}else {if(x<xmin)xmin=x;if(x>xmax)xmax=x;}
            hi=i;
        }
    }
    b->s.closest.valid=min<=4000;b->s.closest.mm=min<=4000?min:0;
    /* Observed horizontal span only. Require sampled clear brackets; never
       advertise a clipped FOV or one-ray return as a valid width estimate. */
    b->s.width.valid=0;b->s.width.mm=0;
    if(lo>0 && lo<hi && hi<24 && b5_scan_get(b,lo-1).valid &&
       b5_scan_get(b,hi+1).valid && b5_scan_get(b,lo-1).mm>B5_PROFILE_MM &&
       b5_scan_get(b,hi+1).mm>B5_PROFILE_MM) {
        unsigned w=(unsigned)((xmax-xmin+16384)/32768);
        if(w<=4095){
        b->s.width.mm=w; /* retain candidate estimate for diagnostics */
        b->s.width.valid=B5_WIDTH_VALIDATED ? 1u : 0u;
        }
    }
    bool left=b->s.left.valid && b->s.left.mm>=B5_SIDE_CLEAR_MM;
    bool right=b->s.right.valid && b->s.right.mm>=B5_SIDE_CLEAR_MM;
    if(!left && !right){fault(b,B5_F_NO_ROUTE);return;}
    b->s.request=(left && (!right || b->s.left.mm>=b->s.right.mm))?B5_LEFT:B5_RIGHT;
    b->s.state=B5_READY;b->phase_ms=now;point(b,12,now);
}
void b5_step(b5_core *b,const b5_feedback *f,uint32_t now) {
    b->s.at_ms=now;
    bool fresh=f->motion_valid && age(now,f->at_ms)<=B5_FEEDBACK_MAX_MS;
    if(b->s.state==B5_FAULT) {
        if(fresh && f->stopped && f->mode==B5_MODE_RESET)b5_init(b,b->hw,now);
        return;
    }
    if(!fresh){fault(b,B5_F_FEEDBACK);return;}
    if(f->mode==B5_MODE_RESET){
        if(f->stopped)b5_init(b,b->hw,now);else fault(b,B5_F_FEEDBACK);
        return;
    }
    if(b->s.state==B5_WAIT_STOP) {
        /* stopped must be a NEW feedback report after this STOP request. */
        if(f->stopped && (int32_t)(f->at_ms-b->phase_ms)>0) {
            memset(b->scan,0,sizeof(b->scan));b->s.count=0;
            b->s.state=B5_COARSE;b->phase_ms=now;point(b,0,now);
        } else if(age(now,b->phase_ms)>=B5_STOP_ACK_MS)fault(b,B5_F_STOP_TIMEOUT);
        return;
    }
    if(b->s.state==B5_COARSE || b->s.state==B5_FINE) {
        if(!f->stopped){fault(b,B5_F_SCAN);return;}
        if(age(now,b->phase_ms)>=B5_SCAN_MAX_MS){fault(b,B5_F_SCAN);return;}
        b5_range r={0};uint8_t reason;
        if(!measure(b,now,&r,&reason))return;
        if(!r.valid){
            if(++b->retries>=B5_INVALID_LIMIT)fault(b,reason==B5_MEASURE_NO_ECHO?B5_F_NO_ECHO:B5_F_BAD_RANGE);
            return;
        }
        scan_put(b,b->s.slot,r);++b->s.count;
        if(b->s.state==B5_COARSE) {
            if(b->s.slot<24){point(b,b->s.slot+6,now);return;}
            unsigned first=25,last=0;
            for(unsigned i=0;i<25;i+=6)if(b5_scan_get(b,i).mm<=B5_PROFILE_MM){if(first==25)first=i;last=i;}
            if(first==25){finish_profile(b,now);return;}
            b->first=first>=6?first-6:0;b->last=last<=18?last+6:24;
            b->s.state=B5_FINE;
            unsigned next;if(find_fine(b,b->first,&next)){point(b,next,now);return;}
        } else {
            unsigned next;if(find_fine(b,b->s.slot+1,&next)){point(b,next,now);return;}
        }
        finish_profile(b,now);return;
    }
    if(b->s.state==B5_READY) {
        if(b->s.request==B5_RESUME)return; /* reset/stale checks still run above */
        if(f->mode==B5_MODE_BYPASS && age(now,b->servo_ms)>=B5_SERVO_SETTLE_MS) {
            b->s.state=B5_BYPASS;b->recovery_ms=now;b->s.request=B5_STOP;
        } else {
            if(!f->stopped)fault(b,B5_F_SCAN);
            else if(age(now,b->phase_ms)>=B5_SCAN_MAX_MS)fault(b,B5_F_SCAN);
            return;
        }
    }
    if(b->s.state==B5_BYPASS || b->s.state==B5_RECOVERY) {
        /* Includes bypass plus recovery, conservatively bounding whole detour. */
        if(age(now,b->recovery_ms)>=B5_RECOVERY_MAX_MS || f->travel_mm>=B5_RECOVERY_MAX_MM) {
            fault(b,B5_F_RECOVERY);return;
        }
        if(f->mode!=B5_MODE_BYPASS && f->mode!=B5_MODE_RECOVERY) {
            fault(b,B5_F_FEEDBACK);return;
        }
        if(f->mode==B5_MODE_RECOVERY)b->s.state=B5_RECOVERY;
        if(b->s.state==B5_RECOVERY && f->line_valid && f->line_found) {
            /* Hold until VC acknowledges by RESET while stopped; never auto-restart. */
            b->hw.cancel();b->waiting=0;b->s.state=B5_READY;b->phase_ms=now;
            b->s.request=B5_RESUME;return;
        }
    }
    b5_range r={0};uint8_t reason;
    if(!measure(b,now,&r,&reason))return;
    b->s.front=r;b->s.range_ms=now;
    if(!r.valid) {
        b->s.request=B5_STOP; /* no echo is UNKNOWN, never free space */
        if(++b->retries>=B5_INVALID_LIMIT)fault(b,reason==B5_MEASURE_NO_ECHO?B5_F_NO_ECHO:B5_F_BAD_RANGE);
        return;
    }
    b->retries=0;
    if(r.mm<=B5_STOP_MM) {
        b->s.request=B5_STOP;
        if(b->s.state==B5_MONITOR){b->s.state=B5_WAIT_STOP;b->phase_ms=now;}
        else fault(b,B5_F_NO_ROUTE);
    } else b->s.request=B5_NONE;
}

#endif
