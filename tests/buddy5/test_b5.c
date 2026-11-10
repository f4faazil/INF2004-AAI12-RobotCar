#include "b5.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned slot;static bool pending;static unsigned mode;
static void servo(uint8_t s){slot=s;}
static bool start(void){assert(!pending);pending=true;return true;}
static bool poll(uint16_t *mm,uint8_t *reason){
    if(!pending)return false;
    pending=false;
    *reason=mode==2?B5_MEASURE_NO_ECHO:B5_MEASURE_OK;
    *mm=mode==0?1000:((slot>=8 && slot<=16)?200:900);
    if(mode==3)*mm=4100;
    if(mode==4)*mm=200;
    return true;
}
static void cancel(void){pending=false;}
static b5_feedback fb(uint32_t now){b5_feedback f={0};f.motion_valid=1;f.stopped=1;f.at_ms=now;return f;}
static b5_core setup(uint32_t now){b5_core b;b5_init(&b,(b5_hal){servo,start,poll,cancel},now);return b;}
static void ticks(b5_core *b,uint32_t *now,unsigned count){
    for(unsigned i=0;i<count;++i){b5_feedback f=fb(++*now);b5_step(b,&f,*now);}
}
static void codec(void){
    b5_status s={0},out;s.state=7;s.request=4;s.faults=127;s.slot=24;s.count=25;
    s.at_ms=0xfedcba98;s.range_ms=0x12345678;
    for(unsigned mm=0;mm<=4000;++mm){
        s.front=(b5_range){.mm=mm,.valid=mm>=20};s.width=s.front;s.closest=s.front;s.left=s.front;s.right=s.front;
        uint8_t p[B5_STATUS_BYTES];b5_encode_status(&s,p);assert(b5_decode_status(p,&out));
        assert(out.front.mm==(mm>=20?mm:0) && out.front.valid==(mm>=20) && out.width.mm==out.front.mm && out.right.mm==out.front.mm);
        assert(out.state==7 && out.request==4 && out.faults==127 && out.count==25);
        assert(out.at_ms==s.at_ms && out.range_ms==s.range_ms);
    }
    b5_feedback f=fb(0xffffffff),g;f.mode=3;f.line_valid=1;f.line_found=1;f.travel_mm=500;
    uint8_t p[B5_FEEDBACK_BYTES];b5_encode_feedback(&f,p);
    assert((p[0]&63u)==63u);assert(b5_decode_feedback(p,&g));assert(g.travel_mm==500 && g.at_ms==f.at_ms);
    p[5]|=128;assert(!b5_decode_feedback(p,&g));
}
int main(void){
    codec();unsigned cases=1;
    const unsigned monitor_ticks = B5_SERVO_SETTLE_MS + 100u;
    const unsigned scan_ticks =
    (B5_SCAN_SLOTS + 2u) *
    (B5_SERVO_SETTLE_MS + B5_PING_PERIOD_MS + 2u);
    /* Decoder rejects malformed fields before a consumer can act on them. */
    b5_status bad={0},decoded;uint8_t wire[B5_STATUS_BYTES];
    bad.front=(b5_range){.mm=4001,.valid=1};b5_encode_status(&bad,wire);
    assert(!b5_decode_status(wire,&decoded));
    bad.front=(b5_range){0};bad.request=5;b5_encode_status(&bad,wire);
    assert(!b5_decode_status(wire,&decoded));++cases;
    b5_feedback badf={.travel_mm=501},goodf;uint8_t fw[B5_FEEDBACK_BYTES];
    b5_encode_feedback(&badf,fw);assert(!b5_decode_feedback(fw,&goodf));++cases;
    uint32_t now=0;mode=0;b5_core b=setup(now);
    ticks(&b,&now,monitor_ticks);assert(b.s.state==B5_MONITOR && b.s.request==B5_NONE);++cases;
    /* No echo stops on the FIRST invalid sample, faults on third. */
    mode=2;ticks(&b,&now,70);assert(b.s.request==B5_STOP && !b.s.front.valid);
    ticks(&b,&now,200);assert(b.s.state==B5_FAULT && (b.s.faults&B5_F_NO_ECHO));++cases;
    b5_feedback f=fb(++now);f.mode=B5_MODE_RESET;b5_step(&b,&f,now);assert(b.s.state==B5_MONITOR);
    mode=1;	ticks(&b,&now,scan_ticks);assert(b.s.state==B5_READY && b.s.request==B5_LEFT);
    /* Synthetic span calculation for the +/-30 degree scan.
    * This tests arithmetic, not physical sensor accuracy. */
    assert(b.s.width.mm==69);
    assert(b.s.width.valid==(B5_WIDTH_VALIDATED ? 1u : 0u));
    assert(b.s.left.mm==900 && b.s.right.mm==900 && b.s.count>5);
    assert(sizeof b.scan==41 && (b.scan[40]&0xe0u)==0);
    for(unsigned i=0;i<25;++i){b5_range q=b5_scan_get(&b,i);
        if(q.valid)assert(q.mm==((i>=8 && i<=16)?200u:900u));}
    ++cases;
    /* Enter bypass only by explicit controller acknowledgement. */
    f=fb(++now);f.mode=B5_MODE_BYPASS;b5_step(&b,&f,now);assert(b.s.state==B5_BYPASS);
    f=fb(++now);f.mode=B5_MODE_RECOVERY;f.travel_mm=100;f.line_valid=1;f.line_found=1;
    b5_step(&b,&f,now);assert(b.s.request==B5_RESUME);
    b5_step(&b,&f,++now);assert(b.s.request==B5_RESUME);++cases;
    f.at_ms=now-101;b5_step(&b,&f,now);assert(b.s.faults&B5_F_FEEDBACK);++cases;
    /* Stop feedback from before request cannot authorize scan. */
    mode=1;now=0;b=setup(now);ticks(&b,&now,B5_SERVO_SETTLE_MS + 1u);assert(b.s.state==B5_WAIT_STOP);
    f=fb(now);b5_step(&b,&f,now);assert(b.s.state==B5_WAIT_STOP);
    f=fb(++now);b5_step(&b,&f,now);assert(b.s.state==B5_COARSE);
    f=fb(++now);f.stopped=0;b5_step(&b,&f,now);assert(b.s.faults&B5_F_SCAN);++cases;
    /* Saturation is not a valid distance. */
    mode=3;now=0;b=setup(now);ticks(&b,&now,B5_SERVO_SETTLE_MS + 300u);assert(b.s.faults&B5_F_BAD_RANGE);++cases;
    /* No candidate corridor -> STOP, not a blind reverse command. */
    mode=4;now=0;b=setup(now);ticks(&b,&now,scan_ticks);assert(b.s.faults&B5_F_NO_ROUTE);++cases;
    /* Recovery travel and time limits, at the exact boundary. */
    for(unsigned which=0;which<2;++which){mode=0;now=10000;b=setup(now);
        b.s.state=B5_RECOVERY;b.recovery_ms=now;
        f=fb(now);f.mode=B5_MODE_RECOVERY;
        if(which==0)f.travel_mm=500;else {now+=5000;f.at_ms=now;}
        b5_step(&b,&f,now);assert(b.s.faults&B5_F_RECOVERY);++cases;}
    /* Unsigned monotonic wrap remains correct for bounded elapsed intervals. */
    mode=0;now=0xffffff00u;b=setup(now);ticks(&b,&now,B5_SERVO_SETTLE_MS + 300u);assert(b.s.request==B5_NONE);++cases;
    /* Waiting for stop must eventually fault even with fresh moving reports. */
    mode=1;now=0;b=setup(now);ticks(&b,&now,B5_SERVO_SETTLE_MS + 1u);
    for(unsigned i=0;i<501;++i){f=fb(++now);f.stopped=0;b5_step(&b,&f,now);}
    assert(b.s.faults&B5_F_STOP_TIMEOUT);++cases;
    printf("PASS: %u test scenarios; 4001 distance codec round trips. status=%u B feedback=%u B core=%zu B\n",
           cases,B5_STATUS_BYTES,B5_FEEDBACK_BYTES,sizeof(b5_core));
}
