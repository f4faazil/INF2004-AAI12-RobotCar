#if APP_BUDDY5
#include "b5_utk.h"
#include <tk/syslib.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
/* Do not include libc string.h beside tkernel.h: this BSP defines a
 * different size_t. Keep portable codec/core in separate translation units. */
static void copy_bytes(void *to,const void *from,unsigned n) {
    uint8_t *d=to;const uint8_t *s=from;while(n--)*d++=*s++;
}
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif
/* RP2040 IO_BANK0 register offsets: INTR2, PROC0_INTE2, PROC0_INTS2.
 * GP17 has four events at bits 4..7; bit 6 falling, bit 7 rising.
 * This port owns IRQ13 exclusively. Merge a GPIO dispatcher before adding
 * encoder interrupts; another tk_def_int(13,...) would replace this handler. */
#define ECHO_INTR (IO_BANK0_BASE + 0xf8u)
#define ECHO_INTE (IO_BANK0_BASE + 0x108u)
#define ECHO_INTS (IO_BANK0_BASE + 0x128u)
#define EDGE_FALL (1u << 6)
#define EDGE_RISE (1u << 7)
#define EDGES (EDGE_FALL | EDGE_RISE)
#define GPIO_IRQ 13u
#define PWM6 (PWM_BASE + 6u * 0x14u)
_Static_assert(B5_ECHO_GP == 17u, "Update IRQ register offsets for a different ECHO pin");
_Static_assert(B5_SERVO_GP == 12u, "This port reserves PWM slice 6 for GP12");
_Static_assert(XOSC_MHz == 12, "Recalculate watchdog tick divider when changing reference clock");
_Static_assert(TMCLK_MHz == 125, "Recalculate servo divider when changing system clock");
_Static_assert(B5_ECHO_TIMEOUT_US <= 32767, "pulse field is 15 bits");
/* capture is accessed only by the core-0 IRQ and the core-0 pinned task.
 * DI/EI protects the entire packed read-modify-write. volatile alone does not.
 * Cross-core feedback/status uses a kernel mutex instead of local DI/EI. */
static volatile struct {
    uint32_t started_us, rise_us;
    unsigned width_us:15, phase:2, result:2;
} capture;
static ID mailbox;
static uint8_t feedback_packet[B5_FEEDBACK_BYTES], status_packet[B5_STATUS_BYTES];
static uint8_t scan_packet[B5_SCAN_BYTES];
static b5_core core;
static b5_feedback previous_feedback;
_Static_assert(B5_SERVO_RIGHT_US>=1000 && B5_SERVO_RIGHT_US<=2000 &&
               B5_SERVO_CENTRE_US>=1000 && B5_SERVO_CENTRE_US<=2000 &&
               B5_SERVO_LEFT_US>=1000 && B5_SERVO_LEFT_US<=2000,
               "Calibrated servo pulses must remain within 1000..2000 us");
static uint32_t us_now(void) { return in_w(TIMER_TIMERAWL); }
uint32_t b5_now_ms(void) {
    uint32_t hi, lo, next;
    do { hi=in_w(TIMER_TIMERAWH);lo=in_w(TIMER_TIMERAWL);next=in_w(TIMER_TIMERAWH); } while(hi!=next);
    return (uint32_t)((((uint64_t)hi<<32)|lo)/1000u);
}
static void barrier(void) { __asm__ volatile ("" ::: "memory"); }
/* Raw handler: no kernel calls, no mutex, no console. Avoids taking the
 * SMP scheduler lock before timestamping. Hardware still needs jitter tests. */
static void echo_irq(void) {
    uint32_t events=in_w(ECHO_INTS)&EDGES, now=us_now();
    if(!events)return;
    out_w(ECHO_INTR,events);
    if(capture.phase!=1 && capture.phase!=2)return;
    if(now-capture.started_us>B5_ECHO_TIMEOUT_US || events==EDGES) {
        capture.result=B5_MEASURE_BAD;capture.phase=3;
    } else if(events==EDGE_RISE && capture.phase==1) {
        capture.rise_us=now;capture.phase=2;
    } else if(events==EDGE_FALL && capture.phase==2) {
        uint32_t width=now-capture.rise_us;
        capture.width_us=width<=B5_ECHO_TIMEOUT_US?width:0;
        capture.result=width<=B5_ECHO_TIMEOUT_US?B5_MEASURE_OK:B5_MEASURE_BAD;
        capture.phase=3;
    } else {capture.result=B5_MEASURE_BAD;capture.phase=3;}
}
static bool start_ping(void) {
    UW saved;DI(saved);barrier();
    if(capture.phase!=0){barrier();EI(saved);return false;}
    out_w(ECHO_INTR,EDGES);
    capture.width_us=0;capture.started_us=us_now();
    capture.result=B5_MEASURE_NO_ECHO;capture.phase=1;
    if(gpio_get_val(B5_ECHO_GP)){capture.result=B5_MEASURE_BAD;capture.phase=3;}
    bool trigger=capture.phase==1;
    /* Keep only the 10 us TRIG pulse inside the bounded critical section.
     * No busy-wait is used for the ECHO pulse. */
    if(trigger){
        gpio_set_val(B5_TRIG_GP,1);uint32_t begin=us_now();
        while(us_now()-begin<10u){}
        gpio_set_val(B5_TRIG_GP,0);
    }
    barrier();EI(saved);return true;
}
static bool poll_ping(uint16_t *mm,uint8_t *result) {
    UW saved;DI(saved);barrier();
    if((capture.phase==1 || capture.phase==2) && us_now()-capture.started_us>=B5_ECHO_TIMEOUT_US){
        capture.result=B5_MEASURE_NO_ECHO;capture.phase=3;
    }
    if(capture.phase!=3){barrier();EI(saved);return false;}
    *result=capture.result;
    *mm=(uint16_t)(((uint32_t)capture.width_us*10u+29u)/58u);
    capture.phase=0;barrier();EI(saved);return true;
}
static void cancel_ping(void) {
    UW saved;DI(saved);barrier();capture.phase=0;out_w(ECHO_INTR,EDGES);barrier();EI(saved);
}
static void pulse(unsigned us) {
    if(us>=1000u && us<=2000u)out_w(PWM6+PWM_CHx_CC,us);
}
static void servo(uint8_t slot) {
#if APP_BUDDY5 == 1
    (void)slot;return;
#endif
    if(slot>24u)return;
    int us=slot<=12u?(int)B5_SERVO_RIGHT_US+((int)B5_SERVO_CENTRE_US-(int)B5_SERVO_RIGHT_US)*slot/12:
      (int)B5_SERVO_CENTRE_US+((int)B5_SERVO_LEFT_US-(int)B5_SERVO_CENTRE_US)*(slot-12)/12;
    pulse((unsigned)us);
}
static ER hardware_init(bool with_servo) {
    gpio_set_val(B5_TRIG_GP,0);gpio_set_pin(B5_TRIG_GP,GPIO_MODE_OUT);
    gpio_set_pin(B5_ECHO_GP,GPIO_MODE_IN);
    clr_w(GPIO(B5_ECHO_GP),GPIO_PUE);set_w(GPIO(B5_ECHO_GP),GPIO_PDE);
    /* Abort rather than overwriting an existing core-0 GPIO interrupt owner. */
    for(unsigned i=0;i<4;++i)if(in_w(IO_BANK0_BASE+0x100u+i*4u))return E_OBJ;
    T_DINT handler={.intatr=0,.inthdr=(FP)echo_irq};
    ER er=tk_def_int(GPIO_IRQ,&handler);if(er<E_OK)return er;
    out_w(ECHO_INTR,EDGES);out_w(ECHO_INTE+0x2000u,EDGES); /* atomic SET alias */
    ClearInt(GPIO_IRQ);EnableInt(GPIO_IRQ,0);
    if(with_servo){
        /* Release PWM without resetting other slices (motors use slices 4/5).
         * Slice 6, including GP13, must be reserved to Buddy 5. */
        out_w(RESETS_RESET+0x3000u,RESETS_RESET_PWM);
        uint32_t begin=us_now();
        while(!(in_w(RESETS_RESET_DONE)&RESETS_RESET_PWM))if(us_now()-begin>1000u)return E_SYS;
        out_w(PWM6+PWM_CHx_CSR,0);out_w(PWM6+PWM_CHx_DIV,125u<<4);
        out_w(PWM6+PWM_CHx_CTR,0);out_w(PWM6+PWM_CHx_TOP,19999u);
        out_w(PWM6+PWM_CHx_CC,0);pwm_set_pin(B5_SERVO_GP);
        pulse(B5_SERVO_CENTRE_US);out_w(PWM6+PWM_CHx_CSR,1);
    }
    return E_OK;
}
bool b5_submit_feedback(unsigned mode,bool stopped,bool motion_valid,
                        bool line_valid,bool line_found,unsigned travel_mm,
                        uint32_t measured_at_ms) {
    if(mailbox<=0 || mode>3u)return false;
    /* Saturation cannot turn an over-limit detour into a small distance. */
    if(travel_mm>500u)travel_mm=500u;
    b5_feedback f={.mode=mode,.stopped=stopped,.motion_valid=motion_valid,
      .line_valid=line_valid,.line_found=line_found,.travel_mm=travel_mm,.at_ms=measured_at_ms};
    uint8_t p[B5_FEEDBACK_BYTES];b5_encode_feedback(&f,p);
    if(tk_loc_mtx(mailbox,TMO_POL)<E_OK)return false;
    copy_bytes(feedback_packet,p,sizeof p);tk_unl_mtx(mailbox);return true;
}
bool b5_read_packet(uint8_t out[B5_STATUS_BYTES]) {
    if(!out || mailbox<=0 || tk_loc_mtx(mailbox,TMO_POL)<E_OK)return false;
    copy_bytes(out,status_packet,B5_STATUS_BYTES);tk_unl_mtx(mailbox);return true;
}
bool b5_read_scan(uint8_t out[B5_SCAN_BYTES]) {
    if(!out || mailbox<=0 || tk_loc_mtx(mailbox,TMO_POL)<E_OK)return false;
    copy_bytes(out,scan_packet,sizeof scan_packet);tk_unl_mtx(mailbox);return true;
}
static void publish(void) {
    uint8_t p[B5_STATUS_BYTES];b5_encode_status(&core.s,p);
    if(tk_loc_mtx(mailbox,TMO_POL)<E_OK)return;
    copy_bytes(status_packet,p,sizeof p);copy_bytes(scan_packet,core.scan,sizeof scan_packet);
    tk_unl_mtx(mailbox);
}
static void b5_task(INT stacd,void *exinf) {
    (void)stacd;(void)exinf;
#if TK_SUPPORT_SMP
    if(tk_get_prc()!=1){tm_printf((UB *)"[b5] wrong processor\n");tk_ext_tsk();}
#endif
#if APP_BUDDY5 == 3 || APP_BUDDY5 == 4
    if(!B5_SERVO_CALIBRATED){tm_printf((UB *)"[b5] calibrate servo before scan/control mode\n");tk_ext_tsk();}
#endif
#if APP_BUDDY5 != 4
    /* Stationary bench only: hold both motor channels low. No motor task runs. */
    for(unsigned p=8;p<=11;++p){gpio_set_val(p,0);gpio_set_pin(p,GPIO_MODE_OUT);}
#endif
    ER er=hardware_init(APP_BUDDY5!=1);
    if(er<E_OK){tm_printf((UB *)"[b5] hardware init failed: %d\n",er);tk_ext_tsk();}
    tm_printf((UB *)"[b5] mode=%d TRIG=GP16 ECHO=GP17 servo=GP12; stationary bench modes never drive motors\n",APP_BUDDY5);
#if APP_BUDDY5 == 2
    const unsigned positions[4]={B5_SERVO_CENTRE_US,B5_SERVO_RIGHT_US,B5_SERVO_CENTRE_US,B5_SERVO_LEFT_US};
    for(;;)for(unsigned i=0;i<4;++i){pulse(positions[i]);tm_printf((UB *)"[b5] pulse=%u us\n",positions[i]);tk_dly_tsk(4000);}
#else
    b5_init(&core,(b5_hal){servo,start_ping,poll_ping,cancel_ping},b5_now_ms());
    uint32_t last_ping=b5_now_ms()-B5_PING_PERIOD_MS,last_print=b5_now_ms();bool waiting=false;
    for(;;){
        uint32_t now=b5_now_ms();
#if APP_BUDDY5 == 1
        if(waiting){
            uint16_t mm;uint8_t result;
            if(poll_ping(&mm,&result)){
                waiting=false;core.s.at_ms=now;core.s.range_ms=now;
                core.s.front.valid=result==B5_MEASURE_OK && mm>=20 && mm<=4000;
                core.s.front.mm=core.s.front.valid?mm:0;
                core.s.faults=core.s.front.valid?0:(result==B5_MEASURE_NO_ECHO?B5_F_NO_ECHO:B5_F_BAD_RANGE);
                core.s.request=B5_STOP; /* test firmware does not authorise motion */
            }
        } else if(now-last_ping>=B5_PING_PERIOD_MS && start_ping()){last_ping=now;waiting=true;}
#else
#if APP_BUDDY5 == 3
        /* SIMULATED stopped feedback is valid ONLY for this stationary bench.
         * Mode 4 requires real controller feedback; it never synthesises it. */
        previous_feedback=(b5_feedback){.motion_valid=1,.stopped=1,.at_ms=now};
#else
        uint8_t p[B5_FEEDBACK_BYTES];
        if(tk_loc_mtx(mailbox,TMO_POL)>=E_OK){
            copy_bytes(p,feedback_packet,sizeof p);tk_unl_mtx(mailbox);
            if(!b5_decode_feedback(p,&previous_feedback))previous_feedback.motion_valid=0;
        }
#endif
        b5_step(&core,&previous_feedback,now);
#endif
        publish();
#if APP_BUDDY5 != 4
        if(now-last_print>=500u){
            last_print=now;
            if(core.s.front.valid)tm_printf((UB *)"[b5] mm=%u state=%u request=%u faults=0x%x samples=%u\n",core.s.front.mm,core.s.state,core.s.request,core.s.faults,core.s.count);
            else tm_printf((UB *)"[b5] range=INVALID state=%u faults=0x%x\n",core.s.state,core.s.faults);
        }
#endif
        tk_dly_tsk(1);
    }
#endif
}
ER b5_start(void) {
    if(mailbox>0)return E_OBJ;
    b5_status initial={.state=B5_FAULT,.request=B5_STOP,.faults=B5_F_FEEDBACK};
    b5_encode_status(&initial,status_packet);
    /* Kernel timing uses SysTick; ECHO capture uses the RP2040 timer. Its
     * one-microsecond tick comes from the 12 MHz clk_ref watchdog divider.
     * Initialise before exposing the mailbox; never reset an active counter. */
    out_w(RESETS_RESET+0x3000u,RESETS_RESET_TIMER);
    unsigned tries=100000u;
    while(!(in_w(RESETS_RESET_DONE)&RESETS_RESET_TIMER))if(--tries==0)return E_SYS;
    out_w(0x4005802cu,0x200u|12u); /* WATCHDOG_TICK: enable, 12 cycles */
    out_w(TIMER_PAUSE,0);
    uint32_t first=us_now();tries=100000u;
    while(us_now()==first)if(--tries==0)return E_SYS;
    T_CMTX m={.mtxatr=TA_TFIFO|TA_INHERIT};mailbox=tk_cre_mtx(&m);
    if(mailbox<=0)return mailbox;
    T_CTSK t={.tskatr=TA_HLNG|TA_RNG3,.task=b5_task,.itskpri=6,.stksz=4096};
#if TK_SUPPORT_SMP
    t.tskatr|=TA_ASSPRC;t.assprc=TP_PRC1;
#endif
    ID id=tk_cre_tsk(&t);
    if(id<=0){tk_del_mtx(mailbox);mailbox=0;return id;}
    ER er=tk_sta_tsk(id,0);
    if(er<E_OK){tk_del_tsk(id);tk_del_mtx(mailbox);mailbox=0;}
    return er;
}
#endif
