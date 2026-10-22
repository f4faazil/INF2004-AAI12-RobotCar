/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * hal_rp2040.c
 * Hardware layer for the Pico W + Robo Pico.  See hal_rp2040.h.
 *
 * NOT YET RUN ON THE CAR.  Register sequences follow the RP2040 datasheet and
 * the Pico SDK's drivers, and the file compiles cleanly with strict warnings,
 * but every peripheral here still needs the bench checks listed in
 * docs/ROBOT_FIRMWARE.md.
 */
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <sys/machine.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#include "hal_rp2040.h"
#include "robot_config.h"
#include "robot_port.h"

/* ---- registers not defined by the kernel's sysdef.h -------------------- */

#define ADC_BASE_ADDR       (0x4004C000UL)
#define ADC_CS_REG          (ADC_BASE_ADDR + 0x00UL)
#define ADC_RESULT_REG      (ADC_BASE_ADDR + 0x04UL)
#define ADC_FCS_REG         (ADC_BASE_ADDR + 0x08UL)
#define ADC_DIV_REG         (ADC_BASE_ADDR + 0x10UL)
#define ADC_INTE_REG        (ADC_BASE_ADDR + 0x18UL)
#define ADC_CS_EN_BIT       (1UL << 0)
#define ADC_CS_START_ONCE   (1UL << 2)
#define ADC_CS_READY_BIT    (1UL << 8)
#define ADC_CS_AINSEL_LSB   (12U)
#define ADC_RESULT_MASK     (0x0FFFUL)
#define ADC_READY_SPINS     (2000U)

#define WATCHDOG_TICK_REG   (0x4005802CUL)
#define WATCHDOG_TICK_RUN   (1UL << 9)
#define TICK_CYCLES_1US     (12UL)          /* 12 MHz reference clock */

#define IO_INTR_BASE        (IO_BANK0_BASE + 0x0F0UL)
#define IO_PROC0_INTE_BASE  (IO_BANK0_BASE + 0x100UL)
#define IO_PROC0_INTS_BASE  (IO_BANK0_BASE + 0x120UL)
#define IO_EDGE_BOTH        (0x0CUL)        /* EDGE_LOW | EDGE_HIGH per pin */
#define IO_PINS_PER_REG     (8U)
#define IO_BITS_PER_PIN     (4U)

#define I2C1_BASE_ADDR      (0x40048000UL)
#define I2C_CON             (I2C1_BASE_ADDR + 0x00UL)
#define I2C_TAR             (I2C1_BASE_ADDR + 0x04UL)
#define I2C_DATA_CMD        (I2C1_BASE_ADDR + 0x10UL)
#define I2C_SS_HCNT         (I2C1_BASE_ADDR + 0x14UL)
#define I2C_SS_LCNT         (I2C1_BASE_ADDR + 0x18UL)
#define I2C_RX_TL           (I2C1_BASE_ADDR + 0x38UL)
#define I2C_TX_TL           (I2C1_BASE_ADDR + 0x3CUL)
#define I2C_CLR_TX_ABRT     (I2C1_BASE_ADDR + 0x54UL)
#define I2C_ENABLE          (I2C1_BASE_ADDR + 0x6CUL)
#define I2C_STATUS          (I2C1_BASE_ADDR + 0x70UL)
#define I2C_RXFLR           (I2C1_BASE_ADDR + 0x78UL)
#define I2C_SDA_HOLD        (I2C1_BASE_ADDR + 0x7CUL)
#define I2C_TX_ABRT_SOURCE  (I2C1_BASE_ADDR + 0x80UL)
#define I2C_FS_SPKLEN       (I2C1_BASE_ADDR + 0xA0UL)
#define I2C_CON_MASTER      (1UL << 0)
#define I2C_CON_SPEED_STD   (1UL << 1)
#define I2C_CON_RESTART_EN  (1UL << 5)
#define I2C_CON_SLAVE_OFF   (1UL << 6)
#define I2C_CON_TX_EMPTY    (1UL << 8)
#define I2C_CMD_READ        (1UL << 8)
#define I2C_CMD_STOP        (1UL << 9)
#define I2C_CMD_RESTART     (1UL << 10)
#define I2C_STATUS_TFNF     (1UL << 1)
#define I2C_STATUS_TFE      (1UL << 2)
#define I2C_STATUS_ACTIVITY (1UL << 0)
#define I2C_TIMEOUT_US      (5000U)
#define PERI_CLK_HZ         (125000000UL)

#define TIMER_IRQ_0         (0U)
#define IO_IRQ_BANK0        (13U)
#define TICK_IRQ_LEVEL      (1)
#define GPIO_IRQ_LEVEL      (2)

#define ALARM_PERIOD_US     (ROBOT_ADC_FRAME_US)
#define ALARM_MIN_AHEAD_US  (4UL)

#define PWM_TOP_MOTOR       ((PERI_CLK_HZ / ROBOT_PWM_HZ) - 1UL)
#define PWM_TOP_SERVO       (19999UL)       /* 50 Hz with a 1 MHz count */
#define PWM_DIV_ONE         (1UL << 4)      /* divider 1.0 (8.4 fixed point) */
#define PWM_DIV_SERVO       (125UL << 4)    /* 125 MHz / 125 = 1 MHz */
#define PWM_CC_B_SHIFT      (16U)

#define PWM_SLICE(pin)      (((pin) >> 1) & 7U)
#define PWM_BASE_OF(slice)  (PWM_BASE + (0x14UL * (slice)))

#define ECHO_NONE           (0U)
#define ECHO_WAITING        (1U)
#define ECHO_DONE           (2U)

/* ---- state ------------------------------------------------------------- */

static volatile uint32_t g_tick_ms;
static volatile uint32_t g_next_alarm;
static ID g_flag_id;
static volatile uint32_t g_pending;
static volatile uint32_t g_overrun[3];
static volatile uint32_t g_isr_max_us;

static hal_ir_frame_t g_ring[ROBOT_IR_RING_FRAMES];
static volatile uint32_t g_ring_head;
static volatile uint32_t g_ring_tail;
static volatile uint32_t g_ring_overflows;
static volatile uint32_t g_adc_timeouts;

static encoder_t g_enc_left;
static encoder_t g_enc_right;
static volatile int8_t g_dir_left = 1;
static volatile int8_t g_dir_right = 1;

static volatile uint32_t g_echo_state;
static volatile uint32_t g_echo_rise_us;
static volatile uint32_t g_echo_width_us;
static volatile bool g_echo_rise_seen;
static volatile uint32_t g_echo_trigger_us;

#if TK_SUPPORT_SMP
static T_SPLOCK g_lock;
#endif

/* ---- critical section (robot_port.h) ------------------------------------ */

robot_irq_state_t robot_irq_save(void)
{
    UINT status;

#if TK_SUPPORT_SMP
    (void)ISpinLock(&g_lock, &status);
#else
    DI(status);
#endif
    return (robot_irq_state_t)status;
}

void robot_irq_restore(robot_irq_state_t state)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&g_lock, (UINT)state);
#else
    UINT status = (UINT)state;

    EI(status);
#endif
}

/* ---- time ----------------------------------------------------------------- */

uint32_t hal_time_ms(void)
{
    return g_tick_ms;
}

uint32_t hal_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

void hal_delay_us(uint32_t us)
{
    uint32_t start = hal_time_us();

    while ((uint32_t)(hal_time_us() - start) < us) {
        /* spin: used only for the 10 us trigger pulse */
    }
}

/* ---- pins and peripherals --------------------------------------------------- */

static void release_reset(UW mask)
{
    clr_w(RESETS_RESET, mask);
    while ((in_w(RESETS_RESET_DONE) & mask) != mask) {
        /* wait: releasing a block that is already live returns at once */
    }
}

static void pin_function(uint32_t pin, UW function)
{
    out_w(GPIO(pin), GPIO_IE | GPIO_DRIVE_4MA | GPIO_SHEMITT);
    out_w(GPIO_CTRL(pin), function);
}

static void pin_output_low(uint32_t pin)
{
    pin_function(pin, GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_OUT_CLR, 1UL << pin);
    out_w(GPIO_OE_SET, 1UL << pin);
}

static void pin_input(uint32_t pin, bool pull_up)
{
    out_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT | (pull_up ? GPIO_PUE : 0UL));
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_OE_CLR, 1UL << pin);
}

static void pin_adc(uint32_t pin)
{
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
    out_w(GPIO(pin), GPIO_DRIVE_4MA | GPIO_SHEMITT);     /* input buffer off, no pulls */
}

static void pwm_slice_init(uint32_t slice, UW top, UW div)
{
    UW base = PWM_BASE_OF(slice);

    out_w(base + PWM_CHx_CSR, 0);
    out_w(base + PWM_CHx_DIV, div);
    out_w(base + PWM_CHx_TOP, top);
    out_w(base + PWM_CHx_CC, 0);
    out_w(base + PWM_CHx_CTR, 0);
    out_w(base + PWM_CHx_CSR, PWM_CH_CSR_EN);
}

static void io_irq_enable_edges(uint32_t pin)
{
    UW reg = IO_PROC0_INTE_BASE + (4UL * (pin / IO_PINS_PER_REG));
    UW shift = (pin % IO_PINS_PER_REG) * IO_BITS_PER_PIN;

    out_w(IO_INTR_BASE + (4UL * (pin / IO_PINS_PER_REG)), IO_EDGE_BOTH << shift);
    set_w(reg, IO_EDGE_BOTH << shift);
}

static void i2c_init(void)
{
    UW period = (PERI_CLK_HZ + (ROBOT_I2C_HZ / 2UL)) / ROBOT_I2C_HZ;
    UW lcnt = (period * 3UL) / 5UL;
    UW hcnt = period - lcnt;

    pin_function(ROBOT_PIN_I2C_SDA, GPIO_CTRL_FUNCSEL_I2C);
    pin_function(ROBOT_PIN_I2C_SCL, GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(ROBOT_PIN_I2C_SDA), GPIO_IE | GPIO_PUE | GPIO_DRIVE_4MA | GPIO_SHEMITT);
    out_w(GPIO(ROBOT_PIN_I2C_SCL), GPIO_IE | GPIO_PUE | GPIO_DRIVE_4MA | GPIO_SHEMITT);

    out_w(I2C_ENABLE, 0);
    out_w(I2C_CON, I2C_CON_MASTER | I2C_CON_SPEED_STD | I2C_CON_RESTART_EN |
                   I2C_CON_SLAVE_OFF | I2C_CON_TX_EMPTY);
    out_w(I2C_SS_HCNT, hcnt);
    out_w(I2C_SS_LCNT, lcnt);
    out_w(I2C_FS_SPKLEN, (lcnt < 16UL) ? 1UL : (lcnt / 16UL));
    out_w(I2C_SDA_HOLD, ((PERI_CLK_HZ * 3UL) / 10000000UL) + 1UL);
    out_w(I2C_TX_TL, 0);
    out_w(I2C_RX_TL, 0);
}

void hal_init(void)
{
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&g_lock);
#endif
    /* Blocks the kernel's board setup leaves in reset (it only releases what
     * its own sample drivers need). */
    release_reset(RESETS_RESET_PWM | RESETS_RESET_I2C1 | RESETS_RESET_TIMER |
                  RESETS_RESET_IO_BANK0 | RESETS_RESET_PADS_BANK0 | RESETS_RESET_ADC);
    out_w(WATCHDOG_TICK_REG, TICK_CYCLES_1US | WATCHDOG_TICK_RUN);  /* TIMER counts in us */

    /* motors */
    pin_function(ROBOT_PIN_MOTOR_R_FWD, GPIO_CTRL_FUNCSEL_PWM);
    pin_function(ROBOT_PIN_MOTOR_R_REV, GPIO_CTRL_FUNCSEL_PWM);
    pin_function(ROBOT_PIN_MOTOR_L_FWD, GPIO_CTRL_FUNCSEL_PWM);
    pin_function(ROBOT_PIN_MOTOR_L_REV, GPIO_CTRL_FUNCSEL_PWM);
    pwm_slice_init(PWM_SLICE(ROBOT_PIN_MOTOR_R_FWD), PWM_TOP_MOTOR, PWM_DIV_ONE);
    pwm_slice_init(PWM_SLICE(ROBOT_PIN_MOTOR_L_FWD), PWM_TOP_MOTOR, PWM_DIV_ONE);

    /* servo: 50 Hz, parked straight ahead */
    pin_function(ROBOT_PIN_SERVO, GPIO_CTRL_FUNCSEL_PWM);
    pwm_slice_init(PWM_SLICE(ROBOT_PIN_SERVO), PWM_TOP_SERVO, PWM_DIV_SERVO);
    hal_servo_set_pulse_us(servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, (int16_t)SCAN_FORWARD_DEG));

    /* ultrasonic */
    pin_output_low(ROBOT_PIN_TRIG);
    pin_input(ROBOT_PIN_ECHO, false);

    /* IR: ADC inputs, converter on, FIFO and interrupts off (this layer polls) */
    pin_adc(26U);
    pin_adc(27U);
    pin_adc(28U);
    out_w(ADC_CS_REG, ADC_CS_EN_BIT);
    while ((in_w(ADC_CS_REG) & ADC_CS_READY_BIT) == 0UL) {
        /* wait for the converter to power up */
    }
    out_w(ADC_FCS_REG, 0);
    out_w(ADC_INTE_REG, 0);
    out_w(ADC_DIV_REG, 0);

    /* buttons */
    pin_input(ROBOT_PIN_BTN_START, true);
    pin_input(ROBOT_PIN_BTN_STOP, true);

    /* encoders */
    encoder_init(&g_enc_left, ROBOT_ENC_L_INVERT != 0);
    encoder_init(&g_enc_right, ROBOT_ENC_R_INVERT != 0);
    pin_input(ROBOT_PIN_ENC_L_A, true);
    pin_input(ROBOT_PIN_ENC_L_B, true);
    pin_input(ROBOT_PIN_ENC_R_A, true);
    pin_input(ROBOT_PIN_ENC_R_B, true);

    i2c_init();
}

/* ---- motors and servo ------------------------------------------------------ */

static void drive_slice(uint32_t fwd_pin, int32_t pwm, bool invert)
{
    UW base = PWM_BASE_OF(PWM_SLICE(fwd_pin));
    int32_t signed_pwm = invert ? -pwm : pwm;
    int32_t mag = (signed_pwm < 0) ? -signed_pwm : signed_pwm;
    UW duty;
    UW cc;

    if (mag > MOTION_PWM_MAX) {
        mag = MOTION_PWM_MAX;
    }
    duty = (UW)(((UW)mag * (PWM_TOP_MOTOR + 1UL)) / (UW)MOTION_PWM_MAX);
    /* channel A (even pin) drives forward, channel B (odd pin) backward */
    cc = (signed_pwm >= 0) ? duty : (duty << PWM_CC_B_SHIFT);
    out_w(base + PWM_CHx_CC, cc);
}

void hal_motor_set(int16_t left_pwm, int16_t right_pwm)
{
    g_dir_left = (left_pwm >= 0) ? 1 : -1;
    g_dir_right = (right_pwm >= 0) ? 1 : -1;
    drive_slice(ROBOT_PIN_MOTOR_L_FWD, left_pwm, ROBOT_MOTOR_L_INVERT != 0);
    drive_slice(ROBOT_PIN_MOTOR_R_FWD, right_pwm, ROBOT_MOTOR_R_INVERT != 0);
}

void hal_servo_set_pulse_us(uint16_t us)
{
    UW base = PWM_BASE_OF(PWM_SLICE(ROBOT_PIN_SERVO));

    out_w(base + PWM_CHx_CC, (UW)us);       /* channel A, 1 us per count */
}

/* ---- the 1 kHz tick: IR sampling and task release ---------------------------- */

static uint16_t adc_read(uint32_t channel)
{
    uint32_t spins = ADC_READY_SPINS;

    out_w(ADC_CS_REG, ADC_CS_EN_BIT | ((UW)channel << ADC_CS_AINSEL_LSB) | ADC_CS_START_ONCE);
    while (((in_w(ADC_CS_REG) & ADC_CS_READY_BIT) == 0UL) && (spins > 0U)) {
        spins--;
    }
    if (spins == 0U) {
        g_adc_timeouts++;
    }
    return (uint16_t)(in_w(ADC_RESULT_REG) & ADC_RESULT_MASK);
}

static void release_task(uint32_t bit, uint32_t index)
{
    if ((g_pending & bit) != 0U) {
        g_overrun[index]++;                 /* the last tick was never handled */
    }
    g_pending |= bit;
    (void)tk_set_flg(g_flag_id, bit);
}

static void tick_isr(UINT intno)
{
    const uint32_t start = hal_time_us();
    uint32_t used;
    uint32_t now;
    uint32_t ms;

    out_w(TIMER_INTR, 1UL);                 /* clear alarm 0 */
    g_next_alarm += ALARM_PERIOD_US;
    now = hal_time_us();
    if ((int32_t)(g_next_alarm - now) < (int32_t)ALARM_MIN_AHEAD_US) {
        g_next_alarm = now + ALARM_MIN_AHEAD_US;   /* never write a target in the past */
    }
    out_w(TIMER_ALARM0, g_next_alarm);

    g_tick_ms++;
    ms = g_tick_ms;

    {
        uint32_t head = g_ring_head;

        if ((head - g_ring_tail) >= ROBOT_IR_RING_FRAMES) {
            g_ring_overflows++;             /* ring full: drop this frame, the consumer is told */
        } else {
            hal_ir_frame_t *slot = &g_ring[head % ROBOT_IR_RING_FRAMES];

            (void)adc_read(ROBOT_ADC_CH_LEFT);      /* first read after a mux change settles */
            slot->adc[0] = adc_read(ROBOT_ADC_CH_LEFT);
            (void)adc_read(ROBOT_ADC_CH_CENTRE);
            slot->adc[1] = adc_read(ROBOT_ADC_CH_CENTRE);
            (void)adc_read(ROBOT_ADC_CH_RIGHT);
            slot->adc[2] = adc_read(ROBOT_ADC_CH_RIGHT);
            slot->ms = ms;
            Asm("dmb" ::: "memory");
            g_ring_head = head + 1U;
        }
    }

    if ((ms % ROBOT_MOTION_PERIOD_MS) == 0U) {
        release_task(HAL_TICK_MOTION, 0U);
    }
    if ((ms % ROBOT_CONTROL_PERIOD_MS) == 0U) {
        release_task(HAL_TICK_CONTROL, 1U);
    }
    if ((ms % ROBOT_IMU_PERIOD_MS) == 0U) {
        release_task(HAL_TICK_IMU, 2U);
    }

    used = hal_time_us() - start;
    if (used > g_isr_max_us) {
        g_isr_max_us = used;
    }
    ClearInt(intno);
}

ER hal_tick_start(ID flag_id)
{
    T_DINT dint;
    ER err;

    g_flag_id = flag_id;
    dint.intatr = TA_HLNG;
    dint.inthdr = tick_isr;
    err = tk_def_int(TIMER_IRQ_0, &dint);
    if (err < E_OK) {
        return err;
    }
    g_next_alarm = hal_time_us() + ALARM_PERIOD_US;
    out_w(TIMER_INTR, 1UL);
    out_w(TIMER_ALARM0, g_next_alarm);
    set_w(TIMER_INTE, 1UL);
    ClearInt(TIMER_IRQ_0);
    EnableInt(TIMER_IRQ_0, TICK_IRQ_LEVEL);
    return E_OK;
}

void hal_tick_ack(uint32_t bit)
{
    robot_irq_state_t state = robot_irq_save();

    g_pending &= ~bit;
    robot_irq_restore(state);
}

uint32_t hal_tick_overruns(uint32_t bit)
{
    uint32_t index = (bit == HAL_TICK_MOTION) ? 0U : ((bit == HAL_TICK_CONTROL) ? 1U : 2U);

    return g_overrun[index];
}

bool hal_ir_pop(hal_ir_frame_t *frame)
{
    uint32_t tail = g_ring_tail;

    if (tail == g_ring_head) {
        return false;
    }
    *frame = g_ring[tail % ROBOT_IR_RING_FRAMES];
    Asm("dmb" ::: "memory");
    g_ring_tail = tail + 1U;
    return true;
}

uint32_t hal_ir_overflows(void)
{
    return g_ring_overflows;
}

uint32_t hal_adc_max_isr_us(void)
{
    return g_isr_max_us;
}

/* ---- GPIO edges: encoders and the ultrasonic ECHO ---------------------------- */

static bool edge_pending(uint32_t pin)
{
    UW ints = in_w(IO_PROC0_INTS_BASE + (4UL * (pin / IO_PINS_PER_REG)));

    return ((ints >> ((pin % IO_PINS_PER_REG) * IO_BITS_PER_PIN)) & IO_EDGE_BOTH) != 0UL;
}

static void edge_clear(uint32_t pin)
{
    out_w(IO_INTR_BASE + (4UL * (pin / IO_PINS_PER_REG)),
          IO_EDGE_BOTH << ((pin % IO_PINS_PER_REG) * IO_BITS_PER_PIN));
}

static void gpio_isr(UINT intno)
{
    const uint32_t now = hal_time_us();
    const UW levels = in_w(GPIO_IN);

#if ROBOT_ENCODER_SINGLE_CHANNEL
    if (edge_pending(ROBOT_PIN_ENC_L_A)) {
        encoder_update_single(&g_enc_left, g_dir_left, now);
    }
    if (edge_pending(ROBOT_PIN_ENC_R_A)) {
        encoder_update_single(&g_enc_right, g_dir_right, now);
    }
#else
    if (edge_pending(ROBOT_PIN_ENC_L_A) || edge_pending(ROBOT_PIN_ENC_L_B)) {
        encoder_update(&g_enc_left, ((levels >> ROBOT_PIN_ENC_L_A) & 1UL) != 0UL,
                       ((levels >> ROBOT_PIN_ENC_L_B) & 1UL) != 0UL, now);
    }
    if (edge_pending(ROBOT_PIN_ENC_R_A) || edge_pending(ROBOT_PIN_ENC_R_B)) {
        encoder_update(&g_enc_right, ((levels >> ROBOT_PIN_ENC_R_A) & 1UL) != 0UL,
                       ((levels >> ROBOT_PIN_ENC_R_B) & 1UL) != 0UL, now);
    }
#endif

    if (edge_pending(ROBOT_PIN_ECHO)) {
        if (((levels >> ROBOT_PIN_ECHO) & 1UL) != 0UL) {
            g_echo_rise_us = now;
            g_echo_rise_seen = true;
        } else if ((g_echo_state == ECHO_WAITING) && g_echo_rise_seen) {
            g_echo_width_us = now - g_echo_rise_us;
            g_echo_state = ECHO_DONE;
        } else {
            /* a falling edge with no rising edge: ignore */
        }
    }

    edge_clear(ROBOT_PIN_ENC_L_A);
    edge_clear(ROBOT_PIN_ENC_L_B);
    edge_clear(ROBOT_PIN_ENC_R_A);
    edge_clear(ROBOT_PIN_ENC_R_B);
    edge_clear(ROBOT_PIN_ECHO);
    ClearInt(intno);
}

static bool g_gpio_irq_ready;

ER hal_gpio_irq_start(void)
{
    T_DINT dint;
    ER err;

    if (g_gpio_irq_ready) {
        return E_OK;
    }
    dint.intatr = TA_HLNG;
    dint.inthdr = gpio_isr;
    err = tk_def_int(IO_IRQ_BANK0, &dint);
    if (err < E_OK) {
        return err;
    }
    io_irq_enable_edges(ROBOT_PIN_ENC_L_A);
    io_irq_enable_edges(ROBOT_PIN_ENC_R_A);
    io_irq_enable_edges(ROBOT_PIN_ECHO);
#if !ROBOT_ENCODER_SINGLE_CHANNEL
    io_irq_enable_edges(ROBOT_PIN_ENC_L_B);
    io_irq_enable_edges(ROBOT_PIN_ENC_R_B);
#endif
    ClearInt(IO_IRQ_BANK0);
    EnableInt(IO_IRQ_BANK0, GPIO_IRQ_LEVEL);
    g_gpio_irq_ready = true;
    return E_OK;
}

void hal_encoders_snapshot(encoder_snapshot_t *left, encoder_snapshot_t *right)
{
    encoder_snapshot(&g_enc_left, left);
    encoder_snapshot(&g_enc_right, right);
}

void hal_us_trigger(void)
{
    g_echo_rise_seen = false;
    g_echo_state = ECHO_WAITING;
    g_echo_trigger_us = hal_time_us();
    out_w(GPIO_OUT_SET, 1UL << ROBOT_PIN_TRIG);
    hal_delay_us(10U);                      /* HC-SR04 needs a 10 us pulse */
    out_w(GPIO_OUT_CLR, 1UL << ROBOT_PIN_TRIG);
}

hal_echo_t hal_us_poll(void)
{
    hal_echo_t result;

    result.finished = false;
    result.complete = false;
    result.echo_us = 0U;
    if (g_echo_state == ECHO_DONE) {
        result.finished = true;
        result.complete = true;
        result.echo_us = g_echo_width_us;
        g_echo_state = ECHO_NONE;
    } else if ((g_echo_state == ECHO_WAITING) &&
               ((uint32_t)(hal_time_us() - g_echo_trigger_us) > ROBOT_US_ECHO_TIMEOUT_US)) {
        result.finished = true;             /* no (complete) echo */
        g_echo_state = ECHO_NONE;
    } else {
        /* still measuring */
    }
    return result;
}

/* ---- I2C1 (bounded: a stuck bus never hangs the IMU task) --------------------- */

static bool i2c_wait(UW reg, UW mask, UW want)
{
    uint32_t start = hal_time_us();

    while ((in_w(reg) & mask) != want) {
        if ((uint32_t)(hal_time_us() - start) > I2C_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

static void i2c_begin(uint8_t addr)
{
    out_w(I2C_ENABLE, 0);
    out_w(I2C_TAR, (UW)addr);
    out_w(I2C_ENABLE, 1);
}

static bool i2c_aborted(void)
{
    if (in_w(I2C_TX_ABRT_SOURCE) != 0UL) {
        (void)in_w(I2C_CLR_TX_ABRT);
        return true;
    }
    return false;
}

bool hal_i2c_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    i2c_begin(addr);
    if (!i2c_wait(I2C_STATUS, I2C_STATUS_TFNF, I2C_STATUS_TFNF)) {
        return false;
    }
    out_w(I2C_DATA_CMD, (UW)reg);
    out_w(I2C_DATA_CMD, (UW)value | I2C_CMD_STOP);
    if (!i2c_wait(I2C_STATUS, I2C_STATUS_TFE, I2C_STATUS_TFE)) {
        return false;
    }
    (void)i2c_wait(I2C_STATUS, I2C_STATUS_ACTIVITY, 0UL);
    return !i2c_aborted();
}

bool hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    if (len == 0U) {
        return false;
    }
    i2c_begin(addr);
    out_w(I2C_DATA_CMD, (UW)reg);
    for (i = 0U; i < len; i++) {
        UW cmd = I2C_CMD_READ;

        if (i == 0U) {
            cmd |= I2C_CMD_RESTART;
        }
        if (i == (uint8_t)(len - 1U)) {
            cmd |= I2C_CMD_STOP;
        }
        out_w(I2C_DATA_CMD, cmd);
    }
    for (i = 0U; i < len; i++) {
        uint32_t start = hal_time_us();

        while (in_w(I2C_RXFLR) == 0UL) {
            if (i2c_aborted() || ((uint32_t)(hal_time_us() - start) > I2C_TIMEOUT_US)) {
                return false;
            }
        }
        buf[i] = (uint8_t)(in_w(I2C_DATA_CMD) & 0xFFUL);
    }
    return !i2c_aborted();
}

/* ---- buttons ---------------------------------------------------------------------- */

bool hal_button_start(void)
{
    return ((in_w(GPIO_IN) >> ROBOT_PIN_BTN_START) & 1UL) == 0UL;     /* active low */
}

bool hal_button_stop(void)
{
    return ((in_w(GPIO_IN) >> ROBOT_PIN_BTN_STOP) & 1UL) == 0UL;
}
