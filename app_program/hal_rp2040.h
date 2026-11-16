/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * hal_rp2040.h
 * The robot's hardware layer for the Pico W + Robo Pico: peripheral set-up,
 * the 1 kHz acquisition tick, IR sampling, motor/servo PWM, encoder and
 * ultrasonic edge capture, I2C and buttons.  Nothing here makes a decision;
 * all logic lives in the portable modules.
 *
 * Ownership (RP2040 safety rule 6: treat shared blocks as single-owner):
 *   TIMER alarm 0 + ADC     the 1 kHz tick ISR (hal_tick_start)
 *   IO_BANK0 interrupt      encoder edges and the ECHO edge
 *   PWM slices 4, 5         Motion task only (via hal_motor_set)
 *   PWM slice 6             Obstacle task only (via hal_servo_set_pulse_us)
 *   I2C1                    IMU task only
 * The template's sample ADC/I2C drivers must stay closed: this layer drives
 * those blocks directly.
 *
 * Built for the single-core (SMP=0) profile; peripheral interrupts are
 * enabled on the boot core, which owns them in the SMP profile too.
 */
#ifndef HAL_RP2040_H
#define HAL_RP2040_H

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>

#include "encoder.h"

/* Event-flag bits set by the tick. */
#define HAL_TICK_MOTION     (1U << 0)       /* every 5 ms */
#define HAL_TICK_CONTROL    (1U << 1)       /* every 10 ms */
#define HAL_TICK_IMU        (1U << 2)       /* every 20 ms */

typedef struct {
    uint16_t adc[3];                        /* left, centre, right (line_sensor order) */
    uint32_t ms;                            /* tick time of the frame */
} hal_ir_frame_t;

typedef struct {
    bool     finished;                      /* the measurement is over */
    bool     complete;                      /* a whole echo pulse was seen */
    uint32_t echo_us;
} hal_echo_t;

void hal_init(void);

uint32_t hal_time_ms(void);                 /* from the 1 kHz tick: monotonic */
uint32_t hal_time_us(void);                 /* free-running 1 MHz counter */
void hal_delay_us(uint32_t us);

/* Start the 1 kHz tick.  Sets HAL_TICK_* bits on 'flag_id' and fills the IR ring. */
ER hal_tick_start(ID flag_id);
void hal_tick_ack(uint32_t bit);            /* the task has run: clears the overrun marker */
uint32_t hal_tick_overruns(uint32_t bit);   /* times a tick came before the last was handled */

/* IR frames (single consumer). */
bool hal_ir_pop(hal_ir_frame_t *frame);
uint32_t hal_ir_overflows(void);            /* frames dropped because the ring was full */
uint32_t hal_ir_levels(void);               /* raw DO pin levels of the latest frame: bit 0 left, 1 centre, 2 right (1 = pin high) */
void hal_ir_edges(uint32_t edges[3]);       /* DO edges counted since start-up: left, centre, right */
uint32_t hal_adc_max_isr_us(void);          /* longest tick ISR, a CPU-cost measurement */

void hal_motor_set(int16_t left_pwm, int16_t right_pwm);     /* signed per-mille */
void hal_servo_set_pulse_us(uint16_t us);

/* Start the encoder/ECHO edge interrupt (once, after hal_init). */
ER hal_gpio_irq_start(void);

void hal_encoders_snapshot(encoder_snapshot_t *left, encoder_snapshot_t *right);

void hal_us_trigger(void);
hal_echo_t hal_us_poll(void);

bool hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
bool hal_i2c_write(uint8_t addr, uint8_t reg, uint8_t value);

bool hal_button_start(void);                /* raw level, true = pressed */
bool hal_button_stop(void);

#endif /* HAL_RP2040_H */
