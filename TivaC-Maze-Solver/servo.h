/**
 * servo.h — PWM-based hobby servo driver for TM4C123GH6PM
 * =========================================================
 * Pan  servo  → PB6  (PWM Module 0, Generator 0, Output 0  = M0PWM0)
 * Tilt servo  → PB7  (PWM Module 0, Generator 0, Output 1  = M0PWM1)
 *
 * PWM spec:
 *   Period  : 20 ms  (50 Hz)
 *   Pulse   :  1 ms  →   0°
 *              1.5 ms →  90°
 *              2 ms  → 180°
 *
 * System clock assumed: 80 MHz
 * PWM clock divisor  : /64  → PWM clock = 1.25 MHz
 * Period ticks       : 25000
 * Min pulse ticks    :  1250  (1 ms)
 * Max pulse ticks    :  2500  (2 ms)
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/** Initialise PWM hardware and move both servos to 90° (centre). */
void Servo_Init(void);

/**
 * Set both servo angles simultaneously.
 * @param pan_deg   Pan  angle [0, 180]
 * @param tilt_deg  Tilt angle [0, 180]
 * Values outside [0, 180] are silently clamped.
 */
void Servo_SetAngles(uint32_t pan_deg, uint32_t tilt_deg);

/** Set only the pan servo angle [0, 180]. */
void Servo_SetPan(uint32_t deg);

/** Set only the tilt servo angle [0, 180]. */
void Servo_SetTilt(uint32_t deg);

/** Return current pan angle (last commanded value). */
uint32_t Servo_GetPan(void);

/** Return current tilt angle (last commanded value). */
uint32_t Servo_GetTilt(void);

#endif /* SERVO_H */
