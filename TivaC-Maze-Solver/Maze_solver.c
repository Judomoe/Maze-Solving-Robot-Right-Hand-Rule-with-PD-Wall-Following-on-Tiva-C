#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

// ── Globals ───────────────────────────────────────────────────
volatile uint32_t distance  = 0; // Front sensor
volatile uint32_t distance2 = 0; // Right sensor

#define PWM_FREQUENCY   1000
#define PWM_MAX         255

#define WALL_TARGET     12.5f   // target cm from right wall
#define WALL_KP         10.0f   // proportional gain
#define WALL_KD 20.0f
float prevError = 0.0f;

// ── Helpers ───────────────────────────────────────────────────
static int32_t constrainVal(int32_t val, int32_t lo, int32_t hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static uint32_t toPWMTicks(int val, uint32_t period) {
    return ((uint32_t)val * period) / PWM_MAX;
}

// ── PWM Init ──────────────────────────────────────────────────
// ENA = PB6 = M0PWM0  (Motor A speed)
// ENB = PB7 = M0PWM1  (Motor B speed)
static uint32_t g_pwmPeriod = 0;

void InitPWM(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    uint32_t pwmClock = SysCtlClockGet() / 64;
    g_pwmPeriod = pwmClock / PWM_FREQUENCY;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, g_pwmPeriod);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

// ── Hardware Init ─────────────────────────────────────────────
void InitHardware(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Sensor 1 (FRONT): PA3=TRIG, PA2=ECHO
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);

    // Sensor 2 (RIGHT): PD2=TRIG, PD3=ECHO
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3);

    // Motor A (LEFT WHEEL):  PB0=IN1, PB1=IN2
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Motor B (RIGHT WHEEL): PC4=IN3, PC5=IN4
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // LEDs: PF1=Red, PF3=Green
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3);

    InitPWM(); // ENA=PB6, ENB=PB7
}

// ── mv() ──────────────────────────────────────────────────────
// left/right: -255 (full reverse) to +255 (full forward)
void mv(int left, int right) {
    left  = constrainVal(left,  -PWM_MAX, PWM_MAX);
    right = constrainVal(right, -PWM_MAX, PWM_MAX);

    // Motor A — left wheel
    if (left == 0) {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    } else if (left > 0) {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, toPWMTicks(left, g_pwmPeriod));
    } else {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, toPWMTicks(-left, g_pwmPeriod));
    }

    // Motor B — right wheel
    if (right == 0) {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
    } else if (right > 0) {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, toPWMTicks(right, g_pwmPeriod));
    } else {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_5);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, toPWMTicks(-right, g_pwmPeriod));
    }
}

// ── Convenience wrappers ──────────────────────────────────────
void MotorStop(void)          { mv(0, 0); }

// ── Sensors ───────────────────────────────────────────────────
// Returns distance in cm. Returns 999 if no echo (open space).
uint32_t GetDistanceFront(void) {
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    SysCtlDelay(SysCtlClockGet() / 3000000 * 2);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    SysCtlDelay(SysCtlClockGet() / 3000000 * 10);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

    uint32_t timeout = 100000;
    while((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0) && --timeout);
    if (timeout == 0) return 999;

    uint32_t duration = 0;
    while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) != 0) {
        duration++;
        SysCtlDelay(2);
        if(duration > 30000) break;
    }
    return (duration * 0.0343f) / 2;
}

uint32_t GetDistanceRight(void) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
    SysCtlDelay(SysCtlClockGet() / 3000000 * 2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
    SysCtlDelay(SysCtlClockGet() / 3000000 * 10);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

    uint32_t timeout = 100000;
    while((GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3) == 0) && --timeout);
    if (timeout == 0) return 999;

    uint32_t duration = 0;
    while(GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_3) != 0) {
        duration++;
        SysCtlDelay(2);
        if(duration > 30000) break;
    }
    return (duration * 0.0343f) / 2;
}

// ── Main ──────────────────────────────────────────────────────
int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    InitHardware();
    int laststate = 0; //0: non yet, 1: moving right, 2: moving left, 3: maintaining distance


    while(1) {
        // 1. Read sensors
        uint32_t dFront = GetDistanceFront();
        uint32_t dRight = GetDistanceRight();

        // Update globals for debug watch
        distance  = dFront;
        distance2 = dRight;

        // 2. Right-hand rule maze solver
        if (dRight >= 55) {
            // --- GAP ON RIGHT: turn right to follow wall ---
            mv(255, 150);
            
            // --- DEAD END: spin left ---
            while(distance2 >= 55) {
                dFront = GetDistanceFront();
                dRight = GetDistanceRight();

                // Update globals for debug watch
                distance  = dFront;
                distance2 = dRight;
                
                mv(255, 150);
            }

            // LED: blue (both on = white, use red to signal turning)
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_1);
            laststate = 1;
        }
        else if (dFront <= 20 && dRight <= 25) {
            // --- WALL AHEAD: spin left using explicit HIGH/LOW ---
            
            // Left wheel REVERSE: IN1 (PB0) High, IN2 (PB1) Low
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, toPWMTicks(200, g_pwmPeriod));
            
            // Right wheel FORWARD: IN3 (PC4) High, IN4 (PC5) Low
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, toPWMTicks(200, g_pwmPeriod));

            // LED: red
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_1);
            laststate = 2;

            SysCtlDelay(300);

        }
        else if (dFront >= 15 && dRight < 30) {
            // --- FOLLOW RIGHT WALL: PD correction ---
            float error      = WALL_TARGET - (float)dRight;
            float dError     = error - prevError;           // derivative of error
            int correction;
            if (laststate == 3) {
              correction = (int)(error * WALL_KP + dError * WALL_KD);
            } else {
              correction = (int)(error * WALL_KP);
            }
            int   leftSpeed  = constrainVal(150 - correction, 0, PWM_MAX);
            int   rightSpeed = constrainVal(150 + correction, 0, PWM_MAX);
            mv(leftSpeed, rightSpeed);
            prevError = error;                              // store for next iteration            
            // LED: green
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_3);
            laststate = 3;
        }
        // else {
            // // --- DEAD END: spin left using explicit HIGH/LOW ---
            // while(distance < 50 && distance2 < 45) {
            //     dFront = GetDistanceFront();
            //     dRight = GetDistanceRight();

            //     // Update globals for debug watch
            //     distance  = dFront;
            //     distance2 = dRight;
                
            //     // Left wheel REVERSE: IN1 (PB0) High, IN2 (PB1) Low
            //     GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0);
            //     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, toPWMTicks(255, g_pwmPeriod));
                
            //     // Right wheel FORWARD: IN3 (PC4) High, IN4 (PC5) Low
            //     GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_4);
            //     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, toPWMTicks(255, g_pwmPeriod));

            //     SysCtlDelay((SysCtlClockGet() / (3 * 50)) + 100);
            // }
            // // LED: red
            // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_1);
        // }

        // 3. Small delay between pings to prevent echo collision (~20ms)
        SysCtlDelay(SysCtlClockGet() / (3 * 50));
    }
}