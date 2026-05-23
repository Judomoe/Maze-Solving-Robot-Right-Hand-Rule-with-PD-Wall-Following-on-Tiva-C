# Maze-Solving Robot 🤖

An autonomous maze-solving robot built on the **Tiva C (TM4C123GH6PM)** 
microcontroller. It uses the **right-hand rule** algorithm with a **PD controller** 
for smooth wall-following, driven by real-time ultrasonic sensor feedback.

---

## How It Works

The robot operates as a 3-state machine:

| State | Condition | Action |
|-------|-----------|--------|
| Turn Right | dRight ≥ 55 cm (gap detected) | Steer right until wall found |
| Spin Left | dFront ≤ 20 cm AND dRight ≤ 25 cm | Rotate in place (dead end / corner) |
| Follow Wall | dFront ≥ 15 cm AND dRight < 30 cm | PD correction on motor speeds |

### PD Controller
error      = 12.5 cm − dRight
dError     = error − prevError
correction = (Kp × error) + (Kd × dError)
leftSpeed  = 150 − correction
rightSpeed = 150 + correction
- **Kp = 10** — proportional gain (reduces steady-state wall error)  
- **Kd = 20** — derivative gain (dampens oscillation)  
- **Target distance from right wall: 12.5 cm**

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Tiva C TM4C123GH6PM @ 40 MHz |
| Front sensor | HC-SR04 — TRIG: PA3, ECHO: PA2 |
| Right sensor | HC-SR04 — TRIG: PD2, ECHO: PD3 |
| Motor driver | L298N H-bridge |
| Left motor (A) | IN1: PB0, IN2: PB1, ENA: PB6 (PWM) |
| Right motor (B) | IN3: PC4, IN4: PC5, ENB: PB7 (PWM) |
| LEDs | Red: PF1 (turning/avoiding), Green: PF3 (following) |
| Power | 2x Li-ion in series (~7.4V) |

### ⚠️ Voltage Divider (Critical)
The HC-SR04 ECHO pin outputs **5V**, which would damage the Tiva C's **3.3V GPIO**.
A resistor voltage divider is placed on the ECHO line of each sensor to scale the 
signal down to a safe 3.3V before it reaches the microcontroller.

---

## PWM Configuration
- **Frequency:** 1 kHz
- **Pins:** PB6 = M0PWM0 (Motor A), PB7 = M0PWM1 (Motor B)
- **Clock divider:** SysCtlPWMClockSet(SYSCTL_PWMDIV_64)
- Speed range: 0 (stop) to 255 (full)

---

## Sensor Behavior
- Returns distance in **cm**
- Returns **999** if no echo detected (open space / timeout)
- Inter-ping delay of **~20 ms** between readings to prevent echo collision

---

## Project Structure
Maze-Robot/
├── main.c          # Full firmware (state machine, PD, sensors, PWM, motors)
└── README.md

## How to Build & Flash
1. Open in **Code Composer Studio (CCS)**
2. Link **TivaWare driverlib**
3. Build and flash to Tiva C via USB

## Future Improvements
- Add encoder feedback for more accurate motor speed control
- Support multiply connected mazes (Pledge algorithm)
- Add UART logging for real-time PD debug output
- Replace polling sensor reads with timer interrupts
- Add IR sensors for faster, noise-free distance measurement
