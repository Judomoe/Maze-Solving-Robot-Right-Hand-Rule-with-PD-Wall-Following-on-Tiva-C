/**
 * uart_comm.c — UART0 packet parser (USB connection to Raspberry Pi)
 *
 * *** CHANGED FROM UART1 → UART0 ***
 * Reason: USB cable on the TM4C123G LaunchPad connects through the
 * onboard ICDI chip to UART0 (PA0/PA1), NOT to UART1 (PB0/PB1).
 * Using USB for the RPi link requires UART0.
 *
 * Hardware:
 *   UART0 RX → PA0  (connected via USB through ICDI)
 *   UART0 TX → PA1  (connected via USB through ICDI)
 *   RPi sees this as /dev/ttyACM0
 *
 * Packet format received from RPi:
 *   $H<pan>V<tilt>\n    e.g.  $H090V075\n
 *
 * Parser state machine:
 *   WAIT_DOLLAR → WAIT_H → READ_PAN(0..2) → WAIT_V
 *              → READ_TILT(0..2) → WAIT_NEWLINE → (emit) → WAIT_DOLLAR
 *
 * Any unexpected byte resets to WAIT_DOLLAR (frame re-sync).
 * Angle digits are validated: parsed value must be in [0, 180].
 * Invalid packets are silently dropped.
 */

#include "uart_comm.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

/* ── Parser state ────────────────────────────────────────────────────────── */

typedef enum {
    STATE_WAIT_DOLLAR = 0,
    STATE_WAIT_H,
    STATE_PAN_D0,
    STATE_PAN_D1,
    STATE_PAN_D2,
    STATE_WAIT_V,
    STATE_TILT_D0,
    STATE_TILT_D1,
    STATE_TILT_D2,
    STATE_WAIT_NEWLINE
} ParserState_t;

static ParserState_t  s_state    = STATE_WAIT_DOLLAR;
static uint32_t       s_pan_acc  = 0;
static uint32_t       s_tilt_acc = 0;

static uint32_t       s_pan_ready   = 90;
static uint32_t       s_tilt_ready  = 90;
static bool           s_packet_ready = false;

/* ── Internal: feed one byte through the state machine ───────────────────── */

static void parser_feed(uint8_t byte)
{
    switch (s_state) {

    case STATE_WAIT_DOLLAR:
        if (byte == '$') { s_state = STATE_WAIT_H; }
        break;

    case STATE_WAIT_H:
        if (byte == 'H') { s_pan_acc = 0; s_state = STATE_PAN_D0; }
        else              { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_PAN_D0:
        if (byte >= '0' && byte <= '9') {
            s_pan_acc = (byte - '0');
            s_state   = STATE_PAN_D1;
        } else { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_PAN_D1:
        if (byte >= '0' && byte <= '9') {
            s_pan_acc = s_pan_acc * 10 + (byte - '0');
            s_state   = STATE_PAN_D2;
        } else { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_PAN_D2:
        if (byte >= '0' && byte <= '9') {
            s_pan_acc = s_pan_acc * 10 + (byte - '0');
            s_state   = STATE_WAIT_V;
        } else { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_WAIT_V:
        if (byte == 'V') { s_tilt_acc = 0; s_state = STATE_TILT_D0; }
        else              { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_TILT_D0:
        if (byte >= '0' && byte <= '9') {
            s_tilt_acc = (byte - '0');
            s_state    = STATE_TILT_D1;
        } else { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_TILT_D1:
        if (byte >= '0' && byte <= '9') {
            s_tilt_acc = s_tilt_acc * 10 + (byte - '0');
            s_state    = STATE_TILT_D2;
        } else { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_TILT_D2:
        if (byte >= '0' && byte <= '9') {
            s_tilt_acc = s_tilt_acc * 10 + (byte - '0');
            s_state    = STATE_WAIT_NEWLINE;
        } else { s_state = STATE_WAIT_DOLLAR; }
        break;

    case STATE_WAIT_NEWLINE:
        if (byte == '\n') {
            if (s_pan_acc <= 180 && s_tilt_acc <= 180) {
                s_pan_ready    = s_pan_acc;
                s_tilt_ready   = s_tilt_acc;
                s_packet_ready = true;
            }
        }
        s_state = STATE_WAIT_DOLLAR;
        break;

    default:
        s_state = STATE_WAIT_DOLLAR;
        break;
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void UART_Comm_Init(uint32_t baud)
{
    /* ── CHANGED: UART1/GPIOB → UART0/GPIOA ── */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);   /* was UART1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);   /* was GPIOB */
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}

    GPIOPinConfigure(GPIO_PA0_U0RX);               /* was GPIO_PB0_U1RX */
    GPIOPinConfigure(GPIO_PA1_U0TX);               /* was GPIO_PB1_U1TX */
    GPIOPinTypeUART(GPIO_PORTA_BASE,               /* was PORTB */
                    GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE,                /* was UART1_BASE */
                        SysCtlClockGet(),
                        baud,
                        (UART_CONFIG_WLEN_8 |
                         UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    UARTEnable(UART0_BASE);                        /* was UART1_BASE */

    s_state        = STATE_WAIT_DOLLAR;
    s_packet_ready = false;
}

bool UART_Comm_Poll(uint32_t *pan_out, uint32_t *tilt_out)
{
    while (UARTCharsAvail(UART0_BASE)) {            /* was UART1_BASE */
        int32_t c = UARTCharGetNonBlocking(UART0_BASE);
        if (c != -1) {
            parser_feed((uint8_t)c);
        }
    }

    if (s_packet_ready) {
        *pan_out       = s_pan_ready;
        *tilt_out      = s_tilt_ready;
        s_packet_ready = false;
        return true;
    }
    return false;
}

void UART_Comm_SendStr(const char *s)
{
    while (*s) {
        UARTCharPut(UART0_BASE, (uint8_t)*s);      /* was UART1_BASE */
        s++;
    }
}