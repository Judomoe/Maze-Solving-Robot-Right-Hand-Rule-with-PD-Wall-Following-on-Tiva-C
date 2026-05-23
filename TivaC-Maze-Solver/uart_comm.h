/**
 * uart_comm.h — UART packet parser for servo angle commands
 * ===========================================================
 * UART1 on PB0 (RX) / PB1 (TX) is used for the Raspberry Pi link.
 * UART0 (PA0/PA1 via USB) is left free for debug prints.
 *
 * Packet format (ASCII, 10 bytes):
 *   $H<pan>V<tilt>\n
 *
 *   Byte[0]   : '$'            — start-of-frame marker
 *   Byte[1]   : 'H'            — pan field identifier
 *   Byte[2-4] : '0'–'9'  ×3   — pan  angle, zero-padded, 000–180
 *   Byte[5]   : 'V'            — tilt field identifier
 *   Byte[6-8] : '0'–'9'  ×3   — tilt angle, zero-padded, 000–180
 *   Byte[9]   : '\n'           — end-of-frame marker
 *
 *   Example:  $H090V075\n
 *             $H000V150\n
 *
 * The parser runs as a simple non-blocking state machine driven by
 * UART_Comm_Poll() called from the main loop. When a complete, valid
 * packet is decoded UART_Comm_Poll() returns true and writes the angles
 * into the output parameters.
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialise UART1 at the given baud rate.
 * Must be called after SysCtlClockSet().
 * @param baud  Desired baud rate (e.g. 115200)
 */
void UART_Comm_Init(uint32_t baud);

/**
 * Non-blocking poll — call this every main-loop iteration.
 * Drains all available bytes from the UART RX FIFO through the parser.
 *
 * @param pan_out   Written with the decoded pan  angle when a packet arrives.
 * @param tilt_out  Written with the decoded tilt angle when a packet arrives.
 * @return true if a complete, valid packet was decoded this call; false otherwise.
 *
 * Note: if multiple packets arrive between calls, only the most recent
 * complete packet is returned (stale frames are dropped).
 */
bool UART_Comm_Poll(uint32_t *pan_out, uint32_t *tilt_out);

/** Send a NUL-terminated string out of UART1 (for optional ACK/debug). */
void UART_Comm_SendStr(const char *s);

#endif /* UART_COMM_H */
