/**
 * trikig_board.h - LED / BTN / sleep / RTT diag (board-level, niefunkcyjne)
 */
#ifndef TRIKIG_BOARD_H
#define TRIKIG_BOARD_H

#include <stdint.h>
#include <stdbool.h>

#define PIN_LED            28u
#define PIN_BTN            25u
#define PIN_IMU_INT1       9u   /* LSM INT1 / SLEEP_CHANGE (od 0.4.0: activity/inactivity -> IDLE) */
#define PIN_IMU_INT2      10u   /* **LSM INT2/DRDY -> P0.10** [P] potwierdzone plyta 2026-08-30 */
#define LED_ACTIVE_LOW     1
#define BTN_ACTIVE_LOW     1

#define SLEEP_TIMEOUT_S    300u
#define SLEEP_TIMER_MS     1000u                       /* tick 1s */
#define SLEEP_ITERS        (SLEEP_TIMEOUT_S)          /* 1 tick = 1s -> iteracje = sekundy */

/* RTT diag: tylko z TRIKIG_RTT_DIAG=1 (produkcja = cisza; FW tag i readback
 * rejestrow sa w tym samym kanale i wylaczaja sie razem z flaga). */
#if TRIKIG_RTT_DIAG
#include "SEGGER_RTT.h"
#define rtt_diag_printf(...)  do { SEGGER_RTT_printf(0, __VA_ARGS__); SEGGER_RTT_WriteString(0, "\\r\\n"); } while (0)
#else
#define rtt_diag_printf(...)  do { } while (0)
#endif

void led_write(bool on);
void led_blink(uint8_t n, uint16_t on_ms, uint16_t off_ms);
bool btn_pressed(void);
void enter_system_off(void);

#endif /* TRIKIG_BOARD_H */
