/**
 * trikig_lsm6dsl.h - LSM6DSL driver (polling OUT, FIFO OFF)
 *
 * D-016: FIFO nie uzywane (3 poprawne konfiguracje nie dawaly count>0 na tym
 * sprzecie - powrot dopiero z LA + AN4650). Dane: poll OUT 0x22 przez app_timer.
 * D-017: wartosci rejestrow WYLACZNIE z tabeli datasheet (kontrola readback w RTT).
 */
#ifndef TRIKIG_LSM6DSL_H
#define TRIKIG_LSM6DSL_H

#include <stdint.h>
#include <stdbool.h>

#define LSM_ADDR          0x6A                 /* 7-bit (TWIM nrfx) */
#define LSM_ADDR_W        ((0x6A << 1) | 0u)   /* SA0=VDD -> 0x6A */
#define LSM_ADDR_R        ((0x6A << 1) | 1u)

/* --- rejestry (adresy wg datasheet) --- */
#define LSM_WHO_AM_I      0x0F
#define LSM_CTRL1_XL      0x10
#define LSM_CTRL2_G       0x11
#define LSM_CTRL3_C       0x12
#define LSM_OUTX_L_G      0x22   /* burst 12B: gyro(6)+acc(6), IF_INC */
#define LSM_INT1_CTRL     0x0D   /* DRDY_XL = bit0 */
#define LSM_INT2_CTRL     0x0E   /* DRDY_XL = bit0 — **DRDY ukladu wychodzi na INT2** (pin 9
                                  * ukladu wg datasheet ukladu; INT1 puste) — 0.3.5 */

/* --- v0.4.0 IDLE-CONNECTED: activity/inactivity (DS6207 sec 6.5.2, bitfield wg
 * oficjalnego sterownika ST lsm6dsl_reg.h) ---
 * TAP_CFG (0x58): bit7 INTERRUPTS_ENABLE, bity [6:5] INACT_EN (01=activity,
 * 10=inactivity, 11=both), bit4 SLOPE_FDS, bity [3:1] TAP_x_EN, bit0 LIR.
 * INACT_EN=11 => po SLEEP_DUR bezruchu HW sam przechodzi w low-power (acc ODR
 * 12.5Hz, gyro power-down); activity => auto-restore konfiguracji CTRL1/CTRL2.
 * WAKE_UP_THS (0x5B): WK_THS[5:0] — prog amplitudy (1 LSB = FS_XL/2^6; przy
 * FS=16g => 250 mg/LSB — do potwierdzenia/postrojenia na HW, D-017).
 * WAKE_UP_DUR (0x5C): SLEEP_DUR[3:0] — czas bezruchu [s] (max 15).
 * MD1_CFG (0x5E): bit7 INT1_SLEEP_CHANGE — routing zmiany stanu na INT1 (P0.09). */
#define LSM_TAP_CFG       0x58
#define LSM_WAKE_UP_THS   0x5B
#define LSM_WAKE_UP_DUR   0x5C
#define LSM_MD1_CFG       0x5E
#define LSM_WAKE_UP_SRC   0x1B   /* bit4 SLEEP_STATE_IA (readback stanu, czysci LIR) */

#define LSM_TAP_CFG_INACT 0xE1u  /* INTERRUPTS_ENABLE | INACT_EN=11 | LIR */
#define LSM_WK_THS        1u    /* 250 mg @ FS 16g — start, strojenie na HW */
#define LSM_SLEEP_DUR_S   4u    /* 4s bezruchu => inactivity (propozycja 3-5s) */
#define LSM_MD1_SLEEP_CHG 0x80u /* bit7 INT1_SLEEP_CHANGE */

/* --- wartosci CTRL1_XL / CTRL2_G (datasheet, NIE z pamieci!) ---
 * ODR bity [7:4]: 0100 = 104 Hz
 * FS_XL bity [3:2] NIEMONOTONICZNA: 00=2g 01=16g 10=4g 11=8g -> 16g = 01
 * FS_G  bity [3:2] monotoniczna:    00=245 01=500 10=1000 11=2000 dps -> 2000 = 11
 * v19: CTRL1_XL=0x44 (104Hz+16g, 2048 LSB/g), CTRL2_G=0x4C (104Hz+2000dps, 16.4 LSB/dps)
 */
#define LSM_CTRL1_XL_V19  0x44u
#define LSM_CTRL2_G_V19   0x4Cu
/* BDU(bit3) + IF_INC(bit2) = 0x0C (audyt 2026-08-30: wczesniejsze 0x44 ustawialo H_LACTIVE
 * zamiast BDU — brak BDU = mozliwe rozdarte odczyty 12B przy zderzeniu burst z ODR). */
#define LSM_CTRL3_C_CFG   0x0Cu

#define LSM_WHO_AM_I_VAL  0x6Au

#define LSM_RESET_MAX_ITERS 500u  /* 500*100us = 50ms timeout na SW reset (K7) */

bool    lsm6dsl_init(void);                    /* WHO_AM_I + SW reset + config, z readbackiem RTT */
bool    lsm6dsl_read_motion(uint8_t *dst12);   /* burst gyro(6)+acc(6) i16LE (TWIM, fallback bb) */
bool    lsm6dsl_drdy_enable(bool on);          /* INT2_CTRL = DRDY_XL, z readbackiem */
bool    lsm6dsl_drdy1_enable(bool on);         /* INT1_CTRL = DRDY_XL, z readbackiem (scan) */
bool    lsm6dsl_inactivity_enable(bool on);    /* v0.4.0: TAP_CFG INACT_EN + progi + MD1_CFG, z readbackiem */
bool    lsm6dsl_inact_state(bool *sleeping);    /* v0.4.0: WAKE_UP_SRC.SLEEP_STATE_IA (sync FW<->HW) */

#endif /* TRIKIG_LSM6DSL_H */
