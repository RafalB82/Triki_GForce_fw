/**
 * trikig_diag.h - instrumentacja (plan VBT C1): liczniki dropow/okresu probkowania
 * + timingi CPU (DWT->CYCCNT @64MHz, 1us = 64 cyc).
 *
 * Liczniki aktualizowane ZAWSZE (koszt ~0, bss ~40B — przydaja sie tez jako
 * telemetria F5); wydruk RTT tylko przy TRIKIG_RTT_DIAG=1 (diag_print, ~1s).
 * Konteksty — kazdy licznik ma JEDNEGO producenta (RMW race-free bez sekcji
 * krytycznych): poll (SWI) = imu_samples/imu_dups/ring_drops/period/acq/dsp,
 * main = ble_drops/seq_gaps.
 */
#ifndef TRIKIG_DIAG_H
#define TRIKIG_DIAG_H

#include <stdint.h>
#include "nrf.h"
#include "trikig_board.h"

typedef struct {
    uint32_t imu_samples;    /* swieze probki IMU (bez duplikatow), kontekst poll */
    uint32_t imu_dups;       /* duplikaty (timer 9ms dogonal ODR 104Hz) */
    uint32_t ring_drops;     /* drop ramki BLE: pelny ring (K4: seq juz poszedl) */
    uint32_t seq_gaps;       /* dziury w seq widziane przy konsumpcji ringu (main) */
    uint32_t ble_drops;      /* ble_nus_data_send != NRF_SUCCESS (main) */
    uint16_t per_min_us;     /* okres miedzy swiezymi probkami [us] */
    uint16_t per_max_us;
    uint16_t per_avg_us;     /* EMA 1/32 */
    uint16_t acq_us_max;     /* I2C burst 12B — worst case */
    uint16_t dsp_us_max;     /* vbt_on_frame — worst case */
    uint16_t ble_us_max;     /* ble_nus_data_send — worst case */
} trikig_diag_t;

extern trikig_diag_t g_diag;

void diag_init(void);         /* DWT CYCCNT on (TRCENA + CYCCNTENA) */

static inline uint32_t diag_cyc(void) { return DWT->CYCCNT; }
static inline uint16_t diag_cyc_us(uint32_t cycles) { return (uint16_t)(cycles >> 6); }   /* 64MHz */

/* okres swiezej probki: min/max + EMA (init EMA pierwsza probka) */
void diag_period_us(uint16_t us);

static inline void diag_max16(volatile uint16_t *slot, uint16_t us)
{
    if (us > *slot) *slot = us;
}

#if TRIKIG_RTT_DIAG
void diag_print(void);        /* snapshot licznikow/timingow do RTT */
#endif

#endif /* TRIKIG_DIAG_H */
