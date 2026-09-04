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
    uint32_t imu_samples;    /* swieze probki IMU (bez duplikatow) */
    uint32_t imu_dups;       /* duplikaty — TYLKO diagnostyka (memcmp nie steruje VBT) */
    uint32_t ring_drops;     /* drop ramki BLE: pelny ring (K4: seq juz poszedl) */
    uint32_t seq_gaps;       /* dziury w seq widziane przy konsumpcji ringu (main) */
    uint32_t ble_drops;      /* ble_nus_data_send != NRF_SUCCESS (main) */
    uint32_t dt_faults;      /* dt poza clampem lub gap > 60ms (C7) */
    uint32_t drdy_fallbacks; /* odczyty watchdogiem poll-timera zamiast DRDY (C7) */
    uint32_t twim_faults;    /* bledy TWIM -> fallback bit-bang (C8) */
    uint32_t saadc_faults;   /* nieudane konwersje SAADC baterii (audyt F/G; producent:
                              * sleep timer 1s + rzadki fallback z main — dual-producer
                              * akceptowany dla telemetrii, race moze zgubic ++ zdarza) */
    uint16_t per_min_us;     /* dt = t[n]-t[n-1] z timestampow DRDY [us] */
    uint16_t per_max_us;
    uint16_t per_avg_us;     /* EMA 1/32 */
    uint16_t acq_us_max;     /* I2C burst 12B (worst case) */
    uint16_t dsp_us_max;     /* vbt_on_frame calkowity (worst case) */
    uint16_t ble_us_max;     /* ble_nus_data_send (worst case) */
    uint16_t grav_us_max;    /* profil DSP (C6 planu): propagacja+korekcja+renorm */
    uint16_t lin_us_max;     /* LPF roznicy + detektor rest + projekcja */
    uint16_t vel_us_max;     /* integracja + ZUPT + clamp */
    uint8_t  drdy_mode;      /* 0 = polling; 1/2 = P0.09 rising/falling; 3/4 = P0.10 r/f */
    uint8_t  idle_state;     /* v0.4.0: 1 = IDLE-CONNECTED (acc 12.5Hz LP, gyro PD) */
    uint8_t  train_mode;     /* v0.5.0: 1 = training mode (RX 20 18 01 — IDLE wylaczony na czas serii) */
    uint16_t idle_trans;     /* v0.4.0: liczba przejsc idle<->active */
    uint16_t idle_cp_fail;   /* v0.4.0: odrzucone/znokautowane conn param update (IDLE) */
} trikig_diag_t;

extern trikig_diag_t g_diag;

/* Timebase: TIMER1 @1MHz (16-bit, wrap 65.5ms — wszystkie mierzone okna < 65.5ms;
 * wrap-safe przez arytmetyke uint16: (uint16_t)(now - t0)).
 * UWAGA (log RTT 0.3.3): DWT->CYCCNT NIE ISTNIEJE na nRF52810 (obciety M4) — wszystkie
 * pomiary dawaly 0. Nie uzywac DWT do pomiarow na 52810/52811. */
void diag_init(void);         /* TIMER1 start @1MHz */

static inline uint32_t diag_cyc(void)                 /* [us], capture -> CC[0] */
{
    NRF_TIMER1->TASKS_CAPTURE[0] = 1;
    return NRF_TIMER1->CC[0];
}
static inline uint16_t diag_cyc_us(uint32_t diff_cyc) { return (uint16_t)diff_cyc; }

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
