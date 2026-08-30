/**
 * trikig_batt.c - SAADC AIN2: pomiar Vbat CR2032 [mV] (patrz trikig_batt.h)
 *
 * Kontekst wywolania: batt_init() z main przy boocie; batt_sample_mv() z ticku
 * 1s (app_timer SWI — blokujace probkowanie ~0.5ms, dopuszczalne: poll I2C robi
 * ~300us w tym samym kontekscie co 9ms). SAADC v1 (nrfx v1 w SDK 17):
 * probkowanie blokowe sample_convert, event handler tylko dla kalibracji.
 *
 * Arytmetyka calkowitoliczbowa (bez FPU): sum(counts) * 3600mV * NUM /
 * (4095 * SAMPLES * DEN) — max ~118e6 miesci sie w int32.
 */
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_saadc.h"
#include "trikig_batt.h"
#include "trikig_diag.h"

#define BATT_FS_MV              3600L   /* 0.6V * gain 1/6 */
#define BATT_FS_COUNTS          4095L   /* 12-bit */
#define BATT_CAL_TIMEOUT_US     10000L  /* kalibracja offsetu < ~1ms; 10ms zapas */

static nrf_saadc_value_t s_sample;
static volatile bool     s_cal_done = false;
static bool              s_ready    = false;

static void batt_saadc_evt(nrf_drv_saadc_evt_t const *p_evt)
{
    if (p_evt->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE) {
        s_cal_done = true;
    }
}

void batt_init(void)
{
    nrf_drv_saadc_config_t cfg = {
        .resolution         = NRF_SAADC_RESOLUTION_12BIT,
        .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
        .interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY,
        .low_power_mode     = SAADC_CONFIG_LP_MODE,
    };
    if (nrf_drv_saadc_init(&cfg, batt_saadc_evt) != NRF_SUCCESS) return;

    nrf_saadc_channel_config_t cc = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_40US,   /* dzielnik wysokiej impedancji */
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0 + TRIKIG_BATT_AIN),
        .pin_n      = NRF_SAADC_INPUT_DISABLED,
    };
    if (nrf_drv_saadc_channel_init(0, &cc) != NRF_SUCCESS) return;

    s_cal_done = false;
    if (nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS) return;
    for (int32_t us = 0; us < BATT_CAL_TIMEOUT_US && !s_cal_done; us += 50) {
        nrf_delay_us(50);
    }
    s_ready = s_cal_done;
}

uint16_t batt_sample_mv(void)
{
    if (!s_ready) return 0;

    int32_t sum = 0;
    uint32_t faults = 0;
    for (uint32_t i = 0; i < TRIKIG_BATT_SAMPLES; i++) {
        /* audyt F/G 2026-08-30: fail SAADC nie jest cichy — licznik w diag; pojedyncza
         * nieudana konwersja nie kasuje pomiaru (sample ignorowana, reszta probek liczy). */
        if (nrf_drv_saadc_sample_convert(0, &s_sample) != NRF_SUCCESS) {
            faults++;
            continue;
        }
        if (s_sample > 0) sum += s_sample;      /* ujemne (offset) -> ignoruj */
    }
    if (faults != 0 && g_diag.saadc_faults < 0xFFFFFFFFu) g_diag.saadc_faults += faults;
    if (faults == TRIKIG_BATT_SAMPLES || sum == 0) {
        /* caly pomiar nieudany albo sum=0 (same probki <= 0): zwroc 0 = "brak pomiaru"
         * — main nie nadpisze starej wartosci. Guard sum==0 chroni takze przypadek
         * skalibrowanego OFFSET_MV != 0 (audyt G: inaczej OFFSET wygladalby jak odczyt). */
        return 0;
    }

    /* node_mV -> Vbat: skala + offset diody; clamp do uint16 */
    int32_t mv = (sum * BATT_FS_MV * (int32_t)TRIKIG_BATT_SCALE_NUM) /
                 (BATT_FS_COUNTS * (int32_t)(TRIKIG_BATT_SAMPLES - faults) * (int32_t)TRIKIG_BATT_SCALE_DEN);
    mv += TRIKIG_BATT_OFFSET_MV;
    if (mv < 0) mv = 0;
    if (mv > 0xFFFF) mv = 0xFFFF;
    return (uint16_t)mv;
}

bool batt_is_low(uint16_t mv)
{
    return mv != 0 && mv < TRIKIG_BATT_LOW_MV;
}
