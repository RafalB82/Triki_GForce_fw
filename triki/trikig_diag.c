/**
 * trikig_diag.c - instrumentacja (plan VBT C1), patrz trikig_diag.h.
 */
#include "trikig_diag.h"
#if TRIKIG_RTT_DIAG
#include "SEGGER_RTT.h"
#endif

trikig_diag_t g_diag;

void diag_init(void)
{
    /* TIMER1 @1MHz, 16-bit (nRF52810: brak DWT CYCCNT — patrz trikig_diag.h).
     * TIMER0 zajety przez SoftDevice; TIMER1 wolny (app_timer uzywa RTC1). */
    NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->PRESCALER = 4;                   /* 16MHz / 2^4 = 1MHz => 1 tick = 1us */
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->TASKS_START = 1;
}

void diag_period_us(uint16_t us)
{
    if (us == 0) return;                        /* pierwszy pomiar / wrap — pomin */
    if (g_diag.per_min_us == 0 || us < g_diag.per_min_us) g_diag.per_min_us = us;
    if (us > g_diag.per_max_us) g_diag.per_max_us = us;
    if (g_diag.per_avg_us == 0) {
        g_diag.per_avg_us = us;
    } else {
        int32_t d = (int32_t)us - (int32_t)g_diag.per_avg_us;
        g_diag.per_avg_us = (uint16_t)((int32_t)g_diag.per_avg_us + (d >> 5));
    }
}

#if TRIKIG_RTT_DIAG
void diag_print(void)
{
    SEGGER_RTT_printf(0, "DIAG drdy=%u smpl=%u dup=%u rdrop=%u gap=%u bdrop=%u | dtf=%u fb=%u twi=%u sadc=%u",
                      (unsigned)g_diag.drdy_mode, (unsigned)g_diag.imu_samples,
                      (unsigned)g_diag.imu_dups, (unsigned)g_diag.ring_drops,
                      (unsigned)g_diag.seq_gaps, (unsigned)g_diag.ble_drops,
                      (unsigned)g_diag.dt_faults, (unsigned)g_diag.drdy_fallbacks,
                      (unsigned)g_diag.twim_faults, (unsigned)g_diag.saadc_faults);
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_printf(0, "DIAG dt %u/%u/%u us | max acq=%u dsp=%u g=%u lin=%u v=%u ble=%u us",
                      (unsigned)g_diag.per_min_us, (unsigned)g_diag.per_avg_us,
                      (unsigned)g_diag.per_max_us,
                      (unsigned)g_diag.acq_us_max, (unsigned)g_diag.dsp_us_max,
                      (unsigned)g_diag.grav_us_max, (unsigned)g_diag.lin_us_max,
                      (unsigned)g_diag.vel_us_max, (unsigned)g_diag.ble_us_max);
    SEGGER_RTT_WriteString(0, "\r\n");
    SEGGER_RTT_printf(0, "DIAG idle=%u trans=%u cpfail=%u",
                      (unsigned)g_diag.idle_state, (unsigned)g_diag.idle_trans,
                      (unsigned)g_diag.idle_cp_fail);
    SEGGER_RTT_WriteString(0, "\r\n");
}
#endif
