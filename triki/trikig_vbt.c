/**
 * trikig_vbt.c - estymacja velocity (fixed-point q8.8, bez FPU — nRF52810)
 *
 * v0.0.24: detekcja bezruchu po |a| ~ g (norma wektora, niezalezna od orientacji).
 * Historia: v0.0.22/23 mialy stillness po |lin| osi X > th => pułapka: kapsel polozony
 * pod innym katem niz kalibracja = staly residual gravitacji => mv=1 na zawsze =>
 * bias sie nie doucza => rampa velocity do clampa (log 09:13: saturacja -15625 mm/s).
 * Zasada: w spoczynku |a| = g NIEZALEZNIE od orientacji (gravitacja to jedyny wektor).
 * Rest (|a|-g < 0.8 m/s^2 przez 8 ramek): bias MA-16 aktualizowany + ZUPT.
 * Move: integracja lin; ZUPT/bias wroca przy polozzeniu.
 */
#include "trikig_vbt.h"
#include <string.h>

static int32_t s_bias[3];       /* q8.8 m/s^2, gravitacja + offset (per os) */
static int32_t s_lpf[3];        /* q8.8 m/s^2, acc po lowpass */
static int32_t s_vel_q88[3];    /* q8.8 m/s */
static uint8_t s_bias_n;
static uint8_t s_rest_cnt;
static bool    s_rest;
static uint16_t s_frames;       /* licznik ramek od resetu (fallback kalibracji biasu) */

/* raw i16 (1g=2048) -> q8.8 m/s^2: raw/2048*9.80665*256 = raw*1.22583 => raw*314>>8 (err 0.06%) */
static inline int32_t acc_q88(int16_t r)
{
    return ((int32_t)r * 314) >> 8;
}

/* integer sqrt (bit-by-bit, bez FPU; 111 wywolan/s — pomijalne) */
static int32_t isqrt32(uint32_t n)
{
    uint32_t res = 0, bit = 1u << 30;
    while (bit > n) bit >>= 2;
    while (bit) {
        if (n >= res + bit) { n -= res + bit; res = (res >> 1) + bit; }
        else res >>= 1;
        bit >>= 2;
    }
    return (int32_t)res;
}

void vbt_reset(void)
{
    memset(s_bias, 0, sizeof(s_bias));
    memset(s_lpf, 0, sizeof(s_lpf));
    memset(s_vel_q88, 0, sizeof(s_vel_q88));
    s_bias_n = 0;
    s_rest_cnt = 0;
    s_rest = false;
    s_frames = 0;
}

static void bias_update(const int32_t a[3])
{
    if (s_bias_n < TRIKIG_VBT_BIAS_MA_N) {
        s_bias_n++;
        for (int i = 0; i < 3; i++)
            s_bias[i] += (a[i] - s_bias[i]) / (int32_t)s_bias_n;
    } else {
        for (int i = 0; i < 3; i++)
            s_bias[i] += (a[i] - s_bias[i]) / (int32_t)TRIKIG_VBT_BIAS_MA_N;
    }
}

void vbt_on_frame(const uint8_t *raw12)
{
    int16_t a[3];
    /* payload: [0..5]=gyro (pomijany w v1), [6..11]=acc i16 LE */
    memcpy(&a[0], &raw12[6], 2);
    memcpy(&a[1], &raw12[8], 2);
    memcpy(&a[2], &raw12[10], 2);

    if (s_frames < 0xFFFF) s_frames++;

    int32_t av[3] = { acc_q88(a[0]), acc_q88(a[1]), acc_q88(a[2]) };

    for (int i = 0; i < 3; i++)
        s_lpf[i] += (av[i] - s_lpf[i]) >> TRIKIG_VBT_ACC_LPF_ALPHA;

    /* stillness: ||a| - g| < th (norma, nie os!) — w spoczynku |a|=g w kazdej orientacji */
    int64_t p = (int64_t)s_lpf[0]*s_lpf[0] + (int64_t)s_lpf[1]*s_lpf[1] + (int64_t)s_lpf[2]*s_lpf[2];
    if (p > (int64_t)UINT32_MAX) p = (int64_t)UINT32_MAX;   /* audyt 2026-08-30: 3 osie @ ~13g moga przepelnic uint32 (wrap = falszywy rest) */
    int32_t norm = isqrt32((uint32_t)p);
    int32_t dev = norm - (int32_t)TRIKIG_VBT_G_REF_Q88;
    if (dev < 0) dev = -dev;

    if (dev < (int32_t)TRIKIG_VBT_REST_TH) {
        if (s_rest_cnt < 255) s_rest_cnt++;
    } else {
        s_rest_cnt = 0;
    }
    s_rest = (s_rest_cnt >= TRIKIG_VBT_REST_FRAMES);

    if (s_rest) {
        /* lezy: bias doucza sie aktualnej orientacji, velocity gaśnie */
        bias_update(s_lpf);
        for (int i = 0; i < 3; i++)
            s_vel_q88[i] -= s_vel_q88[i] >> TRIKIG_VBT_BETA_LPF;
    } else if (s_bias_n < TRIKIG_VBT_BIAS_MA_N && s_frames < TRIKIG_VBT_BIAS_FORCE_FRAMES) {
        /* boot: integracja wstrzymana do pierwszej pelnej kalibracji biasu (usuwa transient ~-2000 mm/s) */
    } else {
        if (s_bias_n < TRIKIG_VBT_BIAS_MA_N) {
            /* audyt#5 2026-08-30: ruch od startu bez ani jednej klatki spoczynku — wymus bias
             * z aktualnego LPF, zeby velocity w ogole wystartowal. Dokladnosc gorsza do czasu
             * pierwszego prawdziwego bezruchu (potem bias doucza sie MA-16 jak zwykle). */
            for (int i = 0; i < 3; i++)
                s_bias[i] = s_lpf[i];
            s_bias_n = TRIKIG_VBT_BIAS_MA_N;
        }
        /* v += a*dt; dt=9.6ms q16.16 = 629/2^16 */
        for (int i = 0; i < 3; i++) {
            int32_t lin = s_lpf[i] - s_bias[i];
            s_vel_q88[i] += (lin * 629) >> 16;
            if (s_vel_q88[i] >  TRIKIG_VBT_V_CLAMP) s_vel_q88[i] =  TRIKIG_VBT_V_CLAMP;
            if (s_vel_q88[i] < -TRIKIG_VBT_V_CLAMP) s_vel_q88[i] = -TRIKIG_VBT_V_CLAMP;
        }
    }
}

int32_t vbt_velocity_mms(void)
{
    /* os X barbell; q8.8 -> mm/s */
    return (s_vel_q88[0] * 1000) >> 8;
}

bool vbt_moving(void)
{
    return !s_rest;
}

uint8_t vbt_flags(void)
{
    uint8_t f = 0;
    if (!s_rest) f |= 0x01;   /* moving */
    if (s_rest)  f |= 0x02;   /* rest */
    if (s_bias_n >= TRIKIG_VBT_BIAS_MA_N) f |= 0x04;   /* bias calibrated */
    return f;
}
