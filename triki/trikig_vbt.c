/**
 * trikig_vbt.c - VBT: gravity tracking ACC+GYRO + movement-axis velocity
 *                (plan VBT C2-C5, v0.3.0, fixed-point q8.8/q16.16, bez FPU — nRF52810)
 *
 * Historia: v0.0.22/23 stillness po |lin| osi X => pulapka orientacji. v0.0.24: rest
 * po normie + bias per os MA-16 — poprawne TYLKO w bezruchu: bias = zamrozony wektor
 * grawitacji, po obrocie sensora do integracji trafia pozorne a (plan VBT-01).
 *
 * v0.3.0 (C2-C5): bias per os zastapiony estymatorem grawitacji (filtr komplementarny):
 *   1. propagacja gyro:   dg = -(w x g)*dt co ramke (wektor staly w swiecie rotuje
 *      w ukladzie ciala przeciwnie do obrotu ciala), z ZAOKRAGLIENIEM (truncation >>32
 *      bil 4%/ramke => kumulatywny lag ~2 deg przy 90 deg obrotu — zlapano w C6)
 *   2. korekcja ACC:      g += (LPF(acc) - g) >> 5 — tylko gdy rest I |w| < 2 dps
 *      (quasi-statyka: kasuje gyro-bias; przy rotacji propagacja jest dokladna, a
 *      korekcja ciagnela g ku opoznionemu LPF(acc) => pozorny lin — zlapano w C6)
 *   3. renormalizacja do g_ref co ramke (isqrt)
 *   4. lin = LPF(acc_raw - g_est) — JEDNO LPF na ROZNICY (struktura z C6: rozne lagi
 *      dwoch LPF-ow — acc vs gest — nie kasowaly znieksztalcenia; LPF na roznicy
 *      przenosi caly lag grawitacji na strone estymaty)
 *   5. detektor rest: ||lin|| < 0.3 m/s^2 przez 8 ramek — 1. rzedu wzgledem a_perp
 *      (stara norma ||a|-g| byla 2. rzedu: dev ~ a^2/2g, slepa na wolne pushy — C6)
 *   6. a_move = dot(lin, axis), v += a_move*dt (dt = 1/ODR q16.16; C7 DRDY podmieni)
 * Zasady zachowane: boot-hold do pierwszej kalibracji g + fallback BIAS_FORCE_FRAMES
 * (audyt 0.1.1), ZUPT w bezruchu z min-krokiem (decay stenal przy |v|<250 mm/s — C6).
 */
#include "trikig_vbt.h"
#include <stdlib.h>
#include <string.h>

/* Profil DSP (plan VBT, P2): sekcje mierzone DWT do g_diag — tylko build deweloperski
 * (TRIKIG_VBT_PROFILE=1 z Makefile przy TRIKIG_RTT_DIAG=1); host/harness bez tego.
 * Variadic (__VA_ARGS__): przecinki w kodzie sekcji rozwalylyby argumenty zwyklego makra. */
#if TRIKIG_VBT_PROFILE
#include "trikig_diag.h"
#define VBT_PROF(section, ...) \
    do { uint32_t t0_ = diag_cyc(); { __VA_ARGS__ } diag_max16(&g_diag.section, diag_cyc_us(diag_cyc() - t0_)); } while (0)
#else
#define VBT_PROF(section, ...) \
    do { { __VA_ARGS__ } } while (0)
#endif

static int32_t s_gest[3];       /* q8.8 m/s^2, estymata wektora grawitacji (body frame) */
static int32_t s_lpf[3];        /* q8.8 m/s^2, LPF(acc) — cel korekcji + boot-snap */
static int32_t s_lin[3];        /* q8.8 m/s^2, LPF(acc - g_est) — linear acc */
static int32_t s_vel_q88;       /* q8.8 m/s, skalar po projekcji na os ruchu */
static int32_t s_axis[3];       /* q12, os ruchu (default X — gryf, kapsel na gryfie) */
static uint8_t s_rest_cnt;
static bool    s_rest;
static bool    s_gest_ok;       /* false do pierwszej kalibracji g (boot-hold / fallback) */
static bool    s_gest_forced;   /* kalibracja g z WYMUSZENIA (fallback 5s w ruchu) — host
                                 * powinien oznaczyc poczatkowe serie jako niepewne (audyt) */
static int32_t s_wbias[3];      /* q16.16 rad/s, estymata biasu gyro (nauka w quasi-bezruchu,
                                 * 0.3.7: dryf z biasu podtrzymywal blad g na progu gate'a
                                 * => raczeta velocity przy ruchach — log wire v2 21:42) */
static uint16_t s_frames;       /* licznik ramek od resetu (fallback kalibracji) */

/* raw i16 (1g=2048) -> q8.8 m/s^2: raw/2048*9.80665*256 = raw*1.22583 => raw*314>>8 (err 0.06%) */
static inline int32_t acc_q88(int16_t r)
{
    return ((int32_t)r * 314) >> 8;
}

/* integer sqrt (bit-by-bit, bez FPU; ~222 wywolan/s — pomijalne, DSP-01 po profilu) */
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
    memset(s_gest, 0, sizeof(s_gest));
    memset(s_lpf, 0, sizeof(s_lpf));
    memset(s_lin, 0, sizeof(s_lin));
    memset(s_axis, 0, sizeof(s_axis));
    s_axis[0] = (int32_t)TRIKIG_VBT_AXIS_Q12;   /* os X — barbell (kontrakt 0.0.x) */
    s_vel_q88 = 0;
    s_rest_cnt = 0;
    s_rest = false;
    s_gest_ok = false;
    s_gest_forced = false;
    memset(s_wbias, 0, sizeof(s_wbias));
    s_frames = 0;
}

void vbt_set_axis(const int16_t axis_q12[3])
{
    /* TODO (audyt E 2026-08-30): funkcja nie podlaczona do komendy RX — przy dodaniu
     * ustawiania osi z hosta DODAC walidacje zakresu axis_q12 (obecnie: wektor zerowy
     * odrzucany, reszta normalizowana; max int16*4096 miesci sie w int32 — OK). */
    int64_t p = (int64_t)axis_q12[0]*axis_q12[0] +
                (int64_t)axis_q12[1]*axis_q12[1] +
                (int64_t)axis_q12[2]*axis_q12[2];
    int32_t n = isqrt32(p > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)p);
    if (n == 0) return;                         /* odrzuc wektor zerowy — zachowaj stara os */
    for (int i = 0; i < 3; i++)
        s_axis[i] = ((int32_t)axis_q12[i] * (int32_t)TRIKIG_VBT_AXIS_Q12) / n;
}

void vbt_reset_velocity(void)
{
    s_vel_q88 = 0;                              /* twardy ZUPT (C7: gap dt / rekalibracja) */
}

void vbt_on_frame(const uint8_t *raw12, uint16_t dt_q16)
{
    int16_t g_raw[3], a[3];
    /* payload: [0..5]=gyro i16LE, [6..11]=acc i16LE */
    memcpy(&g_raw[0], &raw12[0], 2);
    memcpy(&g_raw[1], &raw12[2], 2);
    memcpy(&g_raw[2], &raw12[4], 2);
    memcpy(&a[0], &raw12[6], 2);
    memcpy(&a[1], &raw12[8], 2);
    memcpy(&a[2], &raw12[10], 2);

    if (s_frames < 0xFFFF) s_frames++;

    int32_t av[3] = { acc_q88(a[0]), acc_q88(a[1]), acc_q88(a[2]) };

    for (int i = 0; i < 3; i++)
        s_lpf[i] += (av[i] - s_lpf[i]) >> TRIKIG_VBT_ACC_LPF_ALPHA;

    if (!s_gest_ok) {
        /* boot: g z pierwszego bezruchu (usuwa transient LPF z zimnego startu); fallback:
         * ruch od startu bez ani jednej klatki spoczynku — wymus z LPF po ~5s (audyt#5
         * 2026-08-30). Boot-detekcja po NORMIE LPF; po snapie detektor przechodzi na lin. */
        int64_t p = (int64_t)s_lpf[0]*s_lpf[0] + (int64_t)s_lpf[1]*s_lpf[1] + (int64_t)s_lpf[2]*s_lpf[2];
        if (p > (int64_t)UINT32_MAX) p = (int64_t)UINT32_MAX;
        int32_t norm = isqrt32((uint32_t)p);
        int32_t dev = norm - (int32_t)TRIKIG_VBT_G_REF_Q88;
        if (dev < 0) dev = -dev;
        if (dev < (int32_t)TRIKIG_VBT_REST_TH || s_frames >= TRIKIG_VBT_BIAS_FORCE_FRAMES) {
            memcpy(s_gest, s_lpf, sizeof(s_gest));
            s_gest_ok = true;
            s_gest_forced = (s_frames >= TRIKIG_VBT_BIAS_FORCE_FRAMES);   /* audyt: host widzi */
            s_rest = true;                      /* pierwszy po-snapie frame z korekcja */
        }
        return;                                 /* hold: integracja wstrzymana do kalibracji g */
    }

    /* 1. propagacja gyro: dg = -(w x g)*dt. w q16.16 [rad/s], g q8.8 => iloczyn q24 (int64,
     *    przepelnia int32 przy FS 2000dps); *dt q16.16 >> 32 => q8.8, z zaokragleniem
     *    (+2^31): truncation bil ~4% katowe na ramke => kumulatywny lag (C6). */
    int32_t wq[3] = { (int32_t)g_raw[0] * TRIKIG_VBT_GYR_Q16_RAD_LSB,
                      (int32_t)g_raw[1] * TRIKIG_VBT_GYR_Q16_RAD_LSB,
                      (int32_t)g_raw[2] * TRIKIG_VBT_GYR_Q16_RAD_LSB };
    /* propagacja na gyro SKORYGOWANYM o nauczony bias (0.3.7): raw - wbias */
    int32_t we[3] = { wq[0] - s_wbias[0], wq[1] - s_wbias[1], wq[2] - s_wbias[2] };
    VBT_PROF(grav_us_max,
    {
        int64_t cx = (int64_t)we[1]*s_gest[2] - (int64_t)we[2]*s_gest[1];
        int64_t cy = (int64_t)we[2]*s_gest[0] - (int64_t)we[0]*s_gest[2];
        int64_t cz = (int64_t)we[0]*s_gest[1] - (int64_t)we[1]*s_gest[0];
        s_gest[0] -= (int32_t)((cx * (int64_t)dt_q16 + ((int64_t)1 << 31)) >> 32);
        s_gest[1] -= (int32_t)((cy * (int64_t)dt_q16 + ((int64_t)1 << 31)) >> 32);
        s_gest[2] -= (int32_t)((cz * (int64_t)dt_q16 + ((int64_t)1 << 31)) >> 32);

        /* 2. korekcja ACC: gate |w| < 15 dps (szybka rotacja => ufa gyro; BIAS zyroskopu
         *    do 15 dps MUSI przepuszczac — 2 dps powodowalo rampę, patrz trikig_vbt.h)
         *    I innowacja ||acc-g|| < 1.0 m/s^2. Cel = RAW acc (brak lagu w petli). Dodatkowo
         *    powolny leak 1/2048 ZAWSZE (net przeciw permanentnemu dead-lockowi przy duzym
         *    bledzie g — innowacja > 1.0 blokuje szybka sciezke, leak wyciaga stan). */
        int32_t dg[3] = { av[0] - s_gest[0], av[1] - s_gest[1], av[2] - s_gest[2] };
        int64_t pg = (int64_t)dg[0]*dg[0] + (int64_t)dg[1]*dg[1] + (int64_t)dg[2]*dg[2];
        if (abs(we[0]) < TRIKIG_VBT_GYR_REST_TH_Q16 &&
            abs(we[1]) < TRIKIG_VBT_GYR_REST_TH_Q16 &&
            abs(we[2]) < TRIKIG_VBT_GYR_REST_TH_Q16 &&
            pg < (int64_t)TRIKIG_VBT_G_CORR_MAX * TRIKIG_VBT_G_CORR_MAX) {
            for (int i = 0; i < 3; i++)
                s_gest[i] += dg[i] >> TRIKIG_VBT_G_CORR_SHIFT;
        }
        for (int i = 0; i < 3; i++)
            s_gest[i] += dg[i] >> TRIKIG_VBT_G_LEAK_SHIFT;

        /* 3. renormalizacja do g_ref (Euler rozmywa norme, kwantyzacja tez) */
        int64_t p = (int64_t)s_gest[0]*s_gest[0] + (int64_t)s_gest[1]*s_gest[1] + (int64_t)s_gest[2]*s_gest[2];
        if (p > (int64_t)UINT32_MAX) p = (int64_t)UINT32_MAX;
        int32_t gn = isqrt32((uint32_t)p);
        if (gn > 0) {
            for (int i = 0; i < 3; i++)
                s_gest[i] = (int32_t)(((int64_t)s_gest[i] * (int64_t)TRIKIG_VBT_G_REF_Q88) / gn);
        }
    });

    /* 4. linear acc: JEDNO LPF na roznicy (struktura z C6 — patrz naglowek) */
    VBT_PROF(lin_us_max,
    {
        for (int i = 0; i < 3; i++)
            s_lin[i] += ((av[i] - s_gest[i]) - s_lin[i]) >> TRIKIG_VBT_ACC_LPF_ALPHA;

        /* 5. detektor bezruchu: ||lin|| — 1. rzedu wzgledem a_perp (C6, patrz naglowek).
         *    P2: porownanie na kwadracie (bez isqrt). */
        int64_t pl = (int64_t)s_lin[0]*s_lin[0] + (int64_t)s_lin[1]*s_lin[1] + (int64_t)s_lin[2]*s_lin[2];
        if (pl < (int64_t)TRIKIG_VBT_REST_TH * TRIKIG_VBT_REST_TH) {
            if (s_rest_cnt < 255) s_rest_cnt++;
        } else {
            s_rest_cnt = 0;
        }
        s_rest = (s_rest_cnt >= TRIKIG_VBT_REST_FRAMES);

        /* 0.3.7: nauka biasu gyro w quasi-bezruchu (||lin|| < 1.2 m/s^2 i |w-wbias| <
         * 15 dps => raw gyro = bias; tau ~0.6s). Bez tego dryf propagacji podtrzymywal
         * blad g na progu gate'a innowacji => ZUPT nie gasil velocity przy ruchach. */
        int64_t pq = (int64_t)TRIKIG_VBT_REST_TH * TRIKIG_VBT_REST_TH *
                     TRIKIG_VBT_QUASI_REST_MULT * TRIKIG_VBT_QUASI_REST_MULT;
        if (pl < pq &&
            abs(we[0]) < TRIKIG_VBT_GYR_REST_TH_Q16 &&
            abs(we[1]) < TRIKIG_VBT_GYR_REST_TH_Q16 &&
            abs(we[2]) < TRIKIG_VBT_GYR_REST_TH_Q16) {
            for (int i = 0; i < 3; i++)
                s_wbias[i] += (wq[i] - s_wbias[i]) >> TRIKIG_VBT_WBIAS_LEARN_SHIFT;
        }
    });

    /* 6. projekcja na os ruchu -> integracja 1D */
    VBT_PROF(vel_us_max,
    {
        int32_t a_move = (s_lin[0] * s_axis[0] +
                          s_lin[1] * s_axis[1] +
                          s_lin[2] * s_axis[2]) >> 12;

        if (s_rest) {
            /* ZUPT: min krok 1 q8.8 — bez tego s_vel>>5 == 0 przy |v| < 125 mm/s i decay
             * STOI (bug dziedziczony z v0.0.24, zlapano w C6). */
            int32_t d = s_vel_q88 >> TRIKIG_VBT_BETA_LPF;
            if (d == 0 && s_vel_q88 != 0) d = (s_vel_q88 > 0) ? 1 : -1;
            s_vel_q88 -= d;
        } else {
            s_vel_q88 += (a_move * (int32_t)dt_q16) >> 16;   /* v += a*dt (dt q16.16) */
        }
        if (s_vel_q88 >  TRIKIG_VBT_V_CLAMP) s_vel_q88 =  TRIKIG_VBT_V_CLAMP;
        if (s_vel_q88 < -TRIKIG_VBT_V_CLAMP) s_vel_q88 = -TRIKIG_VBT_V_CLAMP;
    });
}

int32_t vbt_velocity_mms(void)
{
    /* oś ruchu (default X barbell); q8.8 -> mm/s */
    return (s_vel_q88 * 1000) >> 8;
}

bool vbt_moving(void)
{
    return !s_rest;
}

uint8_t vbt_flags(void)
{
    uint8_t f = 0;
    if (!s_rest)     f |= 0x01;   /* moving */
    if (s_rest)      f |= 0x02;   /* rest */
    if (s_gest_ok)   f |= 0x04;   /* g estimated (bylo: bias calibrated) */
    if (s_gest_forced) f |= 0x10; /* g z wymuszenia (fallback 5s w ruchu) — kalibracja niepewna */
    return f;
}
