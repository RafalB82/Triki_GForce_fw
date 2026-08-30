/**
 * vbt_offline/main.c - harness offline VBT (plan VBT C6)
 *
 * Kompiluje trikig_vbt.c na hoście i pędzi przez niego syntetyczne scenariusze
 * (104 Hz, te same konwersje raw co LSM6DSL @FS 16g/2000dps). Wykrywa regresje
 * estymatora grawitacji/osy/dt PRZED flashowaniem. Referencja "old" = model v0.2.0
 * (LPF - bias per os, tylko Vx, boot-hold + fallback 520 — wiernie) — pokazuje deltę.
 *
 * UWAGA: to sanity-regresja na danych syntetycznych; test C planu (MAE vs
 * offline-reference z realnych nagrań) pozostaje po stronie danych użytkownika
 * (feed przez stdin: 12B raw na klatkę, bez nagłówka — binarnie).
 *
 * Build:  ./build.sh
 * Run:    ./vbt_offline [scenariusz]     (rest60|rot|rot_move|rep|all)
 * stdin:  cat frames.bin | ./vbt_offline stdin
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../triki/trikig_vbt.h"

#define FS_HZ       104.0
#define DT_S        (1.0 / FS_HZ)
#define G           9.80665
#define ACC_LSB     (2048.0 / G)     /* 1 m/s^2 -> raw */
#define GYR_LSB     (1.0 / 0.07)     /* 1 dps -> raw @FS 2000dps */
#define DT_Q16      ((uint16_t)TRIKIG_VBT_DT_104HZ_Q16)

static int16_t acc_raw[3], gyr_raw[3];
static double s_v_old_mm;                    /* referencja stara [mm/s] */

static void pack_acc(double x, double y, double z)   /* m/s^2 */
{
    acc_raw[0] = (int16_t)lround(x * ACC_LSB);
    acc_raw[1] = (int16_t)lround(y * ACC_LSB);
    acc_raw[2] = (int16_t)lround(z * ACC_LSB);
}

static void pack_gyr(double x, double y, double z)   /* dps */
{
    gyr_raw[0] = (int16_t)lround(x * GYR_LSB);
    gyr_raw[1] = (int16_t)lround(y * GYR_LSB);
    gyr_raw[2] = (int16_t)lround(z * GYR_LSB);
}

static double old_model_step(const uint8_t *raw12);   /* referencja v0.2.0 (def. nizej) */

static void feed(void)
{
    uint8_t raw12[12];
    memcpy(raw12, gyr_raw, 6);
    memcpy(raw12 + 6, acc_raw, 6);
    vbt_on_frame(raw12, DT_Q16);
    s_v_old_mm = old_model_step(raw12);
}

/* ---- referencja stara (v0.2.0): linear = LPF - bias per os, velocity = Vx ---- */
#define OLD_LPF_A  5
#define OLD_MA_N   16
static int32_t o_bias[3], o_lpf[3], o_vx;
static int32_t o_bias_n, o_rest_cnt;
static uint16_t o_frames;

static int32_t o_q88(int16_t r) { return ((int32_t)r * 314) >> 8; }

static double old_model_step(const uint8_t *raw12)
{
    int16_t a[3];
    memcpy(a, raw12 + 6, 6);
    int32_t av[3] = { o_q88(a[0]), o_q88(a[1]), o_q88(a[2]) };
    for (int i = 0; i < 3; i++)
        o_lpf[i] += (av[i] - o_lpf[i]) >> OLD_LPF_A;

    double n = sqrt((double)o_lpf[0]*o_lpf[0] + (double)o_lpf[1]*o_lpf[1] + (double)o_lpf[2]*o_lpf[2]);
    double dev = fabs(n - TRIKIG_VBT_G_REF_Q88);
    if (dev < TRIKIG_VBT_REST_TH) { if (o_rest_cnt < 255) o_rest_cnt++; }
    else o_rest_cnt = 0;
    int rest = (o_rest_cnt >= (int)TRIKIG_VBT_REST_FRAMES);
    if (o_frames < 0xFFFF) o_frames++;

    if (rest) {
        if (o_bias_n < OLD_MA_N) {
            o_bias_n++;
            for (int i = 0; i < 3; i++) o_bias[i] += (o_lpf[i] - o_bias[i]) / o_bias_n;
        } else {
            for (int i = 0; i < 3; i++) o_bias[i] += (o_lpf[i] - o_bias[i]) / OLD_MA_N;
        }
        o_vx -= o_vx >> TRIKIG_VBT_BETA_LPF;
    } else if (o_bias_n < OLD_MA_N) {
        if (o_frames >= TRIKIG_VBT_BIAS_FORCE_FRAMES) {
            for (int i = 0; i < 3; i++) o_bias[i] = o_lpf[i];
            o_bias_n = OLD_MA_N;
        }                       /* boot-hold v0.2.0: bez spoczynku integracja stoi */
    } else {
        for (int i = 0; i < 3; i++) {
            int32_t lin = o_lpf[i] - o_bias[i];
            o_vx += (lin * 629) >> 16;   /* stare dt 629 (przed C5) */
            if (o_vx >  TRIKIG_VBT_V_CLAMP) o_vx =  TRIKIG_VBT_V_CLAMP;
            if (o_vx < -TRIKIG_VBT_V_CLAMP) o_vx = -TRIKIG_VBT_V_CLAMP;
        }
    }
    return ((double)o_vx * 1000.0) / 256.0;      /* q8.8 -> mm/s */
}

static void old_reset(void)
{
    memset(o_bias, 0, sizeof(o_bias));
    memset(o_lpf, 0, sizeof(o_lpf));
    o_vx = 0; o_bias_n = 0; o_rest_cnt = 0; o_frames = 0;
}

/* ---- scenariusze (wszystkie mms) ---- */
#define REST_WARM_FRAMES (2 * 104)   /* rozgrzewka LPF (~80f) + detekcja rest; jak realny boot */

static void scenario_rest60(void)
{
    printf("== rest60: 60s bezruchu (plan test A) ==\n");
    vbt_reset(); old_reset();
    double vmax = 0;
    for (int f = 0; f < 60 * 104; f++) {
        pack_acc(0, 0, G);
        pack_gyr(0, 0, 0);
        feed();
        double vn = fabs((double)vbt_velocity_mms());
        if (vn > vmax) vmax = vn;
    }
    printf("   max|v_new| = %.1f mm/s (limit 50) | max|v_old| = %.1f mm/s\n", vmax, fabs(s_v_old_mm));
    printf("   %s\n", vmax < 50.0 ? "PASS" : "FAIL");
}

/* pitch o Y: grawitacja w body = (-g sin th, 0, g cos th), gyro body Y.
 * with_move: dodatkowy oscylacyjny ruch liniowy X (defetuje rest => propagacja gyro). */
static void scenario_rot(int with_move)
{
    printf("== %s: 2s bezruch, potem pitch 0->45deg @45dps (2s)%s ==\n",
           with_move ? "rot_move" : "rot",
           with_move ? " + osc 5.0 m/s^2 2Hz X (propagacja bez korekcji)" : "");
    vbt_reset(); old_reset();
    const double w = 45.0 / 180.0 * M_PI;            /* rad/s */
    double vmax_new = 0, vmax_old = 0, tmove_vmax_new = 0, vend_new = 0;
    for (int f = 0; f < (REST_WARM_FRAMES + 6 * 104); f++) {
        double t = f * DT_S;
        double tm = t - 2.0;                          /* czas od startu pitch */
        double th = (tm > 0.0) ? w * fmin(tm, 2.0) : 0.0;
        double ax = -G * sin(th), az = G * cos(th);
        if (with_move && tm > 0.0 && tm < 3.0) ax += 5.0 * sin(2.0 * M_PI * 2.0 * tm);
        pack_acc(ax, 0, az);
        pack_gyr(0, (tm > 0.0 && tm < 2.0) ? 45.0 : 0.0, 0);
        feed();
        double vn = fabs((double)vbt_velocity_mms());
        if (vn > vmax_new) vmax_new = vn;
        if (fabs(s_v_old_mm) > vmax_old) vmax_old = fabs(s_v_old_mm);
        if (with_move && tm > 0.0 && tm < 3.0 && vn > tmove_vmax_new) tmove_vmax_new = vn;
        if (t > 7.0 && vn > vend_new) vend_new = vn;  /* powrot do zera po ustaniu ruchu */
    }
    if (!with_move) {
        printf("   max|v_new| = %.1f mm/s (limit 150) | max|v_old| = %.1f mm/s\n", vmax_new, vmax_old);
        printf("   %s\n", vmax_new < 150.0 ? "PASS" : "FAIL");
    } else {
        printf("   max|v_new| (faza ruchu) = %.1f mm/s (limit 500, znane ograniczenie: wander\n"
               "   przy rotacji+oscylacji jednoczesnie) | powrot po ruchu = %.1f mm/s (limit 50)\n",
               tmove_vmax_new, vend_new);
        printf("   %s\n", (tmove_vmax_new < 500.0 && vend_new < 50.0) ? "PASS" : "FAIL");
    }
}

static void scenario_rep(void)
{
    printf("== rep: 2s bezruch, half-sine push X 0.5s @6 m/s^2, potem ZUPT (plan test C) ==\n");
    vbt_reset(); old_reset();
    /* A=6 m/s^2 (realny bench 3-10), T=0.5s: dev ||a|-g| > prog przez wiekszosc push
     * => FW widzi ruch; przy A=2 syntetyk caly push byl "rest" (norma nie widzi wolnych
     * half-sinow) i integracja nigdy nie startowala — lesson learned C6. */
    const double T = 0.5, A = 6.0;
    double vref = 0, vpeak_new = 0, vpeak_ref = 0, dmax = 0, vend = 0;
    for (int f = 0; f < (REST_WARM_FRAMES + 3 * 104); f++) {
        double t = f * DT_S;
        double tp = t - 2.0;                          /* czas od startu push */
        double a = (tp >= 0.0 && tp < T) ? A * sin(M_PI * tp / T) : 0.0;
        if (tp >= 0.0 && tp < T) { vref += a * DT_S; if (fabs(vref) > vpeak_ref) vpeak_ref = fabs(vref); }
        pack_acc(a, 0, G);
        pack_gyr(0, 0, 0);
        feed();
        double vn = (double)vbt_velocity_mms();
        if (fabs(vn) > vpeak_new) vpeak_new = fabs(vn);
        if (fabs(vn - s_v_old_mm) > dmax) dmax = fabs(vn - s_v_old_mm);
        if (t > 4.5 && fabs(vn) > vend) vend = fabs(vn);
    }
    printf("   peak v_new = %.0f mm/s (ref %.0f mm/s) | max|v_new-v_old| = %.0f mm/s | v po 1.5s ZUPT = %.1f mm/s\n",
           vpeak_new, vpeak_ref * 1000.0, dmax, vend);
    /* dmax tylko diagnostyka: old (rest po LPF-normie) moze zostac w rest i nie integrowac */
    int ok = (vpeak_new > 0.35 * vpeak_ref * 1000.0) && (vpeak_new < 0.95 * vpeak_ref * 1000.0) && (vend < 50.0);
    printf("   %s\n", ok ? "PASS" : "FAIL");
}

/* wolny push (A=2.5): wymaga progu rest 0.3 (RAW) — przy starym 0.8 caly push w rest */
static void scenario_rep_soft(void)
{
    printf("== rep_soft: wolny push X 0.6s @2.5 m/s^2 (VBT musi widziec delikatne temposy) ==\n");
    vbt_reset(); old_reset();
    const double T = 0.6, A = 2.5;
    double vref = 0, vpeak_new = 0, vpeak_ref = 0, vend = 0;
    for (int f = 0; f < (REST_WARM_FRAMES + 3 * 104); f++) {
        double t = f * DT_S;
        double tp = t - 2.0;
        double a = (tp >= 0.0 && tp < T) ? A * sin(M_PI * tp / T) : 0.0;
        if (tp >= 0.0 && tp < T) { vref += a * DT_S; if (fabs(vref) > vpeak_ref) vpeak_ref = fabs(vref); }
        pack_acc(a, 0, G);
        pack_gyr(0, 0, 0);
        feed();
        double vn = (double)vbt_velocity_mms();
        if (fabs(vn) > vpeak_new) vpeak_new = fabs(vn);
        if (t > 4.5 && fabs(vn) > vend) vend = fabs(vn);
    }
    printf("   peak v_new = %.0f mm/s (ref %.0f mm/s) | v po 1.5s ZUPT = %.1f mm/s\n",
           vpeak_new, vpeak_ref * 1000.0, vend);
    int ok = (vpeak_new > 0.25 * vpeak_ref * 1000.0) && (vend < 50.0);
    printf("   %s\n", ok ? "PASS" : "FAIL");
}

/* regresja 0.3.7 (log 22:22): nauka biasu w quasi-bezruchu pochlonela WOLNA ROTACJE
 * (10 dps < gate 15dps) => wbias zatruty => propagacja nadrabia w zla strone => rampa.
 * Scenariusz: rest -> wolna rotacja 10 dps o Y (6s) -> rest. */
static void scenario_slowrot(void)
{
    printf("== slowrot: 2s rest, wolna rotacja 10 dps o Y (6s), rest 6s ==\n");
    vbt_reset(); old_reset();
    double vmax = 0;
    for (int f = 0; f < 14 * 104; f++) {
        double t = f * DT_S;
        double th = (t > 2.0 && t < 8.0) ? (10.0 / 180.0 * M_PI) * (t - 2.0) : ((t >= 8.0) ? 10.0/180.0*M_PI*6.0 : 0.0);
        pack_acc(-G * sin(th), 0, G * cos(th));
        pack_gyr(0, (t > 2.0 && t < 8.0) ? 10.0 : 0.0, 0);
        feed();
        double vn = fabs((double)vbt_velocity_mms());
        if (vn > vmax) vmax = vn;
    }
    printf("   max|v_new| = %.1f mm/s (limit 300) | max|v_old| = %.1f mm/s\n", vmax, fabs(s_v_old_mm));
    printf("   %s\n", vmax < 300.0 ? "PASS" : "FAIL");
}

/* audyt HW 2026-08-30 (log RTT): gyro bias > 2 dps => stary gate |w|<2dps trwale
 * zamkniety => gest rotuje z bias-em => innowacja > 1.0 => dead-lock => rampa do clampa.
 * Urzadzenie LEZY; bias zyroskopu 3 dps (realny wg spec LSM6DSL: max ±5 dps). */
static void scenario_bias(void)
{
    printf("== bias: 12s bezruchu, gyro bias +3 dps wokol Y (gate |w|<2dps nie moze blokowac korekcji) ==\n");
    vbt_reset(); old_reset();
    double vmax = 0;
    for (int f = 0; f < 12 * 104; f++) {
        pack_acc(0, 0, G);
        pack_gyr(0, 3.0, 0);                 /* bias 3 dps wokol Y (pitch — leje sie na os X) */
        feed();
        double vn = fabs((double)vbt_velocity_mms());
        if (vn > vmax) vmax = vn;
    }
    printf("   max|v_new| = %.1f mm/s (limit 150) | max|v_old| = %.1f mm/s\n", vmax, fabs(s_v_old_mm));
    printf("   %s\n", vmax < 150.0 ? "PASS" : "FAIL");
}

static int scenario_stdin(void)
{
    printf("== stdin: raw12 per frame, CTRL-D konczy ==\n");
    vbt_reset(); old_reset();
    uint8_t raw12[12];
    long n = 0;
    /* UWAGA: NIE przez feed() — ten buduje ramke ze statycznych gyr_raw/acc_raw
     * (zera); stdin idzie 1:1 do vbt_on_frame (audyt: replay zer przez 0.3.4). */
    while (fread(raw12, 1, 12, stdin) == 12) {
        vbt_on_frame(raw12, DT_Q16);
        s_v_old_mm = old_model_step(raw12);
        printf("%ld;%.0f;%.0f;%u\n", n, (double)vbt_velocity_mms(),
               s_v_old_mm, (unsigned)vbt_flags());
        n++;
    }
    fprintf(stderr, "przetworzono %ld ramek\n", n);
    return 0;
}

int main(int argc, char **argv)
{
    const char *sc = (argc > 1) ? argv[1] : "all";
    vbt_set_axis((const int16_t[3]){TRIKIG_VBT_AXIS_Q12, 0, 0});   /* os X (default, jawne) */
    if (!strcmp(sc, "rest60"))    { scenario_rest60(); return 0; }
    if (!strcmp(sc, "rot"))       { scenario_rot(0); return 0; }
    if (!strcmp(sc, "rot_move"))  { scenario_rot(1); return 0; }
    if (!strcmp(sc, "rep"))       { scenario_rep(); return 0; }
    if (!strcmp(sc, "rep_soft"))  { scenario_rep_soft(); return 0; }
    if (!strcmp(sc, "bias"))      { scenario_bias(); return 0; }
    if (!strcmp(sc, "slowrot"))   { scenario_slowrot(); return 0; }
    if (!strcmp(sc, "stdin"))     return scenario_stdin();
    if (!strcmp(sc, "all")) {
        scenario_rest60();
        scenario_rot(0);
        scenario_rot(1);
        scenario_rep();
        scenario_rep_soft();
        scenario_bias();
        scenario_slowrot();
        return 0;
    }
    fprintf(stderr, "uzycie: %s [rest60|rot|rot_move|rep|rep_soft|bias|stdin|all]\n", argv[0]);
    return 1;
}
