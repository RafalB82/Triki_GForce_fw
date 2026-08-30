/**
 * trikig_vbt.h - Velocity Based Training: estymacja velocity barbell (os X)
 *
 * Side-band do streamu (kontrakt wire 14B NIEZMIENIONY): liczone na kapslu
 * z surowego acc, dostepne przez API dla przyszlego pola statusu / profilu BLE.
 *
 * Model (O-012): v = integral a_lin dt, gdzie a_lin = LPF(acc) - bias_gravity.
 * Bezruch: ||a|-g| < 0.8 m/s^2 przez 8 ramek (NIE po osi — patrz trikig_vbt.c
 * naglowek: pułapka orientacji v0.0.22/23). Bias grawitacji: MA-16 przy kazdym bezruchu;
 * fallback 0.1.1: wymuszona kalibracja z LPF po ~5s bez bezruchu (BIAS_FORCE_FRAMES).
 * ZUPT: przy bezruchu v -> 0 (first-order decay).
 * Ograniczenie biomechaniczne: ruch barbell = os podluzna gryfu (os X ukladu).
 *
 * Ograniczenia (uczciwie): dryf po usunieciu biasu (ZUPT tylko w bezruchu),
 * os tylko X, bez fuzji z gyro. Walidacja vs PWA/G-Path przed uzyciem produkcyjnym.
 */
#ifndef TRIKIG_VBT_H
#define TRIKIG_VBT_H

#include <stdint.h>
#include <stdbool.h>

/* --- parametry (q8.8: 256 = 1.0) --- */
#define TRIKIG_VBT_ACC_LPF_ALPHA   5u      /* LPF acc: 1/32 (fc ~0.5Hz @104Hz) */
#define TRIKIG_VBT_BETA_LPF        6u      /* ZUPT decay: 1/64 na ramke (tau ~0.6s) */
#define TRIKIG_VBT_G_REF_Q88       2514u   /* g = 9.8197 m/s^2 q8.8 (ref normy; err 0.03%) */
#define TRIKIG_VBT_REST_TH         205u    /* 0.80 m/s^2 — prog ||a|-g| dla bezruchu */
#define TRIKIG_VBT_REST_FRAMES     8u      /* ~77ms ciaglego spoczynku => rest */
#define TRIKIG_VBT_V_CLAMP         0x0FA0  /* 4000 q8.8 = 15.6 m/s — anty-windup */
_Static_assert(TRIKIG_VBT_V_CLAMP <= 0x7FFF,
               "V_CLAMP must fit int16_t: wire v2 packs vel as i16 (audyt 027)");
#define TRIKIG_VBT_BIAS_MA_N       16u     /* okno MA biasu (ramki) */
#define TRIKIG_VBT_BIAS_FORCE_FRAMES 520u  /* ~5s @104Hz (audyt#5 2026-08-30): fallback — wymuszona
                                              * kalibracja biasu z LPF, gdy caly ten czas ruch
                                              * (velocity inaczej nie wystartuje nigdy) */

void    vbt_reset(void);                    /* zero stanu; wywolac po imu init */
void    vbt_on_frame(const uint8_t *raw12); /* feed: 12B z OUT 0x22 (gyro6+acc6 i16LE) */
int32_t vbt_velocity_mms(void);             /* velocity os X [mm/s] (podpisane) */
bool    vbt_moving(void);
uint8_t vbt_flags(void);                    /* bit0 moving, bit1 rest, bit2 bias-calibrated */

#endif /* TRIKIG_VBT_H */
