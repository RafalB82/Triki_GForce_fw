/**
 * trikig_vbt.h - Velocity Based Training: gravity tracking ACC+GYRO + movement-axis
 *                velocity (plan VBT C2-C5, v0.3.0)
 *
 * Side-band do streamu (kontrakt wire 19B NIEZMIENIONY): liczone na kapslu z
 * surowego acc+gyro, dostepne przez API dla pola vel/flags wire v2.
 *
 * Model (plan VBT-01/02/03):
 *   g_est (q8.8) propagowany gyro: dg = -(w x g)*dt, renormalizacja do g_ref
 *   korekcja ACC tylko w bezruchu: g_est += (LPF - g_est) >> 5 (tau ~0.3s)
 *   linear_acc = LPF(acc) - g_est          <- poprawne po zmianie orientacji
 *   a_move = dot(linear, axis_q12)         <- os ruchu (default X = gryf, kapsel
 *                                             na gryfie; set przez vbt_set_axis)
 *   v_move += a_move * dt                  <- dt = 1/ODR (q16.16), parametr wywolania;
 *                                             C7 DRDY podmieni na mierzony timestamp
 * Bezruch (v0.3.0): ||LPF(acc) - LPF(g_est)|| < 0.3 m/s^2 przez 8 ramek — detektor 1. rzedu
 * (stara norma ||a|-g|, lekcja v0.0.22/23, byla 2. rzedu i slepa na wolne pushy — C6).
 * ZUPT: przy bezruchu v -> 0 (decay z min-krokiem).
 * Fallback (audyt 0.1.1): wymuszona kalibracja z LPF po ~5s bez bezruchu.
 *
 * Ograniczenia (uczciwie): korekcja g tylko w bezruchu => dryf gyro-bias podczas
 * dlugich quasi-statycznych trzyman (typ. 40mdps -> ~2.4deg/min => ~0.4 m/s^2);
 * os stalych w ukladzie kapsla (adaptacja osi po stronie hosta pozniej);
 * walidacja vs PWA/G-Path (testy offline C6) przed uzyciem produkcyjnym.
 */
#ifndef TRIKIG_VBT_H
#define TRIKIG_VBT_H

#include <stdint.h>
#include <stdbool.h>

/* --- parametry (q8.8: 256 = 1.0) --- */
#define TRIKIG_VBT_ACC_LPF_ALPHA   5u      /* LPF acc: 1/32 (fc ~0.5Hz @104Hz) */
#define TRIKIG_VBT_BETA_LPF        5u      /* ZUPT decay: 1/32 na ramke (tau ~0.31s) — rest => v=0
                                              * zbiezne szybko (0.6s bylo za wolne: 1.5s po pushu
                                              * zostawalo ~20% v; C6). Min-krok patrz trikig_vbt.c */
#define TRIKIG_VBT_G_REF_Q88       2514u   /* g = 9.8197 m/s^2 q8.8 (ref normy; err 0.03%) */
#define TRIKIG_VBT_REST_TH         77u     /* 0.30 m/s^2 — prog ||LPF(acc)-LPF(g_est)|| dla bezruchu
                                            * (v0.3.0 detektor 1. rzedu: stara norma ||a|-g| dala
                                            * dev ~ a^2/2g i nie widziala wolnych pushy; 0.8 bylo
                                            * strojone pod szum LPF-normy; immunity: 8 ramek) */
#define TRIKIG_VBT_REST_FRAMES     8u      /* ~77ms ciaglego spoczynku => rest */
#define TRIKIG_VBT_V_CLAMP         0x0FA0  /* 4000 q8.8 = 15.6 m/s — anty-windup */
_Static_assert(TRIKIG_VBT_V_CLAMP <= 0x7FFF,
               "V_CLAMP must fit int16_t: wire v2 packs vel as i16 (audyt 027)");
#define TRIKIG_VBT_G_CORR_SHIFT    5u      /* korekcja g w bezruchu: 1/32 na ramke */
#define TRIKIG_VBT_BIAS_FORCE_FRAMES 520u  /* ~5s @104Hz: fallback — wymuszona kalibracja
                                              * g z LPF, gdy caly ten czas ruch (velocity
                                              * inaczej nie wystartuje nigdy) */
#define TRIKIG_VBT_GYR_Q16_RAD_LSB 80      /* gyro->q16.16 rad/s @FS 2000dps:
                                              * 70 mdps/LSB * pi/180 * 2^16 = 80.07 */
#define TRIKIG_VBT_GYR_REST_DPS    15      /* korekcja g aktywna gdy |w| < 15 dps I innowacja
                                              * < G_CORR_MAX. 2 dps bylo martwaca: bias zyroskopu
                                              * (spec LSM6DSL do ±5 dps) sam zamykal gate => korekcja
                                              * nigdy => gest rotuje z biasem => rampa velocity do
                                              * clampa (log RTT 0.3.3, repro: harness scenario bias).
                                              * Cel korekcji = RAW acc => lag-rotacja nie dotyczy;
                                              * 15 dps blokuje nadal szybka rotacje (45 dps). */
#define TRIKIG_VBT_GYR_REST_TH_Q16 (TRIKIG_VBT_GYR_REST_DPS * 1144)  /* 1 dps ~ 1144 q16.16 rad/s */
#define TRIKIG_VBT_G_CORR_MAX      256u    /* max ||acc-g_est|| dla korekcji [q8.8 = 1.0 m/s^2]:
                                              * gate innowacji — korekcja NIE moze zalezec od
                                              * s_rest (dead-lock: blad g => "moving" => korekcja
                                              * nigdy => permanentny falszywy ruch; C6), ani ruszac
                                              * podczas pusha (absorpcja ruchu => bounce po repie) */
#define TRIKIG_VBT_G_LEAK_SHIFT    11      /* powolny leak korekcji 1/2048 ZAWSZE (audyt HW 0.3.3):
                                              * net przeciw permanentnemu dead-lockowi gdy innowacja
                                              * > G_CORR_MAX trwale (duzy blad g, np. forced-snap w
                                              * ruchu): ~20s powrotu, po czym szybka korekcja wchodzi */
#define TRIKIG_VBT_WBIAS_LEARN_DPS  5      /* nauka biasu TYLKO przy |w-wbias| < 5 dps I pelnym
                                              * rest (0.3.9: przy quasi-rest nauka pochlaniala
                                              * wolne rotacje => wbias zatruty => rampa) */
#define TRIKIG_VBT_WBIAS_LEARN_SHIFT 6     /* nauka biasu gyro w pelnym bezruchu (0.3.7/0.3.9):
                                              * 1/64 na ramke (tau ~0.6s); dryf propagacji z biasu
                                              * podtrzymuje blad g na progu gate'a => ||lin|| > prog
                                              * => ZUPT nie gasi velocity (raczeta przy ruchach) */
#define TRIKIG_VBT_QUASI_REST_MULT 4u       /* quasi-bezruch: ||lin|| < 4x prog rest (1.2 m/s^2) */
#define TRIKIG_VBT_DT_104HZ_Q16    630u    /* 1/104 s w q16.16 (=630.15); C5: dt=1/ODR,
                                              * wyprostowane z 629 (err 0.16%) */
#define TRIKIG_VBT_AXIS_Q12        4096u   /* wektor osi ruchu: q12, |axis| = 4096 */

void    vbt_reset(void);                    /* zero stanu; wywolac po imu init */
void    vbt_idle(bool idle);                /* v0.4.0 IDLE-CONNECTED: true = gyro w power-down
                                              * (zamrozone OUT) => bez propagacji/nauki biasu */
void    vbt_on_frame(const uint8_t *raw12, uint16_t dt_q16); /* feed: 12B z OUT 0x22
                                              * (gyro6+acc6 i16LE) + dt [q16.16] */
void    vbt_set_axis(const int16_t axis_q12[3]);  /* os ruchu (normalizowana do 4096) */
void    vbt_reset_velocity(void);           /* twardy ZUPT: v=0 (C7: gap dt > 60ms) */
int32_t vbt_velocity_mms(void);             /* velocity na osi ruchu [mm/s] (podpisane) */
bool    vbt_moving(void);
uint8_t vbt_flags(void);                    /* bit0 moving, bit1 rest, bit2 g-estimated,
                                              * bit4 g-forced (fallback 5s w ruchu => niepewna) */

#endif /* TRIKIG_VBT_H */
