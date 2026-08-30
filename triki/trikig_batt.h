/**
 * trikig_batt.h - pomiar baterii CR2032 przez SAADC (F5, DSP_MAP)
 *
 * Sprzet (zweryfikowane na plycie 2026-08-30): CR2032 3V -> bezposrednio na VDD
 * (brak LDO) oraz przez diode do dzielnika rezystorowego; wylot dzielnika na
 * P0.04/AIN2 (i rownolegle P0.12 — kolizja z CS MX25R, patrz SPEC 1).
 * Pomiar = Vbat minus Vf diody (kompensacja w OFFSET_MV, kalibracja na egzemplarzu).
 *
 * Konfiguracja: gain 1/6, ref wewnetrzny 0.6V => full-scale 3.6V > max Vbat ~3.3V.
 * Srednia software'owa z 4 probek (blokujaca, ~0.5ms — wolac max 1x/s z ticku 1s).
 * Kalibracja offsetu SAADC przy init (batt_init blokuje do ~5ms).
 */
#ifndef TRIKIG_BATT_H
#define TRIKIG_BATT_H

#include <stdint.h>
#include <stdbool.h>

/* --- konfiguracja pomiaru --- */
#define TRIKIG_BATT_AIN          2u      /* AIN2 = P0.04 (wylot dzielnika) */
#define TRIKIG_BATT_SAMPLES      4u      /* srednia software'owa (blokujaca) */

/* Skala node->Vbat: Vbat = node_mV * NUM / DEN + OFFSET_MV.
 * DO KALIBRACJI multimetrem na egzemplarzu (ratio dzielnika + Vf diody);
 * wartosci startowe zakladaja dzielnik 1:1 z pominieciem Vf. Kryterium F5: ±50mV. */
#define TRIKIG_BATT_SCALE_NUM    2u
#define TRIKIG_BATT_SCALE_DEN    1u
#define TRIKIG_BATT_OFFSET_MV    0

/* Prog low-battery: CR2032 pod obciazeniem koniec zycia ~2.4V (OCV do ~2.0V). */
#define TRIKIG_BATT_LOW_MV       2400u

/* flags v2: bit3 = low-battery (bity 0-2 = VBT, patrz trikig_vbt.h) */
#define TRIKIG_BATT_FLAGS_LOW    0x08u

void     batt_init(void);        /* SAADC init + kalibracja offsetu (blokujaca) */
uint16_t batt_sample_mv(void);   /* srednia z N probek -> Vbat [mV]; 0 = pomiar niemozliwy */
bool     batt_is_low(uint16_t mv);

#endif /* TRIKIG_BATT_H */
