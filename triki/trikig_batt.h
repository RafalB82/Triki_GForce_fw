/**
 * trikig_batt.h - pomiar baterii CR2032 przez SAADC (F5, DSP_MAP)
 *
 * Sprzet (POMIAR FW 0.3.3, pierwszy rzeczywisty odczyt na plycie): AIN2/P0.04 widzi
 * ~pelne Vbat — odczyt FW ze skala 2/1 dal 6595mV przy realnych 3.308V => 2× za duzo.
 * Wczesniejszy zapis "dzielnik 100k/100k potwierdzony plyta" byl pomiarem zlego punktu:
 * P0.04 i P0.12 NIE sa jednym node (P0.12 moze byc prawdziwym wylotem dzielnika —
 * do potwierdzenia miernikiem; NIE przelaczac AIN bez weryfikacji plyty).
 * Do diody (jesli w sciezce): kompensacja w OFFSET_MV (SPEC 5.2), teraz 0 — roznica
 * pomiaru FW vs miernik 10.5mV => w praktyce sciezka bez istotnego Vf.
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
#define TRIKIG_BATT_AIN          2u      /* AIN2 = P0.04 (Vbat, bez dzielnika — pomiar FW 0.3.3) */
#define TRIKIG_BATT_SAMPLES      4u      /* srednia software'owa (blokujaca) */

/* Skala node->Vbat: Vbat = node_mV * NUM / DEN + OFFSET_MV.
 * 0.3.3: NUM=1, DEN=1 — P0.04 = Vbat (walidacja: 6595mV @ skala 2/1 vs real 3.3080V,
 * dokladnosc po korekcie ~0.3% — SAADC). Jesli miernik kiedys pokaze inny stosunek,
 * zaktualizowac NUM/DEN + wpis do SPEC. Kryterium F5: ±50mV. */
#define TRIKIG_BATT_SCALE_NUM    1u
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
