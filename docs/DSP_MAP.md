# TrikiG — MAPA: ALGORYTMY ON-SENSOR (pośrednie vs BHI385/nRF52840)

> Decyzja Rafała 2026-08-29: Triki_G (aplikacja) ogarnia wysoką warstwę; firmware kapsla
> przestaje być dumb-streamerem (jak stock) — bierze na siebie filtry/algorytmy, ile da
> sprzęt. To rozwiązanie POŚREDNIE w skali: stock(0) <-> kapsel(tutaj) <-> BHI385+nRF52840(full).
> Zasada nadrzędna: **raw stream zawsze nietknięty** (wire v1/v2), algorytmy = side-band/telemetria
> z flagami, walidacja dual-run vs Triki_G offline przed wlaczeniem domyślnym.

---

## 1. Twarde ograniczenia (decydują o tym co WCHODZI do mapy)

| Zasób | Wartość | Konsekwencja |
|---|---|---|
| MCU | nRF52810: Cortex-M4 @64MHz, **BEZ FPU**, 192K flash / **24K RAM** | zero float w hot-path (fixed-point q8.8/q16.16); float tylko w init/diag |
| RAM budżet | SD S112 ~5.5K + stack/heap 4K + bss 3.4K (v0.0.22) => wolne ~10K | stany DSP zmieszczą się; bez alokacji dynamicznych |
| Flash budżet | 24K/192K użyte | zapas ogromny — kod algorytmów bez presji |
| Zegar | LFCLK **RC** (brak 32k XTAL), dryf ±1-2% typ. | timestamps FW = licznik próbek + kompensacja w Triki_G (regresja dt vs zegar telefonu); NIE absolutny czas |
| CPU zajęte | **TWIM 400kHz + DRDY 104Hz (0.3.11 [P])**: DSP ~224us worst => ~2% CPU; acq ~60-600us | zostaje ~90%+ na DSP; FIFO odroczone (ODR >104 Hz) |
| IMU | LSM6DSL: acc/gyro 104-6.6kHz, **BRAK fusion/kwaternionów** (to ma BHI385) | orientacja = heurystyki 1D/2D + HPF; full AHRS odpada |
| IMU embedded | pedometer, tilt, significant-motion, wake-up, free-fall, HPF acc, FIFO, timestamp (AN+datasheet przed implementacją — D-017) | darmowe na sensorze — użyć zamiast liczyć na MCU |
| Energia | **DRDY 104Hz (INT2/P0.10) + TWIM 400kHz [P] 0.3.11** — brak pollingu 111Hz | ODR on-demand + sleep między próbkami = F7 |
| Protokół | wire v1 14B domyślny po boot | wire v2 komendą `20 11 01` (seq+vel+flags); nowe dane tylko v2 (D-019) |

## 2. Co to pozycjonuje (skala rozwiązań)

| Funkcja | Stock Zabka | TrikiG (ta mapa) | BHI385+nRF52840 |
|---|---|---|---|
| raw stream 14B | TAK | TAK (v1) | TAK (14B QUAT) |
| sample counter FW | NIE | **F1 TAK** (seq w wire v2) | TAK |
| velocity na urządzeniu | NIE | **F1 TAK** (wire v2 vel; gravity tracking 0.3.x) | TAK (Faza C/B) |
| gravity tracking ACC+GYRO | NIE | **TAK (0.3.x, ACC+GYRO komplementarny)** | TAK (fusion) |
| detekcja repa/faz | NIE | **F3** (velocity-based) | TAK (pipeline) |
| MCV/peak per rep | NIE | **F3** | TAK |
| orientacja/kwaterniony | NIE | NIE (heurystyki max) | TAK (SensorAPI) |
| konfiguracja z aplikacji | NIE | **F2 TAK** (20 10-17: wire/info/stream/sleep/batt) | TAK |
| bateria/status | NIE | **F5 TAK** (22 04, skala 1/1 [P]) | TAK |
| offline-record + sync | NIE (zapis szyfrowany) | **F6** (MX25R 1MB) | TAK |
| energy-aware (INT2/DRDY + TWIM) | NIE | **F7 TAK** (0.3.11 [P]) | TAK |

## 3. Mapa wdrożenia (fazy, semver wg D-020)

### F1 — wire v2: licznik + velocity w ramce (v0.1.0)
- Ramka v2 (po komendzie RX `20 11 01`): `22 01 | seq16 | gyro6 | acc6 | vel_i16 | flags8` (16B; seq = FW sample counter, vel = oś X [mm/s] z trikig_vbt; flags: moving/bias-ready).
- Odbiór: kompensacja dryfu RC w Triki_G (seq vs zegar telefonu).
- Odbiór: nRF Connect/PWA widzą dalej v1 (domyślny po boot).
- **Kryterium:** Triki_G liczy dt z seq (dt odchylenie <1ms steady-state), vel z FW koreluje z post-processingiem aplikacji (r > 0.9 na repach).

### F2 — komendy RX: kontrola sensora z aplikacji (v0.1.x)
- `20 11 0x` tryb wire; `20 12` FW info (wersja, build); `20 13 ODR` (104/208/26); `20 14 skale`; `20 15 stream on/off`; `20 16 sleep-now`.
- Walidacja komend (checksum) — reset segura przy garbage (NUS RX bez ochrony).
- **Kryterium:** Triki_G ustawia ODR 26Hz na idle i wraca do 104 na start serii; brak stanów zawieszonych po 100 komendach z błędami.

### F3 — DSP pack v1: detekcja repa on-sensor (v0.2.0)
- Na MCU, fixed-point, feed = istniejący vbt_on_frame:
  - HPF/LPF velocity (już jest), prog bazowy i hystereza => fazy CON/ECC/LOCKOUT,
  - licznik repów + MCV/MPV + peak velocity per rep,
  - auto-stop signal (spadek MCV > 20% vs seria — parametr z Triki_G).
- Emisja: ramka zdarzeniowa `22 02 | rep_idx16 | mcv16 | peak16 | dur16 | flags` (w v2), NIE zanieczyszcza raw.
- **Kryterium:** live pipeline Triki_G jako ground-truth: rep on-sensor vs repy Triki_G z tej samej sesji — zgodność >= 90% count, MCV r > 0.9; potem flaga domyślna.
- RAM: < 1KB stanów. CPU: < 2%.
- **Podstawa (v0.3.11, plan VBT C1-C11 WDROŻONA + [P] zwalidowana na sprzęcie):** gravity
  tracking ACC+GYRO (propagacja gyro + korekcja ACC gated + nauka biasu), movement-axis
  `dot(lin, axis)`, dt z timestampów DRDY (TIMER1), detektor rest 1. rzędu `||lin||<0.3`,
  ZUPT τ0.31s; akwizycja DRDY/TWIM; harness offline `tools/vbt_offline` (7 scenariuszy PASS)
  + replay realnych logów (`nrflog2raw.py`). Wejście do F3 gotowe.

### F4 — embedded features LSM6DSL (v0.2.x, tanie extras)
- Pedometer/sig-motion/tilt/wake-up jako metadane w flags (oszczędza CPU; identyfikacja aktywności, NIE VBT).
- Sprzętowy HPF acc (CTRL8) jako opcjonalne wsparcie biasu grav (A/B vs MA-16).
- **Warunek wejścia:** czytanie AN/datasheet tabel bitowych (D-017) + weryfikacja readbackiem; bez LA nie ruszamy FIFO.

### F5 — telemetria i niezawodność (v0.3.0)
- SAADC bateria (mV) w flags/ramce statusowej; watchdog; licznik dropów send-path jako telemetria; pomiar prądu (PPK) po drodze.
- **Bateria: WDROŻONA 0.2.0** (przed harmonogramem): SAADC AIN2/P0.04 (CR2032 przez diode,
  bez LDO), dzielnik 100k/100k (1:1), ramka `22 04` na RX `20 17`, flags v2 bit3 low-batt; OFFSET = Vf diody DO KALIBRACJI na egzemplarzu.
- **Kryterium:** WDT po sztucznym hangu; dropy widoczne w aplikacji; bateria ±50mV vs miernik.

### F6 — offline-record na MX25R8035F (v0.4.0)
- Nagrywanie serii (raw v1 + eventy) do flash 1MB bez telefonu; sync przez Triki_G po połączeniu (download + kasowanie sektorów).
- SPI driver (piny SCK=14 MOSI=15 MISO=16 [? schemat]; CS **NIE P0.12** — P0.12 = dzielnik
  baterii, plyta 2026-08-30; CS do ustalenia ze schematu) + wear-leveling prosty, weryfikacja JEDEC ID.
- **Kryterium:** 20 min treningu offline zapisane i zsynchronizowane 1:1 z live-streamem równoległym.

### F7 — energy optimization (v0.5.x, po LA)
- INT1 DRDY/FIFO watermark (weryfikacja polaryzacji na LA — D-016), TWIM 400kHz retry, sleep między próbkami, ODR on-demand z F2.
- **Kryterium:** prąd średni zmierzony, cel -50% vs F5 baseline przy 104Hz.
- **Częściowo wdrożone 0.3.1 (Faza 2):** INT1 DRDY z runtime auto-probe polaryzacji
  (bez LA; fallback polling + watchdog), TWIM 400kHz (hybryda z bb recovery), ring 16,
  dt z timestampów ISR. FIFO watermark: pending decyzji ODR >104 Hz.

### Bramka 1.0.0
- F1+F2+F3 stabilne, F5 wchodzi, tygodniowa terenówka bez resetów/utrata danych, Triki_G na wire v2 domyślnie.

## 4. Ryzyka i mitygacje

| Ryzyko | Mitygacja |
|---|---|
| Dryf zegara RC psuje interwały | seq counter + regresja w aplikacji; brak absolutnych timestampów |
| Velocity dryfuje bez ZUPT | ZUPT już jest; detekcja bezruchu z gyro-threshold w F3; walidacja dual-run zawsze |
| Błędy bitfield LSM6DSL (historia v11-v19) | datasheet + readback + osobne commity per feature |
| RAM wyczerpany przez stany DSP | budżet per moduł, statyczne alokacje, size-check w CI (make + size gate) |
| Wire v2 vs legacy klienci | brak legacy (D-021); v1 domyślny do 1.0.0, potem decyzja o wycofaniu |
| Rozjazd wyników FW vs aplikacja | walidacja vs live pipeline Triki_G (D-021); fixtures E-series tam gdzie trzeba offline |

## 5. Co NIE wchodzi (świadomie)

- Kwaterniony/AHRS/fusion — wymaga BHI385 (lub kosztownego fixed-point Madgwicka; re-evaluacja tylko gdy F3 da szybkie wygrane i będzie CPU zapas).
- Absolutne timestampy RTC — brak XTAL.
- Dowolny float w hot-path.
- Zmiany raw wire v1 (domyślny po boot, zawsze).
