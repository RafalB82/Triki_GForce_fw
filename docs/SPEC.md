# TRIKIG (kapsel Triki) — SPECYFIKACJA FIRMWARE

> Baseline rozwoju: **v22** (rebrand trikig + modul VBT). Linia: v19 (produkcja) -> v21 (refaktor, boot zielony 08:09, log PWA pending) -> v22 (rebrand+VBT, build zielony).
> BOOT v21 POTWIERDZONY (RTT 2026-08-29 08:09): S1..S5, FW=v0.0.21 c1=44 c2=4C, imu_ok=1 try=0, adv "Triki GForce" err=0.
> **DSP_MAP.md** (obok) = mapa algorytmow on-sensor (filtry/detekcja repa/offline-record) z ograniczeniami MCU/IMU — droga do 1.0.0.
> Ten dokument = punkt wyjscia dla dalszego rozwoju. Zmiany w kontrakcie (sekcja 5) wymagaja koordynacji z **Triki_G** (glowny konsument, D-019); PWA = legacy.
> Statusy: [Z] zweryfikowane na sprzecie/logiem, [D] decyzja (DECISIONS), [?] niezweryfikowane.

---

## 1. Sprzet

| Element | Detal | Status |
|---|---|---|
| MCU | nRF52810 (192K flash / 24K RAM, Cortex-M4, bez FPU utilized) | [Z] |
| SoftDevice | S112 7.2.0 @ 0x0, app @ 0x19000 | [Z] |
| IMU | LSM6DSL, I2C addr 0x6A (SA0=P0.04 -> VDD), WHO_AM_I(0x0F)=0x6A | [Z] |
| Flash zewn. | MX25R8035F 1MB SPI (CS=12, SCK=14, MOSI=15, MISO=16) — NIE uzywany | [?] piny ze schematu |
| Kryształ 32k | BRAK -> LFCLK = RC wewnetrzny (SRC=0, CTIV=16) | [Z] |
| Debug | SWD pady (3V3/GND/nRESET/SWDIO/SWCLK), probe: Tigard/Win11 lub Pico+Free-DAP; RTT ch0 | [Z] |
| Zasilanie | bateria + LDO (en=06 wg schematu [?]); pomiar baterii: brak (roadmap SAADC) | — |

### Pinout uzywany przez FW
| Pin | Funkcja | Konfiguracja |
|---|---|---|
| P0.05 | I2C SDA (bit-bang) | open-drain S0D1 + pullup wew. |
| P0.06 | I2C SCL (bit-bang) | open-drain S0D1 + pullup wew. |
| P0.09 | LSM INT1 | input NOPULL (nieuzywany w strategii poll; wyjscie na przyszly FIFO/DRDY) |
| P0.25 | BTN do GND | input PULLUP, active-low; sense dla wybudzenia z SYSTEMOFF |
| P0.28 | LED | output, active-low |

Uwaga: P0.09/P0.10 to G-klasa INT (aktywnie-low wg schematu) — polaryzacja niezweryfikowana [?].

---

## 2. Architektura (v21)

Bare-metal + nRF5 SDK 17 (bez RTOS). 4 moduly + main:

```
main.c              boot sekwencja, BLE stack/timers, petla glowna (send + BTN + sleep)
trikig_bb_i2c.c/h   bit-bang I2C master ~250kHz (start/restart/stop/ack/bus-clear)
trikig_lsm6dsl.c/h  init (WHO_AM_I retry, SW reset, config) + poll OUT 0x22 (burst 12B)
trikig_board.c/h    LED/BTN/SYSTEMOFF + rtt_diag_printf (makro pod TRIKIG_RTT_DIAG)
```

### Przeplyw danych (jeden kierunek, SPSC)
```
app_timer(9ms) -> lsm6dsl_read_motion() [BB I2C, 12B z 0x22]
  -> ramka 14B [0x22,0x00 | gyro6 | acc6] -> ring[4] (drop najnowszej przy pelnym)
     -> petla glowna -> ble_nus_data_send (14B/notify) -> PWA (WebBLE)
```

### Kolekcja
- Poll app_timer **9ms** (~111Hz tick) vs ODR **104Hz** -> okazjonalne duplikaty probek zamiast dropow (v19 [Z]; pkt 3 O-013 do decyzji).
- Poll tylko przy aktywnym polaczeniu (handler early-return bez polaczenia).

---

## 3. Konfiguracja LSM6DSL (kontrakt rejestrów)

Wartosci WYLACZNIE z tabeli datasheet (D-017); readback w RTT przy boocie.

| Rejestr | Wartosc | Znaczenie |
|---|---|---|
| CTRL1_XL (0x10) | 0x44 | ODR 104Hz (bity 7:4=0100) + FS_XL=01=**16g** (tabela NIEMONOTONICZNA: 00=2g,01=16g,10=4g,11=8g) |
| CTRL2_G (0x11) | 0x4C | ODR 104Hz + FS_G=11=**2000dps** (monotoniczna) |
| CTRL3_C (0x12) | 0x44 | BDU + IF_INC |
| FIFO_CTRL5 (0x0A) | 0x00 | FIFO bypass (strategia poll; FIFO nie dziala na tym egzemplarzu bez LA — D-016) |

Sensitivity: acc 2048 LSB/g, gyro 16.4 LSB/dps (= skale stocka, 1:1).

---

## 4. BLE

| Parametr | Wartosc |
|---|---|
| Nazwa GAP | **"Triki GForce"** (od v21; wczesniej "Triki") |
| Advertising | FAST 40ms, bez timeoutu; full name w advdata + NUS UUID complete w srdata |
| Serwis | NUS (6e400001-...), RX=0x0002 (host->FW), TX=0x0003 (FW->host, notify) |
| MTU | 247 (sdk_config), ramka 14B idzie jednym notify |
| Conn params | 7.5–15ms, latency 0, sup timeout 4s; update request po 5s |
| Init command (RX) | `20 10 00 D0 07 68 00 03` -> ustawia m_stream_on (kompatybilnosc stocka; stream i tak startuje wlaczony) |

Konsumenci: **TYLKO Triki_G** (D-019/D-021 — PWA/WebBLE wycofane calkowicie, nie sa wazonym klientem; historyczne logi PWA 2026-08-29 = dowoly testowe).

---

## 5. KONTRAKT WIRE (zamrozony, D-017 pkt 6)

```
Ramka 14B: [0x22][0x00] [gX_l gX_h gY_l gY_h gZ_l gZ_h aX_l aX_h aY_l aY_h aZ_l aZ_h]
           header 2B    | gyro FIRST (6B)                  | acc (6B)                         |
           wszystkie i16 LE, raw z sensora (MSB-first w bajtach parami)
```

### 5.1 wire v2 (0.1.0, przełączalny komendą `20 11 01`, powrót `20 11 00`)

```
Ramka 19B: [0x22][0x01] [seq_l seq_h] [gyro6] [acc6] [vel_l vel_h] [flags]
           header 2B  | seq u16LE | gyro FIRST (6B) | acc (6B) | vel i16LE mm/s | flags u8
           flags: bit0 moving, bit1 rest, bit2 bias-calibrated; vel+flags = snapshot VBT z chwili poll
```

**Semantyka seq (0.1.0, plan 027 K4):** licznik każdej próbki IMU przy aktywnym połączeniu —
również dropniętej przy pełnym ringu. Luki w seq są OCZEKIWANE i informacyjne (realny drop
przed BUFOR), nie błędem transportu. Konsument (Triki_G pushDecoded) agreguje luki jako seqGaps.
Brak połączenia BLE = brak inkrementacji (to "nie zbieramy", nie drop).
- Host: acc_si = n / 2048 * 9.80665 [m/s^2]; gyro_dps = n / 16.4.
- PWA eksportuje SI — |a|~9.81 w spoczynku jest POPRAWNE (D-017 pkt 5).
- Zmiana layoutu/skali = decyzja cross-project z koordynacja Triki_G (D-019). Wire v1 do wycofania w 1.0.0 (D-021).

---

## 6. Power / sleep

- Sleep: **300s bez polaczenia** (app_timer 1s, flaga g_go_sleep, SYSTEMOFF z petli glownej — nie z kontekstu SWI).
- Wybudzenie: BTN sense -> reset -> boot blink -> adv.
- BTN w trybie czuwania: 3 wcisniecia = 2x mrug + reset licznika sleep.
- Pobor pradu: NIE mierzony (roadmap O-013 pkt 4).

---

## 7. Error handling / diagnostyka

- APP_ERROR_CHECK = fail-stop -> fault handler: RTT kod+linia, LED SOS (3x40/40 + 1s) w petli.
- adv_start fail -> petla SOS.
- imu_init: 5 prob x (WHO_AM_I 3x z bus-clear + SW reset); fail -> "IMU DEAD" w RTT, FW i tak startuje BLE.
- RTT diag pod `TRIKIG_RTT_DIAG=1` (Makefile default ON): S1..S5, FW tag `FW=v0.0.21 c1=44 c2=4C`, blad APP_ERROR. Produkcja: flaga OFF = pelna cisza (FW tag tez).

## 8. Budowanie i flash

- Host: Ubuntu 104 `~/trikig-fw/triki/` (SOURCE OF TRUTH), GCC 13 systemowy, SDK `/home/ubuntu/nrf5sdk`.
- Build: `make -j4` -> `_build/nrf52810_xxaa.hex`. v21: text=23532 data=140 bss=3348.
- Flash (Win11/OpenOCD): SD->app po erase; **app-only OK gdy SD siedzi**; po recover ZAWSZE SD+app (D-017 pkt 3).
- AP lock przy SWD = najpierw wybudz (BTN/zasilanie), recover = ostatecznosc (D-017 pkt 4).
- Artefakty: SMB `Triki_G/nrf/trikig_triki_v0.0.21.hex` (sha256 d665b372... — PO WYMIANIE: e6c3796c, tag semver) + `s112_nrf52_7.2.0_softdevice.hex`.
- Backup v20: `main.c.v20.bak` na 104.

---

## 9. Metryki referencyjne (kryteria regresji)

Z logu PWA v19 23:12 (v21 musi je powtorzyc):
- rate **104.0Hz** end-to-end, dt med 9.60ms / max 9.7ms, zero zjadan
- spoczynek |a| = 9.973 m/s^2 (SI), gyro realne dps (max ~173 przy ruchu)
- RTT: `FW=v0.x.y c1=44 c2=4C`
- PWA: kalibracja przechodzi, trening loguje

---

---

## 10.5 Modul VBT (v22, side-band O-012)

**Cel:** velocity barbell na kapslu (velocity-based training) bez zmian w wire.

| Element | Detal |
|---|---|
| Wejscie | surowe acc 12B z OUT 0x22 (feed w poll handler, przed ramka) |
| Model | v = integral (LPF(acc) - bias_grav) dt; os X barbell |
| Bias grawitacji | MA-16 ramek, aktualizowany TYLKO w bezruchu (start = kapsel lezy przy boocie, okienko kalibracji PWA) |
| ZUPT | decay 1/64/ramke przy bezruchu (tau ~0.6s) |
| Progi | ruch >1.60 m/s^2 (q8.8), clamp v 15.6 m/s |
| Arytmetyka | fixed-point q8.8 (brak FPU wywolan; +448B text) |
| API | vbt_reset() po imu_init; vbt_on_frame(raw12); vbt_velocity_mms() [mm/s]; vbt_moving() |
| Diag | RTT ~1s: `VBT v=<mm/s> mv=<0/1>` (pod TRIKIG_RTT_DIAG) — walidacja vs PWA |
| Ograniczenia | dryf miedzy ZUPT-ami, os X only, brak fuzji gyro; walidacja terenowa przed produkcyjnym uzyciem |
| Expose | API wewnetrzne; docelowo: pole statusu w ramce LUB profil BLE (decyzja + koordynacja PWA) |

---
## 10. Znane ograniczenia (stan v21)

1. Duplikaty probek (timer 9ms vs ODR 104Hz) — do decyzji: 10ms/dryf albo INT1 DRDY [O-013 pkt 3].
2. Brak backpressure: NRF_ERROR_RESOURCES = drop ramki (brak buforowania na conn interval) [O-013 pkt 2].
3. Brak pomiaru baterii (SAADC + status w ramce/char) [O-013 pkt 5].
4. Brak watchdog [O-013 pkt 6].
5. BB I2C ~250kHz = CPU-heavy burst (12B @104Hz — OK, ale bez DMA).
6. TWIM i FIFO nie dzialaja na tym sprzecie bez diagnozy LA [O-013 pkt 8, AN4650].
7. INT1 polaryzacja niezweryfikowana; INT1 skonfigurowany jako wejscie (dead w strategii poll).
8. m_stream_on zawsze true po starcie (init-komenda tylko potwierdza).
9. v21/v22 bez logow PWA i testow terenowych (produkcja pozostaje v19; v21 boot zielony).

## 11. Roadmap rozwoju (po D-019: Triki_G primary, swoboda hardware)

1. Test v21/v22 (kryteria: sekcja 9) -> promocja produkcji; walidacja VBT vs Triki_G.
2. **Wire v2 (przelaczalny, koordynacja z Triki_G):** FW-side sample counter / timestamp ms (Android timestampy burstowe: dt 0..51ms), velocity (juz liczone w v22), bateria; tryb legacy 14B zostaje. MECHANIZM (decyzja Rafała 2026-08-29): rozpoznanie wersji przez KOMENDĘ RX po connect (aplikacja przełącza tryb), bez osobnej charakterystyki. Propozycja formatu (do ustalenia z Triki_G przy implementacji): `20 11 01` = wire v2 on, `20 11 00` = legacy, `20 12` = FW info (wersja/wiecej); komenda FW_INFO przed wlaczeniem v2 jako handshake.
3. **Komendy RX (kontrola sensora z aplikacji):** stream on/off, ODR/skale, sleep-now, FW info — rozszerzenie handlera 0x20 0x10.
4. Send-path: backpressure + licznik dropow (jako telemetria dla Triki_G).
5. Sleep + pomiar pradu; SAADC bateria.
6. Watchdog + nieblokujacy error-path.
7. RTT OFF build produkcyjny (TRIKIG_RTT_DIAG=0).
8. **Offline-record na MX25R8035F (1MB):** nagrywanie serii bez telefonu, sync przez Triki_G — pelna swoboda treningowa.
9. (opcjonalnie, po LA) TWIM 400kHz + FIFO + INT1.
10. Velocity/Kalman v2 na kapslu (O-012/Triki O-046) — po walidacji VBT v1.
