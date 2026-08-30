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
| Flash zewn. | MX25R8035F 1MB SPI — NIE uzywany; CS **NIE jest na P0.12** (P0.12 = dzielnik baterii, zweryfikowane plyta 2026-08-30); CS/SCK/MOSI/MISO nieznane | [?] piny ze schematu |
| Kryształ 32k | BRAK -> LFCLK = RC wewnetrzny (SRC=0, CTIV=16) | [Z] |
| Debug | SWD pady (3V3/GND/nRESET/SWDIO/SWCLK), probe: Tigard/Win11 lub Pico+Free-DAP; RTT ch0 | [Z] |
| Zasilanie | CR2032 3V, bez LDO — bezposrednio na VDD (pin 9); diode + rezystory do node'a baterii — **AIN2/P0.04 = ~pelne Vbat (bez dzielnika 2×)**; rola P0.12 do potwierdzenia miernikiem | [P] FW 0.3.3/0.3.4: 6595mV @ skala 2, po korekcie 3310mV vs real 3308mV (Δ2mV) |
| Pomiar baterii | SAADC AIN2 (gain 1/6, ref 0.6V, FS 3.6V), srednia 4x, kalibracja offsetu przy boocie; ramka `22 04` + flags bit3 (od 0.2.0); ratio 1:1 potwierdzony, **OFFSET = Vf diody DO KALIBRACJI** | [Z] FW; Vf pending |

### Pinout uzywany przez FW
| Pin | Funkcja | Konfiguracja |
|---|---|---|
| P0.05 | I2C SDA (bit-bang) | open-drain S0D1 + pullup wew. |
| P0.06 | I2C SCL (bit-bang) | open-drain S0D1 + pullup wew. |
| P0.09 | LSM INT1 | input NOPULL (nieuzywany w strategii poll; wyjscie na przyszly FIFO/DRDY) |
| P0.04 | SAADC AIN2 — **Vbat (bez dzielnika 2×; pomiar FW 0.3.3)** | SAADC od 0.2.0, skala 1/1 od 0.3.3 |
| P0.12 | prawdopodobny wylot dzielnika (drugie piete "node" z earlier notki — P0.04 i P0.12 to rozne node'y, rozwiazane pomiarem FW 0.3.3); wylacznie dzielnik — NIE CS flasha | nieuzywany przez FW |
| P0.25 | BTN do GND | input PULLUP, active-low; sense dla wybudzenia z SYSTEMOFF |
| P0.28 | LED | output, active-low |

Uwaga: P0.09/P0.10 to G-klasa INT (aktywnie-low wg schematu) — polaryzacja niezweryfikowana [?].
Uwaga: notka IMU „SA0=P0.04 -> VDD" — POTWIERDZONE dwoma pomiarami: WHO_AM_I=0x6A (SA0
wysoki; przy dzielniku ~1.65V bylby 0x68) + pomiar baterii 0.3.3 (AIN2 ~ Vbat). P0.04 = Vbat
[?] — legenda plyty vs FW: FW ma racje (dwa niezalezne pomiary).

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
INT1 DRDY (LSM6DSL INT1_CTRL=DRDY_XL) -> GPIOTE ISR (timestamp DWT) 
  -> petla glowna: lsm6dsl_read_motion() [TWIM 400kHz, fallback BB I2C, 12B z 0x22]
  -> ramka 14B [0x22,0x00 | gyro6 | acc6] -> ring[16] (drop najnowszej przy pelnym)
     -> ble_nus_data_send (14B/notify) -> PWA (WebBLE)
```

### Kolekcja (od 0.3.1)
- **DRDY**: INT1 -> GPIOTE, timestamp w ISR; odczyt w petli glownej. Auto-probe polaryzacji
  przy boocie (rising -> falling, zliczanie krawedzi w 100ms, ~10 @104Hz) — dziala bez LA;
  brak krawedzi => fallback polling 9ms (app_timer) + watchdog runtime (brak probki > 30ms
  => odczyt zapasowy, licznik `drdy_fallbacks`).
- **dt = t[n]-t[n-1]** z timestampow DRDY (TIMER1 @1MHz — **DWT->CYCCNT nie istnieje na nRF52810**); clamp 4-40ms (`dt_faults`), gap > 60ms (RTC1) => twardy ZUPT + dt nominalny; diag: `dt min/avg/max us`.
- **memcmp dup-guard = wylacznie diagnostyka** w trybie DRDY (probke identyfikuje DRDY,
  P0 audyt 2026-08-30); w trybie polling duplikat nadal nie wchodzi do calkowania.
- **I2C**: TWIM0 @400kHz (DMA, ~40us, ~0 CPU) z fallbackiem bit-bang (init, bus-clear,
  fault-recovery, licznik `twim_faults`, ban po 3 faultach z rzedu).
- **Timebase pomiarow**: TIMER1 @1MHz 16-bit (wrap-safe uint16, okno 65.5ms); gap-detect z RTC1.
  DWT->CYCCNT NIE istnieje na nRF52810 (0.3.3: wszystkie timingi = 0).

---

## 3. Konfiguracja LSM6DSL (kontrakt rejestrów)

Wartosci WYLACZNIE z tabeli datasheet (D-017); readback w RTT przy boocie.

| Rejestr | Wartosc | Znaczenie |
|---|---|---|
| CTRL1_XL (0x10) | 0x44 | ODR 104Hz (bity 7:4=0100) + FS_XL=01=**16g** (tabela NIEMONOTONICZNA: 00=2g,01=16g,10=4g,11=8g) |
| CTRL2_G (0x11) | 0x4C | ODR 104Hz + FS_G=11=**2000dps** (monotoniczna) |
| CTRL3_C (0x12) | 0x0C | BDU(bit3) + IF_INC(bit2). 0.1.0 i wcześniejsze: 0x44 = H_LACTIVE+IF_INC — **BDU nie ustawiony** (audyt 2026-08-30, readback c3 od 0.1.1) |
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
           flags: bit0 moving, bit1 rest, bit2 g-estimated (0.3.0), bit3 low-battery (od 0.3.2 snapshot przy próbce, nie OR przy wysyłce), bit4 g-forced = kalibracja g wymuszona po ~5s bez bezruchu → początkowe serie niepewne (0.3.2); vel+flags = snapshot VBT z chwili próbki
```

**Semantyka seq (0.1.0, plan 027 K4):** licznik każdej próbki IMU przy aktywnym połączeniu —
również dropniętej przy pełnym ringu. Luki w seq są OCZEKIWANE i informacyjne (realny drop
przed BUFOR), nie błędem transportu. Konsument (Triki_G pushDecoded) agreguje luki jako seqGaps.
Brak połączenia BLE = brak inkrementacji (to "nie zbieramy", nie drop).
- Host: acc_si = n / 2048 * 9.80665 [m/s^2]; gyro_dps = n / 16.4.
- PWA eksportuje SI — |a|~9.81 w spoczynku jest POPRAWNE (D-017 pkt 5).
- Zmiana layoutu/skali = decyzja cross-project z koordynacja Triki_G (D-019). Wire v1 do wycofania w 1.0.0 (D-021).

### 5.2 Ramka statusowa baterii (0.2.0)

```
Ramka 4B: [0x22][0x04] [mv_l mv_h]     bat u16LE [mV]; 0 = pomiar niemozliwy
Zadanie: RX `20 17` (1 zadanie = 1 ramka); FW cache'uje pomiar z ticku 1s (fallback: pomiar na zadanie).
flags v2 (ramka 19B): bit3 = low-battery (< 2400 mV); bity 0-2 = VBT bez zmian.
```

- Skala node->Vbat: `SCALE_NUM=1, DEN=1` (od 0.3.3) — pierwszy odczyt HW pokazal 6595mV
  przy skali 2/1 i realnych 3.308V => AIN2 = Vbat bez dzielnika 2× (wczesniejszy zapis
  "dzielnik potwierdzony plyta" byl pomiarem zlego punktu; P0.04 ≠ P0.12).
- **DO KALIBRACJI: OFFSET_MV** — porownaj FW `22 04` z miernikiem na baterii:
  `OFFSET_MV = Vbat_true − FW_mv` (kryterium F5: ±50 mV). Aktualnie roznica 10.5 mV
  (3308 vs 3297.5 po korekcie skali) => w praktyce sciezka bez istotnego Vf; OFFSET=0.
  Jesli miernik pokaze inny stosunek niz 1:1 (nie roznice offsetowa) — zaktualizowac NUM/DEN.

---

## 6. Power / sleep

- Sleep: **300s bez polaczenia** (app_timer 1s, flaga g_go_sleep, SYSTEMOFF z petli glownej — nie z kontekstu SWI).
- Wybudzenie: BTN sense -> reset -> boot blink -> adv.
- BTN w trybie czuwania: 3 wcisniecia = 2x mrug + reset licznika sleep (od 0.1.1: edge-detect + debounc 27ms w poll handler; wczesniej pojedyncze wcisniecie liczilo sie jako 3).
- Pobor pradu: NIE mierzony (roadmap O-013 pkt 4).

---

## 7. Error handling / diagnostyka

- APP_ERROR_CHECK = fail-stop -> fault handler: RTT kod+linia, LED SOS (3x40/40 + 1s) w petli.
- adv_start fail -> petla SOS.
- imu_init: 5 prob x (WHO_AM_I 3x z bus-clear + SW reset); fail -> "IMU DEAD" w RTT, FW i tak startuje BLE.
- RTT diag pod `TRIKIG_RTT_DIAG=1` (Makefile default ON): S1..S5, FW tag `FW=v0.x.y c1=44 c2=4C c3=0C`, blad APP_ERROR. Produkcja: `make release` = flaga OFF = pelna cisza (FW tag tez).

## 8. Budowanie i flash

- Host: Ubuntu 104 `~/trikig-fw/triki/` (SOURCE OF TRUTH), GCC 13 systemowy, SDK `/home/ubuntu/nrf5sdk`.
- Build: `make -j4` -> `_build/nrf52810_xxaa.hex`. v21: text=23532 data=140 bss=3348.
- Build produkcyjny (RTT OFF): `make release` (od 0.1.1).
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

## 10.5 Modul VBT (v0.3.0, side-band O-012; gravity tracking wg planu VBT C2-C5)

**Cel:** velocity barbell na kapslu (velocity-based training) bez zmian w wire.

| Element | Detal |
|---|---|
| Wejscie | gyro+acc 12B z OUT 0x22, odczyt w main loop wyzwalany DRDY (INT1 -> GPIOTE, timestamp w ISR); TWIM 400kHz + fallback bb; dt = t[n]-t[n-1] (clamp 4-40ms, `dt_faults`, gap>100ms => twardy ZUPT); fallback polling 9ms |
| Model | gravity estimator (filtr komplementarny): propagacja gyro `dg = -(w x g)*dt` co ramke (q16.16 x q8.8, zaokraglana), korekcja ACC gated (**|w| < 15 dps** — gate NIE moze byc ponizej spec biasu gyro ±5 dps; 0.3.4 lekcja z loga: bias 3dps > gate 2dps => rampa do clampa — repro `tools/vbt_offline bias`) I innowacja < 1.0 m/s^2 + powolny leak 1/2048 zawsze (net dead-lockow), renormalizacja do g; `lin = LPF(acc - g_est)` (JEDNO LPF na roznicy); `a_move = dot(lin, axis)`; `v += a_move*dt`; os ruchu default X barbell (vbt_set_axis) |
| Bezruch | `||lin|| < 0.3 m/s^2` przez 8 ramek — detektor 1. rzedu (stara norma \|\|a\|-g\| byla 2. rzedu: dev ~ a^2/2g, slepa na wolne pushy); fallback: wymuszona kalibracja g z LPF po ~5s bez bezruchu (TRIKIG_VBT_BIAS_FORCE_FRAMES) |
| ZUPT | decay 1/32/ramke przy bezruchu (tau ~0.31s) z min-krokiem 1 q8.8 (decay stenal przy \|v\| < 125 mm/s) |
| Progi | clamp v 15.6 m/s; clamp normy (audyt 2026-08-30) |
| Arytmetyka | fixed-point q8.8/q16.16 (brak FPU); 2x isqrt/ramke (rest + renorm) |
| API | vbt_reset() po imu_init; vbt_on_frame(raw12, dt_q16); vbt_set_axis(q12[3]); vbt_velocity_mms() [mm/s]; vbt_moving(); vbt_flags() (bit2 = g-estimated) |
| Diag | RTT ~1s: `VBT v=... mv=... dup=...` + `DIAG drdy=... smpl/dup/rdrop/gap/bdrop | dtf/fb/twi/sadc | dt min/avg/max | max acq/dsp/g/lin/v/ble us` (pod TRIKIG_RTT_DIAG); profil DSP gravity/linear/velocity us pod TRIKIG_VBT_PROFILE |
| Walidacja | harness offline `tools/vbt_offline` (5 scenariuszy syntetycznych PASS: rest60/rot/rot_move/rep/rep_soft); znane ograniczenie: wander ~±0.4 m/s przy jednoczesnej rotacji+oscylacji (adversarial, samolimitujacy, powrot do 0 po ustaniu) |
| Ograniczenia | korekcja g tylko quasi-statyka => dryf gyro-bias przy dlugim trzymaniu (typ. 40mdps => ~2.4deg/min); os stalych w ukladzie kapsla; walidacja terenowa vs Triki_G/PWA przed produkcyjnym uzyciem |
| Expose | API wewnetrzne; pole vel/flags wire v2 bez zmian |

---
## 10. Znane ograniczenia (stan 0.3.1)

1. Duplikaty probek — ZROBIONE 0.3.1 (DRDY identyfikuje probki; memcmp = tylko diagnostyka; w fallback polling dup-guard nadal aktywny).
2. Brak backpressure: NRF_ERROR_RESOURCES = drop ramki (brak buforowania na conn interval) [O-013 pkt 2].
3. Pomiar baterii — ZROBIONE 0.2.0/0.3.3 (SAADC AIN2 = Vbat, skala 1/1 — wczesniejsza skala 2/1 miala nieistniejacy dzielnik; ramka `22 04` na `20 17`, flags bit3; dokladnosc ~0.3% vs miernik); OFFSET_MV=0 (roznica 10.5mV — pomijalna).
4. Brak watchdog — ZROBIONE v0.0.27 (WDT 12s, covers boot).
5. BB I2C CPU-heavy — ZROBIONE 0.3.1 dla path danych (TWIM 400kHz + DMA; bb zostaje dla init/bus-clear/fault-recovery).
6. FIFO LSM6DSL — ODROCZONE do decyzji ODR >104 Hz (przy 104Hz DRDY+ring16 wystarcza; bez LA bitfields FIFO niezweryfikowane [AN4650, D-016]).
7. INT1 — **otwarte [HW]**: probe nie widzi krawedzi na P0.09 w zadnej polaryzacji (log 0.3.3/0.3.4: `drdy=0`) => DRDY prawdopodobnie NIE jest na P0.09 (SPEC 1 [?]). Fallback polling dziala (98 smpl/s, dup-guard). Do zrobienia: zmierzyc miernikiem/LA gdzie fizycznie jest DRDY -> poprawic PIN_IMU_INT1 lub notke -> probe zamieni sie w stala polaryzacje.
10. TWIM vs D-016 (audyt A): D-016 (bb_i2c.h) mówił "TWIM0 nie działa na tym sprzęcie" — C8 przywraca TWIM z fallbackiem bb i banem po 3 faultach z rzędu. ROZSTRZYZGA flash v0.3.2: `twim_faults`=0 → D-016 nieaktualne; `twi` rośnie i RTT pokaże "TWIM banned" → żyjemy z bb. Do weryfikacji PRZED produkcją.
11. SAADC fail był cichy (audyt F/G) — od 0.3.2 licznik `sadc` w diag + guard sum==0; OFFSET_MV nadal = 0 (DO KALIBRACJI na egzemplarzu, SPEC 5.2).
12. `bdrop` = 100% w logu 0.3.3 => prawdopodobnie klient bez subskrypcji CCCD (NRF_ERROR_INVALID_STATE); od 0.3.4 FW loguje kod pierwszego bledu (`BLE send err=0x..`, 8 = INVALID_STATE). Do potwierdzenia z subskrypcja PWA — bdrop ma byc ~0 przy streamie.
13. DWT->CYCCNT nie istnieje na nRF52810 — czas: TIMER1 @1MHz; watchdog DRDY 30ms i dt znów zywe od 0.3.4.
8. m_stream_on zawsze true po starcie (init-komenda tylko potwierdza).
9. v21/v22 bez logow PWA i testow terenowych (produkcja pozostaje v19; v21 boot zielony).

## 11. Roadmap rozwoju (po D-019: Triki_G primary, swoboda hardware)

1. Test v21/v22 (kryteria: sekcja 9) -> promocja produkcji; walidacja VBT vs Triki_G.
2. **Wire v2 (przelaczalny, koordynacja z Triki_G):** FW-side sample counter / timestamp ms (Android timestampy burstowe: dt 0..51ms), velocity (juz liczone w v22), bateria; tryb legacy 14B zostaje. MECHANIZM (decyzja Rafała 2026-08-29): rozpoznanie wersji przez KOMENDĘ RX po connect (aplikacja przełącza tryb), bez osobnej charakterystyki. Propozycja formatu (do ustalenia z Triki_G przy implementacji): `20 11 01` = wire v2 on, `20 11 00` = legacy, `20 12` = FW info (wersja/wiecej); komenda FW_INFO przed wlaczeniem v2 jako handshake.
3. **Komendy RX (kontrola sensora z aplikacji):** stream on/off, ODR/skale, sleep-now, FW info — rozszerzenie handlera 0x20 0x10.
4. Send-path: backpressure + licznik dropow (jako telemetria dla Triki_G).
5. Sleep + pomiar pradu; SAADC bateria — bateria ZROBIONE 0.2.0 (pomiar pradu pending).
6. Watchdog + nieblokujacy error-path.
7. RTT OFF build produkcyjny (TRIKIG_RTT_DIAG=0).
8. **Offline-record na MX25R8035F (1MB):** nagrywanie serii bez telefonu, sync przez Triki_G — pelna swoboda treningowa.
9. (opcjonalnie, po LA) TWIM 400kHz + FIFO + INT1.
10. Velocity/Kalman v2 na kapslu (O-012/Triki O-046) — po walidacji VBT v1.
