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
| P0.09 | **LSM INT1 / SLEEP_CHANGE** (od 0.4.0: activity/inactivity → IDLE-CONNECTED, sekcja 6.1; GPIOTE TOGGLE + readback WAKE_UP_SRC) | [P] pomiar plyty 2026-08-30 |
| P0.10 | **LSM INT2/DRDY** — DRDY_XL przez INT2_CTRL (0x0E), GPIOTE, timestamp probek | [P] pomiar plyty 2026-08-30 |
| P0.04 | SAADC AIN2 — **Vbat (bez dzielnika 2×; pomiar FW 0.3.3)** | SAADC od 0.2.0, skala 1/1 od 0.3.3 |
| P0.12 | prawdopodobny wylot dzielnika (drugie piete "node" z earlier notki — P0.04 i P0.12 to rozne node'y, rozwiazane pomiarem FW 0.3.3); wylacznie dzielnik — NIE CS flasha | nieuzywany przez FW |
| P0.25 | BTN do GND | input PULLUP, active-low; sense dla wybudzenia z SYSTEMOFF |
| P0.28 | LED | output, active-low |

Uwaga: P0.10 = LSM INT2/DRDY [P]; polaryzacja auto-probe (LOTOHI/HITOLO) — wynik w RTT.
Uwaga: notka IMU „SA0=P0.04 -> VDD" — POTWIERDZONE dwoma pomiarami: WHO_AM_I=0x6A (SA0
wysoki; przy dzielniku ~1.65V bylby 0x68) + pomiar baterii 0.3.3 (AIN2 ~ Vbat). P0.04 = Vbat
[?] — legenda plyty vs FW: FW ma racje (dwa niezalezne pomiary).

---

## 2. Architektura (0.3.x)

Bare-metal + nRF5 SDK 17 (bez RTOS). 7 modulow + main:

```
main.c              boot (WDT, DRDY probe, BLE stack/timers), petla glowna
                    (akwizycja DRDY -> VBT -> ring, send, BTN, sleep)
trikig_bb_i2c.c/h   bit-bang I2C master ~250kHz (init/bus-clear/fault-recovery)
trikig_lsm6dsl.c/h  init (WHO_AM_I retry, SW reset, config readback) + read OUT 0x22
                    (TWIM 400kHz z fallbackiem bb) + DRDY enable (INT2_CTRL)
trikig_diag.c/h     instrumentacja: liczniki dropow, dt min/avg/max, timingi TIMER1
trikig_vbt.c/h      VBT DSP: gravity tracking, movement-axis velocity (patrz SPEC 10)
trikig_batt.c/h     SAADC bateria CR2032 (SPEC 5.2)
trikig_board.c/h    LED/BTN/SYSTEMOFF + rtt_diag_printf (makro pod TRIKIG_RTT_DIAG)
```

### Przeplyw danych (jeden kierunek, SPSC)
```
INT2 DRDY (LSM6DSL INT2_CTRL=DRDY_XL, P0.10) -> GPIOTE ISR (timestamp TIMER1 @1MHz)
  -> petla glowna: lsm6dsl_read_motion() [TWIM 400kHz, fallback BB I2C, 12B z 0x22]
  -> VBT on-frame (dt = t[n]-t[n-1]) -> ramka v1 14B / v2 19B -> ring[16]
     -> ble_nus_data_send -> PWA (WebBLE)
  fallback (watchdog 30ms): poll 9ms z dup-guardem
IDLE-CONNECTED (0.4.0, sekcja 6.1): INT1 SLEEP_CHANGE (P0.09, TOGGLE) -> readback
  WAKE_UP_SRC -> stan; DRDY/dane dalej @12.5Hz, dt z RTC1, watchdog 160ms
```

### Kolekcja (od 0.3.1)
- **DRDY**: INT2 (P0.10) -> GPIOTE, timestamp w ISR, odczyt w petli glownej (0.3.11;
  probe z drain-read — patrz SPEC 10.7). Fallback: polling 9ms + watchdog 30ms
  (`drdy_fallbacks`). UWAGA HW: DRDY z BDU zalega HIGH bez odczytow OUT — kazdy test
  INT musi czytac dane w oknie pomiaru.
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
| TAP_CFG (0x58) | 0xE1 (od 0.4.0) | INTERRUPTS_ENABLE(bit7) + **INACT_EN=11** (bity[6:5]: activity/inactivity) + LIR(bit0) — zrodlo IDLE-CONNECTED (sekcja 6) |
| WAKE_UP_THS (0x5B) | 0x01 (od 0.4.0) | WK_THS=1 — prog budzenia; 1 LSB = FS_XL/2^6 = **250mg @ FS 16g**. [P] 2026-09-03: klasa ruchu dip/pullup 140-246mg p95 siedzi POD progiem — WK_THS to podloga przy FS16g, nie stroimy (fix = training mode, sekcja 6.2) |
| WAKE_UP_DUR (0x5C) | 0x06 (od 0.5.0; 0x04 w 0.4.0-0.4.3) | SLEEP_DUR=6 — **6s bezruchu => inactivity** (pole 4-bit, max 15s; 4s za krotkie na starty serii bez komendy [P] 2026-09-03) |
| MD1_CFG (0x5E) | 0x80 (od 0.4.0) | bit7 **INT1_SLEEP_CHANGE** — zmiana stanu activity/inactivity na INT1 (P0.09) |

Sensitivity: acc 2048 LSB/g, gyro 16.4 LSB/dps (= skale stocka, 1:1).

Uwaga 0.4.0: INACT_EN=11 uruchamia sprzetowy activity/inactivity — po SLEEP_DUR bezruchu
**HW sam** przechodzi w low-power (acc ODR 12.5Hz + gyro power-down), a po wykryciu ruchu
**sam wraca** do CTRL1/CTRL2 (DS6207 6.5.2; auto-restore potwierdzic readbackiem na HW —
[?] do pierwszego logu RTT). FW nigdy nie przelacza CTRL1/CTRL2 w runtime: reczne wartosci
0x10/0x40 z pierwotnej propozycji gubily FS=16g => 8x zmiana czulosci vs kontrakt wire
(2048 LSB/g). Stany w capability readback: WAKE_UP_SRC (0x1B) bit4 SLEEP_STATE_IA
(odczyt kasuje LIR => kolejna zmiana daje nowy poziom na INT1).

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
| Komendy RX | `20 11 0x` wire v1/v2; `20 12` FW info; `20 15 0x` stream on/off; `20 16` sleep-now; `20 17` bateria; **`20 18 0x` training mode (od 0.5.0: 01=ON — IDLE wylaczony na czas serii, 00=OFF; auto-OFF przy disconnect — sekcja 6.2)** |

Konsumenci: **TYLKO Triki_G** (D-019/D-021 — PWA/WebBLE wycofane calkowicie, nie sa wazonym klientem; historyczne logi PWA 2026-08-29 = dowoly testowe).

---

## 5. KONTRAKT WIRE (zamrozony, D-017 pkt 6)

```
Ramka 14B: [0x22][0x00] [gX_l gX_h gY_l gY_h gZ_l gZ_h aX_l aX_h aY_l aY_h aZ_l aZ_h]
           header 2B    | gyro FIRST (6B)                  | acc (6B)
           wszystkie wielobajtowe pola i16 LE (LSB pierwszy), raw z sensora bez konwersji
```

### 5.1 wire v2 (0.1.0, przełączalny komendą `20 11 01`, powrót `20 11 00`)

```
Ramka 19B: [0x22][0x01] [seq_l seq_h] [gyro6] [acc6] [vel_l vel_h] [flags]
           header 2B  | seq u16LE | gyro FIRST (6B) | acc (6B) | vel i16LE mm/s | flags u8
           flags (u8): bit0 moving, bit1 rest, bit2 g-estimated (0.3.0), bit3 low-battery (0.3.2, snapshot przy próbce), bit4 g-forced — kalibracja g wymuszona po ~5s bez bezruchu → początkowe serie niepewne (0.3.2); bity 5-7 zarezerwowane. vel+flags = snapshot VBT z chwili próbki
```

**Semantyka seq (0.1.0, plan 027 K4):** licznik każdej próbki IMU przy aktywnym połączeniu —
również dropniętej przy pełnym ringu. Luki w seq są OCZEKIWANE i informacyjne (realny drop
przed BUFOR), nie błędem transportu. Konsument (Triki_G pushDecoded) agreguje luki jako seqGaps.
Brak połączenia BLE = brak inkrementacji (to "nie zbieramy", nie drop).
**IDLE-CONNECTED (od 0.4.0):** w trybie idle (sekcja 6) stream biegnie dalej @12.5Hz —
seq CIĄGŁY (bez luk! licznik liczy każdą próbkę), zmienia się tylko TEMPO (104→12.5/s).
Host widzi wolniejszy strumień + flags bit1 (rest) — to NIE jest drop i NIE wymaga
żadnej zmiany parsera; ewentualne luki przy przejściu ODR są pojedyncze i informacyjne.
- Host: acc_si = n / 2048 * 9.80665 [m/s^2]; gyro_dps = n / 16.4.
- PWA eksportuje SI — |a|~9.81 w spoczynku jest POPRAWNE (D-017 pkt 5).
- Zmiana layoutu/skali = decyzja cross-project z koordynacja Triki_G (D-019). Wire v1 do wycofania w 1.0.0 (D-021).

### 5.2 Ramka statusowa baterii (0.2.0)

```
Ramka 4B: [0x22][0x04] [mv_l mv_h]     bat u16LE [mV]; 0 = pomiar niemozliwy
Zadanie: RX `20 17` (1 zadanie = 1 ramka); FW cache'uje pomiar z ticku 1s (fallback: pomiar na zadanie).
flags v2 (ramka 19B): bit3 = low-battery (< 2400 mV, snapshot przy próbce); bity 0-4 = VBT (SPEC 10).
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

### 6.1 IDLE-CONNECTED (od 0.4.0) — low-power w trakcie sesji BLE

**[P] ZWALIDOWANE NA SPRZECIE 2026-09-01 (0.4.1, log nRF Connect 13:14):** tempo
ACTIVE 103.9/s → IDLE 13.0/s (12.5Hz HW LP) → powrot 103.7/s po ruchu (auto-restore
ODR potwierdzony — DS6207 6.5.2); conn params 15ms/lat0 ↔ 150ms/lat4 w rytmie
przejsc S6 (Android zaakceptowal); sesja 70s / 5159 ramek bez drop.

Stan maszyny: **ACTIVE** (jak dotychczas) <-> **IDLE-CONNECTED** (HW inactivity).

| Element | ACTIVE | IDLE-CONNECTED |
|---|---|---|
| IMU acc ODR | 104Hz (CTRL1_XL=0x44) | **12.5Hz low-power** (wymusza HW INACT_EN) |
| IMU gyro | 104Hz (CTRL2_G=0x4C) | **power-down** (wymusza HW; OUT zamrozone BDU) |
| DRDY INT2/P0.10 | wlaczony | **wlaczony** (stream dalej, patrz nizej) |
| INT1/P0.09 | nasluch SLEEP_CHANGE (TOGGLE) | j.w. |
| BLE stream | 19B @104/s | **19B @12.5/s** (seq ciagly, tylko wolniejszy) |
| Conn params | ppcp 7.5-15ms, lat 0 | **150ms, latency 4** (best-effort) |
| VBT | pelny | **velocity = 0 by design** (gyro zamrozone — integracja bez gyro = phantom pod rotacja, 0.4.3); rest/flags liczone acc-only; propagacja gyro + nauka biasu OFF |
| Wejscie | activity (INT1 + readback WAKE_UP_SRC) | **6s bezruchu** (SLEEP_DUR od 0.5.0, WK_THS=250mg) |
| Wyjscie | 4s bezruchu => inactivity | **ruch > prog => activity** (HW auto-restore ODR) |

Zasady:
- Przejsciami steruje **HW** (INACT_EN); FW synchronizuje sie readbackiem WAKE_UP_SRC
  przy kazdym eventcie INT1 i przy connect (D-017: nie ufac polaryzacji krawedzi).
- DRDY zostaje wlaczony w IDLE — stream 19B biegnie dalej @12.5Hz; BLE zyje bez
  keep-alive (SoftDevice utrzymuje lacze pustymi conn eventami). Wylaczenie DRDY
  odrzucone: BDU trzymaloby linie HIGH bez odczytow (fneg 0.3.11).
| Watchdog DRDY w IDLE: **160ms na RTC1** (okno TIMER1 16-bit 65.5ms nie miesci
  okresu 80ms; RTC1 @16384Hz — patrz 11.14(g)); dt w IDLE liczony z RTC1
  (ticks*4 = q16.16 dokladnie), clamps 40-120ms, gap 200ms => twardy ZUPT.
- Conn params przez `ble_conn_params_change_conn_params` (modul SDK sam pilnuje
  negocjacji z budzetem prob; odmowa hosta = bezszkodowa — lacze chodzi na tym,
  co dal central). Licznik diag: `idle_cp_fail`.
- IDLE aktywne tylko przy potwierdzonym DRDY (drdy_mode!=0); probe P0.10 wygrywa
  przed rejestracja INT1 (bez konfliktu kanalow GPIOTE).
- Disconnect w IDLE: flaga w dol, **IMU zostaje w low-power HW** (kapsel lezy —
  oszczednosc sluszna takze bez polaczenia); sleep 300s bez zmian. Reconnect:
  sync stanu z readback przy CONNECTED.
- Wake latency [?]: INT1 @12.5Hz (do 80ms) + gyro turn-on (~70-80ms po power-down)
  => ~150ms slabego okna na starcie ruchu; VBT pokrywa gap-ZUPT; wplyw na repy
  do rozstrzygniecia kryterium F3 (>=90% count vs Triki_G).

### 6.2 TRAINING MODE (od 0.5.0) — IDLE wylaczony na czas serii

**Motywacja [P] (trening 2026-09-03, FW 0.4.3, dip 25kg / pullup 20kg):** klasa ruchu
repów dip/pullup = **140-246 mg p95** (p50 4-17 mg — holdy; max 429-758 mg, clank
2646 mg), tylko 1.4-4.8% probek aktywnych > WK_THS 250mg => HW NIE wybudza z
inactivity w dominujacej czesci serii (pickup/rep < 250mg). Efekt: app dostala tylko
**21-36% ramek** vs duration_sec (kazdy CSV = jeden ciagly burst 104Hz, reszta serii
bez ramek), SetResult raw=1/valid 0-1, base_mcv ~0.008, 2x hardRejected
physicallyImplausible (ujemne czasy repa). Offline V2 na tych samych CSV: R_PCA
0.49-0.97, 1-5 kandydatow. Nagrania NIE nadaja sie do walidacji rep-licznika;
diagnoza WK_THS potwierdzona.

| Element | Wartosc |
|---|---|
| Komenda RX | `20 18 01` = ON, `20 18 00` = OFF (param w 3. bajcie; brak param = ignoruj) |
| ON | `lsm6dsl_inactivity_enable(false)` (TAP_CFG/WK_THS/WK_DUR/MD1 = 0x00); readback `WAKE_UP_SRC`: sleeping => **`lsm6dsl_wake_force()`** (jawne CTRL1_XL/CTRL2_G = V19 + readback — INACT_EN=00 nie jest w DS6207 opisane jako wybudzenie) => `idle_connected_set(false)`; **`vbt_reset()`** (czysta kalibracja na start serii); conn params wracaja do ppcp (7.5-15ms) |
| OFF | `lsm6dsl_inactivity_enable(true)` — HW sam zejdzie w IDLE po SLEEP_DUR=6s bezruchu; VBT bez resetu |
| Auto-OFF | disconnect => zadanie OFF (safety: train ON bez polaczenia = 104Hz bez IDLE = strata pradu) |
| Kontekst | RX handler (SWI) tylko flaguje `g_train_req`; I2C/readback w main loop (wzorzec `g_conn_sync_req` — I2C zakazane w SWI) |
| Diag | RTT `S6 train ON (idle off)` / `S6 train OFF` / `S6 train wake c1=.. c2=..`; DIAG `train=` (0/1) |

Semantyka produktowa: host (Triki_G) wysyla `20 18 01` na start serii i `20 18 00`
po zakonczeniu (docelowo; na teraz recznie nRF Connect). Klienci bez komendy
(PWA/stock) zachowuja sie jak dotad (IDLE-CONNECTED z SLEEP_DUR=6s).

- Pobor pradu [P] (multimetr, 2026-09-01, FW 0.4.2/0.4.3): **~0.4mA w IDLE**
  (uspienie po bezczynnosci; estymata 0.3-0.5mA potwierdzona) i ~0.4mA przy
  advertising — obie sceny w tym samym rzedzie wielkosci (radio + IMU 12.5Hz LP +
  gyro PD + MCU sen). Szacunek zysku vs 0.3.x connected (~1.5-2mA): **~4-5x**.
  CR2032 220mAh / 0.4mA ~ 550h ~ 23 dni ciaglego IDLE. DO ZMIERZENIA: ACTIVE
  streaming (polaczenie+ruch), SYSTEMOFF (oczekiwane uA). Uwaga: 20s stabilizacji
  pomiaru nie odpowiada zadnemu przejsciu FW (IDLE wchodzi po 4s — SLEEP_DUR).

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

## 10. Modul VBT (v0.3.0, side-band O-012; gravity tracking wg planu VBT C2-C5)

**Cel:** velocity barbell na kapslu (velocity-based training) bez zmian w wire.

| Element | Detal |
|---|---|
| Wejscie | gyro+acc 12B z OUT 0x22, odczyt w main loop wyzwalany DRDY (INT2/P0.10 -> GPIOTE, timestamp w ISR); TWIM 400kHz + fallback bb; dt = t[n]-t[n-1] (clamp 4-40ms, `dt_faults`, gap>60ms (RTC1) => twardy ZUPT); fallback polling 9ms |
| Model | gravity estimator (filtr komplementarny): propagacja gyro `dg = -(w x g)*dt` co ramke (q16.16 x q8.8, zaokraglana), **na gyro skorygowanym o nauczony bias** (0.3.7); korekcja ACC gated (**|w-wbias| < 15 dps** — gate NIE moze byc ponizej spec biasu gyro ±5 dps; 0.3.4 lekcja) I innowacja < 1.0 m/s^2 + powolny leak 1/2048 zawsze (net dead-lockow), renormalizacja do g; **nauka biasu gyro w pelnym bezruchu** (s_rest I |w-wbias| < 5 dps, tau 0.6s; 0.3.7 uczyla w quasi-rest i pochlaniala wolne rotacje => wbias zatruty => rampa 15054 — log 22:22, repro `slowrot`; 0.3.9 fix); `lin = LPF(acc - g_est)` (JEDNO LPF na roznicy); `a_move = dot(lin, axis)`; `v += a_move*dt`; os ruchu default X barbell (vbt_set_axis) |
| Bezruch | `||lin|| < 0.3 m/s^2` przez 8 ramek — detektor 1. rzedu (stara norma \|\|a\|-g\| byla 2. rzedu: dev ~ a^2/2g, slepa na wolne pushy); fallback: wymuszona kalibracja g z LPF po ~5s bez bezruchu (TRIKIG_VBT_BIAS_FORCE_FRAMES) |
| ZUPT | decay 1/32/ramke przy bezruchu (tau ~0.31s) z min-krokiem 1 q8.8 (decay stenal przy \|v\| < 125 mm/s) |
| Progi | clamp v 15.6 m/s; clamp normy (audyt 2026-08-30) |
| Arytmetyka | fixed-point q8.8/q16.16 (brak FPU); 1x isqrt/ramke (renorm; detektor rest i gate innowacji na kwadratach) |
| API | vbt_reset() po imu_init; vbt_on_frame(raw12, dt_q16); vbt_set_axis(q12[3]); vbt_velocity_mms() [mm/s]; vbt_moving(); vbt_flags() (bit2 = g-estimated) |
| Diag | RTT ~1s: `VBT v=... mv=... dup=...` + `DIAG drdy=... smpl/dup/rdrop/gap/bdrop | dtf/fb/twi/sadc | dt min/avg/max | max acq/dsp/g/lin/v/ble us` (pod TRIKIG_RTT_DIAG); profil DSP gravity/linear/velocity us pod TRIKIG_VBT_PROFILE |
| Walidacja | harness offline `tools/vbt_offline` (**6 scenariuszy PASS**) + replay realnych logów nRF Connect (`nrflog2raw.py | vbt_offline stdin`): rest 3.5s ✓, ruchy v1 22s (max 835 mm/s, koniec v=0) ✓, ruchy wire v2 14s (porownanie FW-vs-replay: FW stuck ~3000 z bledem g z historii -> 0.3.7 nauka biasu: replay ±675, koniec +238) ✓ + **[P] walidacja RTT 0.3.4: bursty ruchow na urzadzeniu — v ±100-500 mm/s, powrot do 0**; znane ograniczenie: wander ~±0.4 m/s przy jednoczesnej rotacji+oscylacji (adversarial, samolimitujacy) |
| Ograniczenia | korekcja g tylko quasi-statyka => dryf gyro-bias przy dlugim trzymaniu (typ. 40mdps => ~2.4deg/min); os stalych w ukladzie kapsla; walidacja terenowa vs Triki_G/PWA przed produkcyjnym uzyciem |
| Expose | API wewnetrzne; pole vel/flags wire v2 bez zmian |

---
## 11. Znane ograniczenia (stan 0.4.0)

1. Duplikaty probek — ZROBIONE 0.3.1 (DRDY identyfikuje probki; memcmp = tylko diagnostyka; w fallback polling dup-guard nadal aktywny).
2. Brak backpressure: NRF_ERROR_RESOURCES = drop ramki (brak buforowania na conn interval) [O-013 pkt 2].
3. Pomiar baterii — ZROBIONE 0.2.0/0.3.3 (SAADC AIN2 = Vbat, skala 1/1 — wczesniejsza skala 2/1 miala nieistniejacy dzielnik; ramka `22 04` na `20 17`, flags bit3; dokladnosc ~0.3% vs miernik); OFFSET_MV=0 (roznica 10.5mV — pomijalna).
4. Brak watchdog — ZROBIONE v0.0.27 (WDT 12s, covers boot).
5. BB I2C CPU-heavy — ZROBIONE 0.3.1 dla path danych (TWIM 400kHz + DMA; bb zostaje dla init/bus-clear/fault-recovery).
6. FIFO LSM6DSL — ODROCZONE do decyzji ODR >104 Hz (przy 104Hz DRDY+ring16 wystarcza; bez LA bitfields FIFO niezweryfikowane [AN4650, D-016]).
7. DRDY/INT — **ZAMKNIETE 0.3.11 [P] — DZIALA END-TO-END**: INT1->P0.09, INT2->P0.10 (pomiar plyty); probe z drain-read (false-negative 0.3.1-0.3.10: DRDY z BDU zalega HIGH bez odczytow). Walidacja loga wire v2 22:54: tempo seq 103.93/s, **dup=0** (memcmp demobilizowany), dziury seq 1/2123, dt z timestampow, vel 0→-1149→0 bez dryfu, FW-vs-replay mediana 24 mm/s.
10. TWIM vs D-016 (audyt A): D-016 (bb_i2c.h) mówił "TWIM0 nie działa na tym sprzęcie" — C8 przywraca TWIM z fallbackiem bb i banem po 3 faultach z rzędu. ROZSTRZYZGA flash v0.3.2: `twim_faults`=0 → D-016 nieaktualne; `twi` rośnie i RTT pokaże "TWIM banned" → żyjemy z bb. Do weryfikacji PRZED produkcją.
11. SAADC fail był cichy (audyt F/G) — od 0.3.2 licznik `sadc` w diag + guard sum==0; OFFSET_MV nadal = 0 (DO KALIBRACJI na egzemplarzu, SPEC 5.2).
12. `bdrop` = 100% w logu 0.3.3 => prawdopodobnie klient bez subskrypcji CCCD (NRF_ERROR_INVALID_STATE); od 0.3.4 FW loguje kod pierwszego bledu (`BLE send err=0x..`, 8 = INVALID_STATE). Do potwierdzenia z subskrypcja PWA — bdrop ma byc ~0 przy streamie.
13. DWT->CYCCNT nie istnieje na nRF52810 — czas: TIMER1 @1MHz; watchdog DRDY 30ms i dt znów zywe od 0.3.4.
14. IDLE-CONNECTED (0.4.0/0.4.1/0.4.2): (a) WK_THS LSB @FS16g=250mg przyjeta z DS6207
    (FS/2^6) — [P] smoke 0.4.1: ruch powoli (<250mg) NIE wybudza z IDLE (velocity
    rampuje @12.5Hz do ~1100 mm/s, ZUPT gasi po powrocie do rest); WK_THS do strojenia
    na potrzeby UX; (b) auto-restore ODR po activity — POTWIERDZONY [P] 2026-09-01
    (tempo wraca 103.7/s); (c) wake latency ~150ms (INT1 @12.5Hz + gyro turn-on) —
    wplyw na detekcje repow rozstrzygnie F3; (d) zysk energetyczny [P] multimetr
    2026-09-01: **IDLE ~0.4mA** (estymata 0.3-0.5 potwierdzona; ~4-5x mniej niz
    0.3.x connected; CR2032 ~23 dni ciaglego IDLE); do zmierzenia: ACTIVE streaming,
    SYSTEMOFF (uA), PPK dla profilu; (e) przy polaczeniu central mogacy narzucic
    wlasne conn params — IDLE
    wtedy bez zysku radiowego (dziala dalej, tylko bez oszczednosci). VBT w IDLE nie
    liczy propagacji gyro (zamrozone OUT) — velocity w trakcie idle = 0, po activity
    lampa na 1 frame (dt nominal).
    (f) 0.4.1: runtime reg access musi byc TWIM-aware (reg_write_t/reg_read_t) — bb
    po nrfx_twim_enable nie steruje magistrala (latent bug od C8, ujawniony przez
    zapis inactivity; log smoke 0.4.0: cfg=00 MISMATCH).
    (g) 0.4.2 [P]: RTC1 przy app_timer2 = 16384Hz (PRESCALER=1, 61.035us/tick) —
    progi RTC-based musza liczyc sie z ta czestotliwoscia; DT_GAP_TICKS 1966 od
    0.3.4 to de facto 120ms (nie 60ms — komentarz bledny, ACTIVE nieczuly na roznice).
    (h) 0.4.2 [P]: cpfail +1/przejscie — readback WAKE_UP_SRC kasuje LIR => TOGGLE
    lapie deassert => drugi event z tym samym stanem => ponowna negocjacja cp = BUSY;
    idle_cp_apply gated na zmiane stanu (0.4.2) — POTWIERDZONY [P] (0.4.2 smoke:
    cpfail=0 przez 3 przejscia).
    (i) 0.4.3 [P]: klasa ruchu 30-250mg (prog rest VBT 0.3 m/s^2 ~ 30mg << WK_THS
    250mg @FS16g — podloga progu HW) integrowala velocity @12.5Hz z zamrozonym gyro
    (g_est nie sledzi rotacji) => phantom az do clampa 15625 mm/s (log smoke 0.4.2:
    v=12593->15625). FIX: velocity w IDLE = 0 BY DESIGN; realny trening >250mg
    wybudza HW => 104Hz => pelne VBT od zera. Conn params [P]: 150ms/lat4 OD
    CONNECT (sync przy connect) i w rytmie S6 (log 16:31).
    (j) 0.5.0 [P] FALSYFIKACJA (i): realny trening dip/pullup (2026-09-03) ma
    klasę 140-246mg p95 POD WK_THS 250mg (piki 429-758mg tylko 1.4-4.8% probek) =>
    HW NIE wybudza w serii (21-36% pokrycia ramek; reszta serii bez ramek —
    lacznie z IDLE 12.5Hz brak czegokolwiek, do weryfikacji strona app);
    WK_THS=1 to podloga LSB przy FS16g => strojenie progu odpada, fix = training
    mode `20 18` (sekcja 6.2) + SLEEP_DUR 4s->6s. CROSS-PROJECT (Triki): SetResult
    hardRejected physicallyImplausible z UJEMNYM czasem repa (medDur -28.7/-39.3s)
    przy braku danych = odpornosc pipeline'u, osobny watek.
8. m_stream_on zawsze true po starcie (init-komenda tylko potwierdza).
9. v21/v22 bez logow PWA i testow terenowych (produkcja pozostaje v19; v21 boot zielony).

## 12. Roadmap rozwoju (po D-019: Triki_G primary, swoboda hardware)

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
