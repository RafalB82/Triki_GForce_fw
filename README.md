# trikig-fw — firmware kapsla Triki GForce (nRF52810 + LSM6DSL)

Własny, otwarty firmware dla kapsla Triki (klon „zabka-triki”: nRF52810 + LSM6DSL + MX25R8035F).
Kompatybilny wstecz z protokołem stocka (NUS, ramka 14B), rozszerzony o wire v2 (seq + velocity + flags)
i komendy RX. Konsument docelowy: aplikacja **Triki_G** (Flutter, D-019/D-021).

## Stan

| Wersja | Status |
|---|---|
| **v19** (`trikig_triki_v19.hex`) | PRODUKCJA (end-to-end zweryfikowany: 104.0 Hz, dt 9.6 ms, skale OK) |
| **v0.0.26** (`firmware/hex/trikig_triki_v0.0.26.hex`) | wire v2 (F1) + komendy RX (F2); sha256 `ef677b0f…` |
| **v0.0.27** (`firmware/hex/trikig_triki_v0.0.27.hex`) | v0.0.26 + fixes z audytu 2026-08-29 (WDT 12s, BLE send one-shot/drop, static_assert V_CLAMP); sha256 `44eac963…` |
| **v0.1.0** (`firmware/hex/trikig_triki_v0.1.0.hex`) | plan wdrożenia K0-K8 (WDT covers boot, IMU init fail-fast, ring_slot_t + snapshot vel/flags przy poll, dup-guard VBT, seq = każda próbka — luki informacyjne, SPEC 5.1); sha256 `bb6e1b28…` |
| **v0.1.1** (branch `fixy`) | fixy audytu 2026-08-30: CTRL3_C 0x0C = BDU+IF_INC (0x44 ustawiał H_LACTIVE, brak BDU) + readback c3; CP delay w APP_TIMER_TICKS (jednostki app_timer2); BTN debounc+edge-detect (3 wciśnięcia wg SPEC 6); clamp isqrt (uint32 overflow @16g); app_timer_init przed BLE; licznik zadań `20 12`; `make release` (RTT OFF); runda 2: doc WDT 12s, sekcja krytyczna g_btn_presses, reg_write NACK-abort, bb_restart jawny SDA, VBT fallback kalibracji biasu ~5s; hex: do wybudowania z brancha |
| **v0.2.0** (F5 bateria) | v0.1.1 + SAADC CR2032 (AIN2/P0.04, dzielnik 100k/100k przez diodę, bez LDO; P0.12 wylacznie dzielnik — NIE CS flasha): ramka `22 04` na RX `20 17` (bat mV u16LE), flags v2 bit3 low-batt, pomiar 1s w sleep tick; ratio 1:1 potwierdzony, OFFSET Vf do kalibracji (SPEC 5.2); hex: pending build |
| **v0.3.0** (plan VBT C1-C6) | instrumentacja `trikig_diag` (liczniki dropów/okresu/timingi DWT, RTT `DIAG` ~1s); VBT: gravity tracking ACC+GYRO (propagacja gyro + korekcja ACC gated, zamiast biasu per-os), movement-axis (default X, `vbt_set_axis`), dt=1/ODR, detektor rest 1. rzędu `||lin||<0.3`, ZUPT τ0.31s z min-krokiem; harness offline `tools/vbt_offline` (5 scenariuszy PASS); wire v1/v2 nietknięte; build release: text=26392 bss=3020 (v0.2.0: 25464/2964); hex: `firmware/hex/trikig_triki_v0.3.0.hex` sha256 `b3dc2a43…` — flash/smoke PENDING |
| **v0.3.1** (Faza 2: akwizycja) | C7 DRDY: INT1→GPIOTE (timestamp w ISR, odczyt w main loop), auto-probe polaryzacji bez LA + fallback polling + watchdog 30ms; dt = t[n]−t[n−1] (clamp 4-40ms, `dt_faults`, gap>100ms→twardy ZUPT); memcmp = tylko diagnostyka w DRDY; C8 TWIM 400kHz hybryda (bb init/bus-clear, `twim_faults`); C9 ring 4→16; P2: profil DSP gravity/linear/velocity us + isqrt 3→1; release: text=29808 bss=3340; hex: `firmware/hex/trikig_triki_v0.3.1.hex` sha256 `cc527378…` — flash/smoke PENDING |
| **v0.3.4_rtt** (fixy z loga 0.3.3, RTT=1) — **[P] ZWALIDOWANY NA SPRZĘCIE** | bateria 3310 vs 3308mV (Δ2mV); velocity 0 przy leżącym (rampa wyeliminowana); timingi live: acq 515 / dsp 101 / g 88 / lin 18 / v 2 / ble 4 us (DSP ~1% CPU — bez potrzeby optymalizacji); **TWIM potwierdzony (`twi=0`, D-016 zamknięte)**; BLE err=0x8 potwierdza brak CCCD u klienta; otwarte: INT1 nie na P0.09 (probe nie widzi krawędzi → polling fallback; wymaga pomiaru HW) | **DWT->CYCCNT nie istnieje na nRF52810** → timebase TIMER1 @1MHz (timingi diag znów żywe, watchdog DRDY 30ms działa); gap-detect z RTC1 (>60ms → twardy ZUPT); **rampa velocity przy leżącym urządzeniu**: gyro bias ~3dps > gate 2dps → korekcja trwale zamknięta → repro `vbt_offline bias` (FAIL→PASS), gate 2→15dps + leak 1/2048; telemetria pierwszego błędu BLE send (bdrop=104/s w logu — podejrzenie braku CCCD); release: pending zamówienia; dev hex: `firmware/hex/trikig_triki_v0.3.4_rtt.hex` sha256 `fbf04012…` — flash/smoke PENDING |
| **v0.3.5_rtt** | DRDY na INT2 wg datasheet (pin 9 ukladu): register INT2_CTRL (0x0E) zamiast INT1_CTRL (0x0D) z readback; probe rozszerzona o oba kandydujace piny nRF P0.09/P0.10 x polaryzacja (`drdy_mode` 1-4); fallback polling bez zmian; hex sha256 `9409576d…` — flash/smoke PENDING |
| **v0.3.6_rtt** | LSM INT2/DRDY → P0.10 potwierdzone plyta [P] — probe probuje P0.10 najpierw, P0.09 fallback; SPEC 1/10 sync (P0.09 nieuzywany w tej wersji); hex sha256 `b998ef1b…` — flash/smoke PENDING |
| **v0.3.7_rtt** | nauka biasu gyro w quasi-bezruchu (\|\|lin\|\|<1.2, \|w−wbias\|<15dps, tau 0.6s) — dryf propagacji podtrzymywal blad g na progu gate'a innowacji → ZUPT nie gasil velocity (raczeta +250 mm/s/cykl); replay po fixie: v ±675, koniec +238; harness 6/6 PASS; hex sha256 `b054e354…` — flash/smoke PENDING |
| **v0.3.8_rtt** | fix probe DRDY: dedykowany licznik krawedzi w ISR (`s_probe_cnt`, bez pusha do FIFO) — FIFO timestampow wypelnialo sie w 100ms zanim licznik doszedl do progu 5 krawedzi (bug blokowal probe od 0.3.1); hex — flash/smoke PENDING |
| **v0.3.8_scan** | diagnostyka: `pin_scan()` wszystkich wolnych GPIO (60ms/pin, pull-down, GPIOTE hi-acc, DRDY wlaczone) pod katem krawedzi DRDY; `TRIKIG_PIN_SCAN ?= 0`; hex sha256 `c4407a51…` (tylko diagnostyka) |
| **v0.3.9_rtt** | REGRESJA 0.3.7: quasi-rest pochlanial wolne rotacje ≤15dps → wbias zatruty → propagacja nadrabiala w zla strone (rampa do 15054 mm/s); repro harness `slowrot`; fix: nauka TYLKO przy pelnym rest (s_rest) i \|w−wbias\|<5dps (wolne rotacje <5dps w rest self-heal); harness 7/7 PASS (+slowrot); hex sha256 `318c57cf…` — flash/smoke PENDING |
| **v0.3.9_scan (+fix)** | pin_scan nie wlaczal INT2_CTRL przed skanem → pusty wynik; fix: enable+readback na starcie skanu (sha `5b0b501c…`); v2: skan z DRDY wlaczonym na OBU pinach INT (INT1_CTRL 0x0D + INT2_CTRL 0x0E, readback) — nadal zero krawedzi (sha `5f6de938…`) |
| **v0.3.10_rtt** | (wniosek pozniej ODWOLANY 0.3.11) „DRDY zamkniete dla tego HW”: scan v2 (29 pinow, 60ms/pin) = zero krawedzi → probe domyslnie OFF, akwizycja polling 9ms (zwalidowana end-to-end); boot szybszy ~400ms |
| **v0.3.11_rtt** | **DRDY DZIALA — false-negative obalony [P]**: pomiar plyty INT1→P0.09, INT2→P0.10; root-cause: DRDY z BDU zalega HIGH dopoki nikt nie czyta rejestrow OUT, a scan/probe biegly przed startem pollingu → zero rosnacych krawedzi; fix: okna probe/scan z drain-read co 10ms (krawedz co ~9.6ms), probe domyslnie ON; walidacja end-to-end (log wire v2 22:54): tempo seq 103.93/s, dup=0, dziury 1/2123, vel 0→−1149→0 bez dryfu, FW-vs-replay mediana 24 mm/s — **Faza 1+2 planu VBT domknieta**; dev text=33328 bss=3900, hex sha256 `20b650ef…` — flash/smoke PENDING |
| **v0.4.0_rtt** (IDLE-CONNECTED, MINOR) | low-power w sesji BLE: HW activity/inactivity LSM6DSL (TAP_CFG INACT_EN=11+LIR, WK_THS=250mg @FS16g, SLEEP_DUR=4s, MD1_CFG bit7 SLEEP_CHANGE → INT1/P0.09 GPIOTE TOGGLE) — po 4s bezruchu **HW sam**: acc 12.5Hz LP + gyro power-down, activity → auto-restore (DS6207 6.5.2); FW: sync readbackiem WAKE_UP_SRC (event INT1 + connect), DRDY zostaje (stream 19B @12.5/s, seq ciagly — kontrakt 5.1 bez zmian bajtowych), watchdog DRDY 160ms na RTC1 (okno TIMER1 65.5ms < 80ms), dt z RTC1 (ticks×2=q16.16), gap 200ms, vbt_idle() wylacza propagacje gyro + nauke biasu (zamrozone OUT — klasa slowrot), conn params 150ms/lat4 przez ble_conn_params_change_conn_params (ppcp 7.5-15ms po activity, idle_cp_fail w diag); IDLE tylko przy DRDY (drdy_mode≠0); MINOR bo zmienia TEMPO strumienia (D-020); harness 7/7 PASS; dev text=34704 data=144 bss=3908 — **[P] smoke: boot ok, DRDY ok, ale inact cfg=00 MISMATCH** (bb po starcie TWIM nie steruje magistrala) → fix w 0.4.1 |
| **v0.4.1_rtt** | fix smoke 0.4.0: runtime reg access (inactivity + WAKE_UP_SRC) przez **reg_write_t/reg_read_t** (TWIM jak zyje, fault → uninit + fallback bb, ban po 3; wzorzec read_motion) — reg_write/reg_read (bb) po `nrfx_twim_enable` przejmujacym piny 5/6 pisaly w pustke; sync readback przy CONNECTED przeniesiony ze SWI (BLE handler) do main loop (`g_conn_sync_req`); boot-init bez zmian (bb przed TWIM = stuck-bus recovery); **[P] ZWALIDOWANY NA SPRZĘCIE (log nRF Connect 2026-09-01 13:14)**: inact cfg=E1/ths=01/dur=04/md1=80 z readbackiem, tempo ACTIVE 103.9/s → **IDLE 13.0/s** (12.5Hz HW LP, seq ciągły) → powrót 103.7/s po ruchu (**auto-restore ODR potwierdzony** — DS6207 6.5.2), conn params 15ms/lat0 ↔ 150ms/lat4 przejścia w rytm S6, sesja 70s/5159 ramek bez drop; harness 7/7; dev text=35024 data=144 bss=3908, hex `firmware/hex/trikig_triki_v0.4.1_rtt.hex` sha256 `9f5c775f…` — pozostało: RTT DIAG (fb/dup/cpfail) + PPK |
| **v0.3.3_rtt** (build deweloperski do walidacji, RTT=1) | jak v0.3.3, RTT ON (`S2 batt=...`, `DIAG ...` ~1s przy streamie); hex: `firmware/hex/trikig_triki_v0.3.3_rtt.hex` sha256 `51297942…` |
| **v0.3.3** (walidacja HW baterii) | pierwsza rzeczywista lektura: FW 6595mV @ skala 2/1 vs real 3.3080V → **AIN2/P0.04 = Vbat bez dzielnika 2×**, SCALE 1/1 (dokładność ~0.3%); wcześniejsze "dzielnik potwierdzony plytą" był pomiarem złego punktu (P0.04 ≠ P0.12; potwierdza też WHO_AM_I=0x6A → SA0 wysoki); SPEC 1/5.2/10 zaktualizowane; release: text=29936 bss=3340; hex: `firmware/hex/trikig_triki_v0.3.3.hex` sha256 `ad2023fc…` — flash/smoke PENDING |
| **v0.3.2** (audyt zewnętrzny) | A: ban TWIM po 3 faultach z rzędu (D-016 real? → bb do resetu; rozstrzyga `twim_faults` na sprzęcie); F/G: SAADC faults w diag + guard sum==0; H: low-batt snapshot przy próbce (K2 spójność); flags v2 **bit4 = g-forced** (kalibracja wymuszona w ruchu → host oznacza serie niepewne); IMU_ODR_HZ; CI: `vbt_offline` w GH Actions; release: text=29936 bss=3340; hex: `firmware/hex/trikig_triki_v0.3.2.hex` sha256 `0cfbc635…` (release, rebuild z HEAD — pierwotny hex w ea62315 byl omylkowo buildem dev; poprawione) — flash/smoke PENDING |

Semver: MAJOR=0 (beta), MINOR=protokół (wire v2 → 0.1.x docelowo), PATCH=iteracja. Tag FW w RTT + komenda `20 12`.

## Budowa od zera (odtwarzalne środowisko)

1. Toolchain + SDK: `./tools/setup_env_ubuntu.sh` (arm-none-eabi GCC 13.2.1 systemowy; nRF5 SDK 17.1.1 do rozpakowania w `~/nrf5sdk` — nie wchodzi do repo).
2. Build:
   ```bash
   cd triki
   make            # SDK_ROOT domyślnie ~/nrf5sdk (override: make SDK_ROOT=/sciezka)
   ```
   Wynik: `_build/nrf52810_xxaa.hex`. Build produkcyjny (RTT OFF): `make release` — **TYLKO na wyraźne zamówienie**.
   **Zasada (obowiązuje od 0.3.3):** domyślnie buduje się wariant DEV (RTT=1, `_rtt.hex`)
   do walidacji na sprzęcie; build produkcyjny/release hex powstaje wyłącznie na wyraźne
   żądanie (np. "zbuduj release", promocja produkcji). Release hex jest RTT-silent —
   JLink RTT pokaże "No control block found" (to oczekiwane, nie błąd wgrywania).
   UWAGA: buduj przez `cd triki && make` — `make -C triki` włącza print-directory, który
   zaśmieca generowany plik wejściowy linkera (`.in`) i wywala link (szablon SDK 17).
3. Weryfikacja odtworzenia: build referencyjny v0.0.26 → text=24620, bss=3396. Drobne różnice rozmiaru (toolchain) są OK; funcjonalna weryfikacja = FW tag w RTT + test streamu.

## Flash (kapsel)

- Debug: SWD pady 3V3/GND/nRESET/SWDIO/SWCLK (APPROTECT locked przy pierwszym flashu → `nrf52_recover` = full erase).
- **Kolejność: najpierw SoftDevice `s112_nrf52_7.2.0_softdevice.hex` (0x0), potem app (0x19000).** App-only po full erase = HardFault (pusty vector table).
- **AP lock najczęściej = SystemOFF**: najpierw wybudź (przycisk/zasilanie), dopiero potem `nrf52_recover` (recover w SystemOFF niszczy flash i wymusza full reflash).
- SoftDevice hex + konfiguracja flashera: mirror SMB `Triki_G/nrf/` (SDK licencja zabrania redystrybucji hexa w repo).
- targets `flash`/`flash_softdevice` w Makefile zakładają nrfjprog (J-Link); workflow Tigard/Pico+OpenOCD — docs/SPEC.md.

## Protokół (skrót; pełny kontrakt: docs/SPEC.md)

- **wire v1 (14B, domyślny):** `22 00 | gyro6 | acc6` i16LE; gyro FIRST; skale stocka (2048 LSB/g @16g, 16.4 LSB/dps @2000dps).
- **wire v2 (19B, przełączalny):** `22 01 | seq u16LE | gyro6 | acc6 | vel i16LE [mm/s] | flags u8`; flags: bit0 moving, bit1 rest, bit2 bias-calibrated; seq wrapuje co 65536.
- **komendy RX (prefiks `20`):** `20 10` init/stream-on (kompat stock), `20 11 nn` wire mode (01=v2, 00=v1), `20 12` → FW info `22 03 | maj | min | pat | mode` (6B), `20 15 nn` stream on/off, `20 16` sleep NOW (SYSTEMOFF; BTN wybudza).

## Struktura repo

```
triki/               źródła (main.c + moduły trikig_*) + Makefile + linker script
  config/            sdk_config.h (LFCLK=RC — kapsel nie ma kryształu 32k)
tools/               setup środowiska
tools/vbt_offline/   harness offline VBT (7 scenariuszy) + replay logów nRF Connect
docs/                SPEC.md (kontrakt wire + known-issues) + DSP_MAP.md (mapa DSP)
firmware/hex/        hexy wersjonowane ręcznie (`_rtt` = dev RTT=1; release na zamówienie)
.github/workflows/   CI: harness VBT przy push/PR (+ build FW gdy SDK dostępne)
```

## Narzędzia walidacyjne (od 0.3.x)

```bash
# regresja VBT bez sprzętu (host cc, kompiluje triki/trikig_vbt.c 1:1 z FW):
tools/vbt_offline/build.sh && tools/vbt_offline/vbt_offline all

# replay realnego loga nRF Connect (wire v1 14B lub v2 19B) przez FW VBT:
python3 tools/vbt_offline/nrflog2raw.py "Log 2026-08-30 22_54_29.txt" | tools/vbt_offline/vbt_offline stdin
# kolumny: frame;v_new_mm_s;v_old_mm_s(v0.2.0 ref);flags
```

## Procesowe (D-017)

- Bitfieldy rejestrów TYLKO z tabeli datasheet (LSM6DSL: FS_XL niemonotoniczna, FIFO_MODE w CTRL5[2:0]).
- FW tag w RTT obowiązkowy przy każdej wersji.
- Nie flashować app-only po full erase (SD first).
