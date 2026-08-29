# trikig-fw — firmware kapsla Triki GForce (nRF52810 + LSM6DSL)

Własny, otwarty firmware dla kapsla Triki (klon „zabka-triki”: nRF52810 + LSM6DSL + MX25R8035F).
Kompatybilny wstecz z protokołem stocka (NUS, ramka 14B), rozszerzony o wire v2 (seq + velocity + flags)
i komendy RX. Konsument docelowy: aplikacja **Triki_G** (Flutter, D-019/D-021).

## Stan

| Wersja | Status |
|---|---|
| **v19** (`trikig_triki_v19.hex`) | PRODUKCJA (end-to-end zweryfikowany: 104.0 Hz, dt 9.6 ms, skale OK) |
| **v0.0.26** (`firmware/hex/trikig_triki_v0.0.26.hex`) | wire v2 (F1) + komendy RX (F2); sha256 `ef677b0f…` |
| **v0.0.27** (`firmware/hex/trikig_triki_v0.0.27.hex`) | aktualny development: v0.0.26 + fixes z audytu 2026-08-29 (WDT 8s, BLE send one-shot/drop, static_assert V_CLAMP); sha256 `44eac963…` |

Semver: MAJOR=0 (beta), MINOR=protokół (wire v2 → 0.1.x docelowo), PATCH=iteracja. Tag FW w RTT + komenda `20 12`.

## Budowa od zera (odtwarzalne środowisko)

1. Toolchain + SDK: `./tools/setup_env_ubuntu.sh` (arm-none-eabi GCC 13.2.1 systemowy; nRF5 SDK 17.1.1 do rozpakowania w `~/nrf5sdk` — nie wchodzi do repo).
2. Build:
   ```bash
   cd triki
   make            # SDK_ROOT domyślnie ~/nrf5sdk (override: make SDK_ROOT=/sciezka)
   ```
   Wynik: `_build/nrf52810_xxaa.hex`.
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
docs/                SPEC.md (kontrakt wire + roadmapa DSP_MAP)
firmware/hex/        release hexy (wersjonowane ręcznie; build-artefakty nie committowane)
```

## Procesowe (D-017)

- Bitfieldy rejestrów TYLKO z tabeli datasheet (LSM6DSL: FS_XL niemonotoniczna, FIFO_MODE w CTRL5[2:0]).
- FW tag w RTT obowiązkowy przy każdej wersji.
- Nie flashować app-only po full erase (SD first).
