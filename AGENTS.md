# AGENTS.md — zasady pracy w tym repo (dla agentów; hierarchia: użytkownik > ten plik > globalny)

## Buildy i artefakty
- **NIE twórz buildów produkcyjnych (`make release`, hex release) bez wyraźnego żądania użytkownika.**
  Domyślny artefakt do walidacji na sprzęcie = build DEV (`make`, RTT=1), wypuszczany jako
  `firmware/hex/trikig_triki_vX.Y.Z_rtt.hex`.
- Release hex powstaje wyłącznie na wyraźną prośbę ("zbuduj release", promocja produkcji).
- Przy oddawaniu hexu: zawsze zweryfikuj, że to właściwy wariant (dev vs release po
  `data bytes == text+data` w hexie i po rozmiarach z `arm-none-eabi-size`); błąd
  "No control block found" w JLink RTT = wgrany release, nie uszkodzony flash.

## Środowisko buildu
- Build wyłącznie na `ubuntu-vm` (`ssh ubuntu-vm`), SDK 17.1.1 w `~/nrf5sdk`,
  toolchain arm-none-eabi 13.2.1. Lokalne repo (Debian) = edycja kodu; sync przez rsync
  do `~/trikig-fw-build/` lub `git archive` z konkretnego commita (dla buildów per-commit).
- nrfjprog nie jest zainstalowany na VM — flash/smoke robi użytkownik na swoim stanowisku.
- nRF5 SDK pobrany z nordicsemi.com może zwracać 403 (robots) — w CI build FW jest opcjonalny.

## Walidacja (zanim napiszesz "zadziałało")
- Zmiana = commit tylko po `make`/`make release` na VM (zgodnie z powyższą zasadą) bez
  warningów z plików projektu; harness `tools/vbt_offline/vbt_offline all` przy każdej
  zmianie w `triki/trikig_vbt.c`.
- Status "zadziałało na sprzęcie" ustala WYŁĄCZNIE log RTT / pomiar użytkownika.
  Until then: "flash/smoke PENDING".
- Nie deklaruj skuteczności fixa bez repro-before/after (patrz: wykryte bugi C6).

## Konwencje
- Commity: polski, jedna linia z pełnym opisem (styl z `git log`); semver D-020
  (MINOR = protokół, PATCH = iteracja); kontrakt wire v1/v2 zamrożony.
- Dokumentacja: SPEC.md (kontrakty, known-issues), DSP_MAP.md (roadmap F1-F7);
  każda zmiana funkcjonalna = sync docs w tym samym pushu.
- Pomiary HW (podziałki, polaryzacje, piny) mają pierwszeństwo przed notatkami historycznymi
  w docs — po pomiarze aktualizuj SPEC i oznaczaj weryfikację jako [P] (pomiar FW) / [Z] (plyta).
