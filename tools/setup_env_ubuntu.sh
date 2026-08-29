#!/usr/bin/env bash
# Przygotowanie srodowiska buildu trikig-fw (Ubuntu/Debian x86_64, testowane na 24.04).
# Uzycie: ./tools/setup_env_ubuntu.sh
set -euo pipefail

# 1) Toolchain arm-none-eabi (systemowy GCC 13.2.1 - build reference).
sudo apt-get update
sudo apt-get install -y gcc-arm-none-eabi

# 2) nRF5 SDK 17.1.1 (app uzywa S112; SDK nie wchodzi do repo - licencja + rozmiar).
SDK_DIR="${HOME}/nrf5sdk"
if [ ! -d "$SDK_DIR" ]; then
  echo "Pobierz nRF5 SDK 17.1.1 (nRF5_SDK_17.1.1_ddde560.zip) z nordicsemi.com"
  echo "i rozpakuj do: $SDK_DIR  (https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/download)"
  exit 1
fi

# 3) Makefile.posix: systemowy toolchain zamiast dedykowanego (build reference na 104).
POSIX="$SDK_DIR/components/toolchain/gcc/Makefile.posix"
if ! grep -q 'GNU_VERSION := 13.2.1' "$POSIX" 2>/dev/null; then
  echo "Ustaw w $POSIX:"
  echo '  GNU_INSTALL_ROOT := /usr/bin/'
  echo '  GNU_VERSION := 13.2.1'
  echo '  GNU_PREFIX := arm-none-eabi'
fi

echo "OK. Build: cd triki && make  (SDK_ROOT domyslnie ~/nrf5sdk)"
