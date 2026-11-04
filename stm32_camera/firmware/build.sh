#!/bin/bash
# macOS build (and optional flash) for the camera node firmware. Windows: build.ps1.
#
#   ./build.sh                 full firmware   -> build-mac/evac_cam.bin
#   ./build.sh --led-test      LED-only test   -> build-mac-ledtest/evac_cam.bin
#   ./build.sh --pin-test      PE9 3 s high / 3 s low, for a multimeter -> build-mac-pintest/
#   ./build.sh --probe         is the Nucleo's ST-LINK visible? (reads only, does not flash)
#   ./build.sh [--led-test] --flash    build, then write to the board and reset
#
# Tools: Arm GNU Toolchain in ~/tools (or ARM_TOOLCHAIN=/path/to/bin), and
# Homebrew's cmake, ninja and stlink (st-flash).
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
toolchain="${ARM_TOOLCHAIN:-$HOME/tools/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin}"
export PATH="$toolchain:/opt/homebrew/bin:$PATH"

led_test=0 pin_test=0 flash=0 probe=0
for arg in "$@"; do
  case "$arg" in
    --led-test) led_test=1 ;;
    --pin-test) pin_test=1 ;;
    --flash)    flash=1 ;;
    --probe)    probe=1 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

missing=0
for tool in arm-none-eabi-gcc cmake ninja; do
  command -v "$tool" >/dev/null || { echo "missing: $tool" >&2; missing=1; }
done
if [ "$flash" = 1 ] || [ "$probe" = 1 ]; then
  command -v st-flash >/dev/null || { echo "missing: st-flash (brew install stlink)" >&2; missing=1; }
fi
[ "$missing" = 0 ] || exit 1

if [ "$probe" = 1 ]; then
  st-info --probe
  exit $?
fi

if [ "$pin_test" = 1 ]; then
  build="$here/build-mac-pintest"
  cmake -S "$here" -B "$build" -G Ninja -DLED_TEST_ONLY=ON -DLED_PIN_TEST=ON
elif [ "$led_test" = 1 ]; then
  build="$here/build-mac-ledtest"
  cmake -S "$here" -B "$build" -G Ninja -DLED_TEST_ONLY=ON -DLED_PIN_TEST=OFF
else
  build="$here/build-mac"
  cmake -S "$here" -B "$build" -G Ninja -DLED_TEST_ONLY=OFF -DLED_PIN_TEST=OFF
fi
cmake --build "$build"
echo "built: $build/evac_cam.bin"

if [ "$flash" = 1 ]; then
  # Under reset: with the camera firmware running, its DMA crashed st-flash's RAM loader.
  st-flash --connect-under-reset --reset write "$build/evac_cam.bin" 0x08000000
fi
