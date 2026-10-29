#!/bin/bash
# 第一次用 waveshare-s3-lcd128-pm 環境建置若遇到連結錯誤
#   dangerous relocation: l32r: literal placed after use
# 是 pioarduino 套件裡 esp32s3/qio_qspi/sections.ld 少列了 5 個 .literal.* 區段。這支腳本把 patches/sections.ld.patch 套上去 (會先備份)。
# 函式庫套件重裝後要再跑一次。
set -e
LD=${1:-$HOME/.platformio/packages/framework-arduinoespressif32-libs/esp32s3/qio_qspi/sections.ld}
[ -f "$LD" ] || { echo "找不到 $LD (先跑一次 pio run 讓套件下載下來)"; exit 1; }
cp -n "$LD" "$LD.orig" || true
patch --forward "$LD" "$(dirname "$0")/sections.ld.patch" && echo "已套用: $LD"
