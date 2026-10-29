# PPG Watch 工具 (Flutter, Android)

用藍牙連自製 PPG 壓力手錶:同步時間、調整壓力判定線、看判定狀態、送任意指令。
完整說明(建置、使用步驟、指令表)在上一層的 [README.md](../README.md) 第三節。

```bash
flutter pub get
flutter test
flutter build apk --release
```
程式只有一個檔:`lib/main.dart`。BLE UUID 與狀態列格式要和 `firmware/src/ble_link.h`、`firmware/src/main.cpp` 一致。只做 Android。
