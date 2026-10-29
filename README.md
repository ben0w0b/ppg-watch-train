# PPG Watch — 自製 PPG 壓力手錶韌體與雙端設定工具

自製手錶（Waveshare ESP32-S3-Touch-LCD-1.28 ＋ SparkFun MAX30101／MAX32664 感測器）的韌體，加上一支 Android 設定工具 app：用藍牙連上手錶、**同步時間**、**調整壓力判定線**、看目前的判定狀態。

這份 repo 是 2026-10-29 從研究用的主專案抽出來的「目前燒在手錶上的版本」，之後獨立維護。

```
ppg-watch/
├── .github/workflows/build-app.yml   GitHub Actions:push 後自動編 Android APK
├── firmware/        PlatformIO 專案(ESP32-S3,Arduino 3.x / ESP-IDF 5.x hybrid)
│   ├── src/         main.cpp 與所有標頭(處理鏈、tiny CNN、活動辨識、BLE、顯示、電源)
│   ├── tools/       模型標頭(stress_model.h、har_forest.h、har_model.h;由訓練腳本產生,勿手改)
│   ├── patches/     PlatformIO 套件連結腳本的補丁(第一次建置若連結失敗用)
│   ├── platformio.ini、partitions_16MB.csv、sdkconfig.*、pyproject.toml
└── app/             Flutter 設定工具「PPG Watch 工具」(Android)
```

---

## 一、手錶做什麼

- 感測器 hub 以 100 Hz 讀紅外 PPG → 重取樣 64 Hz → 0.5–10 Hz 帶通 → 120 秒視窗 → **tiny 1D-CNN 每秒判定一次**(分數 > 判定線 = 壓力);另有 18 維 HRV 特徵的邏輯迴歸模型可切換(`MODEL,0|1`)。
- 加速度 20 Hz → 隨機森林活動辨識(sit／stand／walk／run);配戴偵測(脈波相關＋動作),脫下 3 分鐘暫停判定。
- 螢幕(時鐘頁)**預設常亮**:時間、日期、STRESS／CALM／NO SIGNAL／WAIT／OFF WRIST、即時分數 vs 判定線、心率。`SLEEP,<秒>` 可改成沒操作就關背光。
- 原始訊號寫進 flash 的滾動快取(30 秒一檔、留最新 20 檔),手機可用 `SYNC`／`GET` 拉回;每 60 秒一筆電池紀錄 `/bat.csv`。
- BLE(NimBLE,名稱 `PPG Watch`)一個服務四個特徵值;每秒推一行狀態給手機。
- 電源管理:40–240 MHz 動態調頻、自動 light sleep(螢幕亮、接 USB、傳檔時不睡)。螢幕常亮時不會進 light sleep,續航明顯短於關螢幕的版本。

## 二、建置與燒錄韌體

### 需要
- Python 3.11+ 與 [PlatformIO](https://platformio.org/)(`pip install platformio` 或 `uv sync`)。
- 第一次建置 `waveshare-s3-lcd128-pm` 環境會下載 pioarduino 平台、ESP-IDF 與工具鏈(約 2.5 GB)並從原始碼編譯 Arduino 核心,**20–40 分鐘**;之後改程式只要十幾秒。
- 手錶用 USB-C 接電腦,板上 CH343 UART 自動下載,不用按鍵。Linux 的序列埠是 `/dev/ttyACM0`,Windows 是 `COMx`(改 `platformio.ini` 的 `upload_port`／`monitor_port` 或加 `--upload-port`)。
- WSL:先在 Windows 用 `usbipd attach --wsl --busid <手錶的 busid>` 把 USB 裝置交給 WSL。

### 步驟
```bash
cd firmware
pio run                       # 編譯(預設環境 waveshare-s3-lcd128-pm)
pio run -t upload             # 燒錄
pio device monitor -b 921600  # 看序列埠(開埠會讓手錶重開機一次)
```
開機後序列埠應該看到:
```
# cnn selftest ok err=...      ← CNN 自檢
# chaintest nPeak=... score=... ← 處理鏈自檢
# stress model=cnn thr=... (nvs|default)
# BLE advertising as PPG Watch
```

### 第一次建置的兩個已知問題
1. **連結錯誤 `dangerous relocation: l32r: literal placed after use`**:pioarduino 套件裡 `esp32s3/qio_qspi/sections.ld` 少列了 5 個 `.literal.*` 區段。跑 `firmware/patches/apply_sections_patch.sh`(會備份原檔再套 `sections.ld.patch`),重新 `pio run`。套件重裝後要再跑一次。
2. `custom_sdkconfig` 裡寫 `=n` 的行不會生效(pioarduino 的限制),所以關閉的選項是靠不寫。

### 序列埠／BLE 指令(兩邊共用,一行一個,結尾 `\n`)
| 指令 | 作用 |
|---|---|
| `TIME,<unix 秒>,<時區分鐘>` | 對時(手錶沒有 RTC,開機後時間由手機給;沒對時螢幕顯示 `--:--`) |
| `THR,<數值>` / `THR,RESET` | 寫入／重設個人壓力判定線(存 NVS,重開機仍在;模型切換時作廢) |
| `MODEL,0|1` | 0 = 邏輯迴歸(HRV 特徵)、1 = tiny CNN(預設) |
| `STAT` | 印狀態:上線時間、快取、電壓與電量、螢幕、CPU 頻率 |
| `SLEEP,<秒>` | 幾秒沒操作關背光;0 = 常亮(預設) |
| `LIVE,0|1` | 時鐘頁的即時分數(預設 1) |
| `FAST,0|1` | 校正模式:狀態列 `fast=1`(手機端校正時用;15 分鐘自動關) |
| `SRATE,<秒>` | 狀態列心跳間隔(預設 30;判定每秒一次時實際每秒送) |
| `REC` `STOP` `DUMP` `SYNC,<after>[,1]` `GET,<off>,<len>` `CLEAR` | 快取段落標記、整份／增量拉取、清除 |
| `WORN,0|1` `USB,0|1|A` `BATLOG` `BATCLR` `REBOOT` | 診斷:強制配戴／電源狀態、電池紀錄、重開 |
其餘(`MASK` `HUBMS` `HUBRATE` `PW` `IR` `GYRO` `IMUFIFO` `ROT` `LP` `BG` `VERB` `PM` `NOSLEEP` `GPIOSW` `BL` `PMDUMP` `IMUDBG`)見 `src/main.cpp` 的 `handleCommand`。

### 狀態列(BLE 特徵值 0003 通知;序列埠不印)
`S,ms,score,thr,state,nRR,har,rec,fast,hr,worn,gIRcorr,vbat,chg,alt,pct`
- `state`:1 壓力、0 非壓力、-1 訊號不足、-2 還沒算過;`worn`:0 = 脫下超過 3 分鐘(暫停判定);`alt` = 另一個模型的分數;`pct` = 電量 %。

### BLE 服務
服務 `7a5e0001-8f2b-4c1d-9e3a-0c1f2a3b4c5d`;特徵值 `…0002` 指令(write / write without response)、`…0003` 狀態(notify / read)、`…0004` 波形串流(notify,預設不用)、`…0005` 檔案傳輸(notify)。

---

## 三、設定工具 app「PPG Watch 工具」(Android)

Flutter 寫的單頁工具,功能只有三件:**連線**、**同步時間**、**調整判定線**(另有看狀態與送任意指令的進階區)。不收資料、不連網。只做 Android(iOS 要 Apple 開發者帳號與 Mac 才能裝給別人,2026-10-29 決定不做)。

### 建置
```bash
cd app
flutter pub get
flutter test                         # 狀態列解析的測試
flutter build apk --release          # → build/app/outputs/flutter-apk/app-release.apk,傳到手機安裝 (要允許安裝未知來源)
```
Android 12+ 用 `BLUETOOTH_SCAN`／`BLUETOOTH_CONNECT`,11 以下掃描要定位權限(都寫在 `AndroidManifest.xml`)。
push 到 GitHub 後,`.github/workflows/build-app.yml` 會在 Ubuntu 機器自動編,到 **Actions › Build app** 的 Artifacts 下載 `ppg-watch-tool-android.apk`。

### 使用步驟
1. 手錶開機(接電或電池都可以);手機開藍牙。第一次開 app 會要「附近的裝置」(藍牙) 與定位(Android 11 以下)權限。
2. 開 app,按 **掃描**,清單出現 `PPG Watch`(只列有本服務 UUID 的裝置),點它 → 連線。連上後畫面上方顯示「已連線」,下面每秒更新狀態(分數／判定線／STRESS 或 CALM／心率／配戴／電量)。
3. **同步時間**:連上時 app 會自動送一次手機的時間與時區;要再對一次按 **同步時間**。手錶螢幕的時間立刻變成手機時間。
4. **調整判定線**:畫面顯示目前手錶上的判定線(從狀態列讀回來)。在輸入框填新值(或用 −0.1／+0.1 微調),按 **套用判定線** → 手錶存進 NVS,狀態列的 `thr` 隨即更新;**重設判定線**回到模型預設值(CNN 為 0)。
   - 判定線的單位是模型分數(CNN 是 logit),不是心率。分數高於判定線就顯示 STRESS。要知道合理的值,先看 CALM 狀態下的分數大約落在哪,再往上抓。
5. 要看更多:展開 **進階**,可以手打任何一個第二節表裡的指令(例如 `STAT`、`SLEEP,15`、`MODEL,0`)並送出;回應會印在手錶的序列埠,不會回到手機(狀態列的變化看得到)。
6. 用完按 **中斷**。手錶不連手機也照樣判定、寫快取。

### 常見問題
- 掃不到:確認手錶螢幕有亮、距離 1 公尺內、沒有另一支手機已經連著(手錶同時只接受一個連線)。Android 要確認藍牙與(Android 11 以下)定位都開著。
- 連上但沒有狀態:等 1–2 秒;若一直沒有,斷開重連(app 會重新訂閱通知)。
- 時間還是 `--:--`:再按一次同步時間;手錶重開機後要再對時(沒有 RTC)。

---

## 四、放到 GitHub
```bash
cd ppg-watch
git init && git add . && git commit -m "PPG Watch firmware + setting tool"
git remote add origin git@github.com:<你的帳號>/ppg-watch.git
git push -u origin main
```
`.gitignore` 已排除 PlatformIO 的 `.pio/`、`managed_components/`(建置時自動下載)與 Flutter 的 `build/`、`.dart_tool/`、`android/local.properties`。模型標頭(`firmware/tools/*.h`、`src/cnn_model.h`)是訓練腳本的產物,直接進版控;訓練腳本與資料集不在這份 repo。

## 五、授權
尚未指定。放上 GitHub 前請選一個(研究用常見 MIT 或 Apache-2.0),並確認第三方函式庫(SparkFun Bio Sensor Hub、LovyanGFX、NimBLE-Arduino、flutter_blue_plus、permission_handler)的授權相容。
