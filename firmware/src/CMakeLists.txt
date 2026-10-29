# 只給 pioarduino hybrid compile (env waveshare-s3-lcd128-pm) 的 ESP-IDF 階段用:
# 那一階段只是用自訂 sdkconfig 重建 Arduino/IDF 函式庫,韌體本身仍由 PlatformIO 的 Arduino 建置器
# 依 build_src_filter 編譯。這裡註冊空元件,免得 IDF 把 src/ 底下多份 setup()/loop() 一起編。
idf_component_register()
