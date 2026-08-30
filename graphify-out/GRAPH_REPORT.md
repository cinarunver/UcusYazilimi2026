# Graph Report - UcusYazilimi2026  (2026-07-30)

## Corpus Check
- 43 files · ~68,601 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 930 nodes · 1059 edges · 129 communities (61 shown, 68 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `ca2fbe62`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Debug Firmware (main_debug)
- SIT/SUT Reference Firmware
- Main Flight Firmware
- Payload Debug Firmware
- DebugSnapshot IMU Raw/Cal
- DebugSnapshot Payload Sensors
- Video Parsed Firmware
- Video Firmware
- Payload Firmware (GorevYuku)
- Flight Logic Unit Tests
- TelemetryPacket Struct
- SIT/SUT Python Test Harness
- Kalman Filter Test
- SitPaketi Struct (A)
- SitPaketi Struct (B)
- Kalman Filter Tests
- SIT/SUT Tasks 1-2 (Constants & Serialization)
- Integration Plan & Tasks 5-6
- Apogee & Funye Safety Logic
- Flight Computer Overview (README)
- LoRa Telemetry & Transport Design
- Igniter Driver (Funye)
- I2C Sensor Test
- FreeRTOS Dual-Core Architecture
- SD Card Ping-Pong Buffer
- CRC16 Framing & Tests
- TTL Parser & Big-Endian Decode
- Task 4 (Mode Reset & Flight-State)
- RS232 RX Test
- Final-review fixes
- TelemetryPacket
- SİT/SUT'un Güncel Main'e Gömülmesi — Tasarım
- Task 1 Report: SİT/SUT sabitleri, enum, global ve SitPaketi struct eklendi
- Task 4 Report — Task1code: mod reset, SİT basınç okuma, 10Hz TTL gönderim, durum bitleri
- Global Constraints
- Task 3 Report: Fünye SUT→LED güvenlik mantığı
- Task 2: Yardımcı Fonksiyonlar — Rapor
- Task 5 Report: Task2code — TTL parser + gecikmeli aktivasyon
- task-6-brief.md
- progress.md
- task-1-brief.md
- task-2-brief.md
- task-3-brief.md
- task-4-brief.md
- task-5-brief.md
- SitSutModu Enum + sitSutMod Global
- Task 1: SİT/SUT Sabitleri, Enum, Global ve SitPaketi Struct
- Task 1 Raporu
- be32_to_float() Big-Endian Çözücü
- float_to_be32() Big-Endian Dönüştürücü
- gonder_durum_paketi() SUT Durum Gönderimi (6B)
- gonder_sit_paketi() SİT Telemetri Gönderimi (36B)
- Task 2: Yardımcı Fonksiyonlar (big-endian, gönderim)
- yuvarla2() 2 Basamak Yuvarlama
- Task 2 Raporu
- Funye1Atesle() (SUT→LED guard eklendi)
- Funye2Atesle() (SUT→LED guard eklendi)
- Task 3: Fünye SUT→LED Güvenlik Mantığı
- Task 3 Raporu
- Task1code() Uçuş Görevi (Core 0)
- Task 4: Task1code mod reset, SİT basınç, 10Hz TTL, durum bitleri
- Task 4 Raporu
- 1 sn Gecikmeli Mod Aktivasyonu
- İki Checksum Konvansiyonu Kabulü (Header+Cmd / Cmd+0x6C)
- Task2code() Haberleşme Görevi (Core 1)
- Task 5: Task2code TTL Parser + Gecikmeli Aktivasyon
- TTL Komut/Veri Parser (5B komut, 36B veri)
- Task 5 Raporu
- setup() UART0 Serial.begin + LED init
- Task 6: setup UART0 Başlat + Gösterge LED Init
- Fix C1: Her Mod Geçişinde Flight-State Reset
- Fix I2: 36B TTL Enjeksiyonu Mod-Guard (MOD_SUT)
- Fix I3: Uçuş-İçi START Komutu Kilidi (durum<YUKSELIYOR)
- Task 6 Raporu + Final-Review Düzeltmeleri
- SİT/SUT Main Entegrasyon İmplementasyon Planı
- Test-Modu Onay Mesajları Gönderilmez
- Slim TelemetryPacket Korunumu (YAGNI)
- SUT Modu Donanım Atlama ve TTL Veri Enjeksiyonu
- Transport Ayrımı (LoRa UART1 vs SİT/SUT UART0)
- Apogee Tespiti ve Durum Makinesi (5 faz)
- Big-Endian Byte Sırası (Ek-7 zorunlu)
- E32-433T30D LoRa + ESP-IDF DMA UART Sürücüsü
- Trakya Roket 2026 Uçuş Kontrol Bilgisayarı (V2.0)
- Hata Modları ve Etki Analizi (FMEA)
- ESP32 Çift Çekirdek + FreeRTOS Mimarisi
- FreeRTOS Telemetri Kuyruğu (10 kapasite)
- 1D SimpleKalmanFilter Sensör Füzyonu (10 filtre)
- LoRa Telemetri Paketi (Slim, CRC16-CCITT, 64B)
- PlatformIO Çoklu Derleme Ortamları
- SD Kart Ping-Pong Buffer (512B A/B, DMA)
- SİT/SUT Entegrasyon Protokolü
- sitsut Referans Firmware (SİT_SUT/SİT-SUT.cpp)
- Task1 / Core 0 UçuşGörevi (Priority 2, ~100 Hz)
- Task2 / Core 1 HaberleşmeGörevi (Priority 1)
- Eğim (Tilt) Açısı Hesabı ve NaN Koruması
- Üçlü Çapraz Onay (Drogue Ayrılma Güvenliği)
- lora_test.cpp
- gps_test.cpp
- main.cpp
- LED Durum Göstergesi Tasarımı
- SimpleKalmanFilter
- hesapla_led_durumu
- SİT-SUT.cpp
- LED Durum Göstergesi Implementation Plan
- Bilimsel Görev Yükü — Görev Tanımı ve Yazılım Yeterliliği
- gorevyuku.cpp
- GorevYukuWire
- LoRa Paket Küçültme (Fixed-Point) — Tasarım
- SimpleKalmanFilter
- Global Constraints
- Haberleşme Testi — Veri Paketi Yapısı
- SimpleKalmanFilter
- gonder_paket_csv
- gonder_paket_framed_dma
- hesapla_led_durumu_bgy
- 14. Kurulum ve Çalıştırma
- 4. Uçuş Algoritması ve Matematiksel Modeller
- 7. SD Kart Kara Kutu Loglama
- 8. SİT / SUT Entegrasyon Protokolü
- 15. Dokümantasyon İndeksi
- 2. Repo Yapısı ve Derleme Ortamları
- 5. Bilimsel Görev Yükü (BGY)
- 11. Donanım Altyapısı ve Pinout
- 3. Sistem Mimarisi: Çift Çekirdek ve FreeRTOS

## God Nodes (most connected - your core abstractions)
1. `DebugSnapshot` - 35 edges
2. `DebugSnapshot` - 34 edges
3. `TelemetryPacket` - 26 edges
4. `TelemetryPacket` - 24 edges
5. `GorevYukuPaket` - 24 edges
6. `TelemetryPacket` - 23 edges
7. `TelemetryPacket` - 22 edges
8. `GorevYukuPaket` - 19 edges
9. `🚀 Trakya Roket 2026 — Uçuş Yazılımı` - 18 edges
10. `TelemetryPacket` - 17 edges

## Surprising Connections (you probably didn't know these)
- `hesapla_led_durumu_bgy()` --references--> `LedDurumBgy`  [EXTRACTED]
  GorevYukuYazilimi/gorevyuku.cpp → GorevYukuYazilimi/gorevyuku.cpp  _Bridges community 118 → community 109_
- `bufferla_ve_yaz_sd()` --references--> `GorevYukuPaket`  [EXTRACTED]
  GorevYukuYazilimi/gorevyuku.cpp → GorevYukuYazilimi/gorevyuku.cpp  _Bridges community 8 → community 112_
- `pack_gorevyuku_wire()` --references--> `GorevYukuPaket`  [EXTRACTED]
  GorevYukuYazilimi/gorevyuku.cpp → GorevYukuYazilimi/gorevyuku.cpp  _Bridges community 8 → community 109_
- `pack_gorevyuku_wire()` --references--> `GorevYukuWire`  [EXTRACTED]
  GorevYukuYazilimi/gorevyuku.cpp → GorevYukuYazilimi/gorevyuku.cpp  _Bridges community 110 → community 109_
- `gonder_paket_framed_dma()` --calls--> `pack_gorevyuku_wire()`  [EXTRACTED]
  GorevYukuYazilimi/gorevyuku.cpp → GorevYukuYazilimi/gorevyuku.cpp  _Bridges community 109 → community 112_

## Import Cycles
- None detected.

## Communities (129 total, 68 thin omitted)

### Community 0 - "Debug Firmware (main_debug)"
Cohesion: 0.06
Nodes (50): bufferla_ve_yaz_sd(), build_framed(), File, uart_port_t, crc16_ccitt(), dbg_append(), durum_adi(), Funye1Atesle() (+42 more)

### Community 1 - "SIT/SUT Reference Firmware"
Cohesion: 0.06
Nodes (48): HardwareSerial, Print, be32_to_float(), crc16_ccitt(), csv_alan(), float_to_be32(), Funye1Atesle(), Funye2Atesle() (+40 more)

### Community 2 - "Main Flight Firmware"
Cohesion: 0.09
Nodes (23): TelemetryPacket, ayrilma1_durum, ayrilma2_durum, dikeyHiz, eglimAcisi, gpsBoylam, gpsEnlem, gyroX (+15 more)

### Community 3 - "Payload Debug Firmware"
Cohesion: 0.07
Nodes (40): beacon_guncelle(), bufferla_ve_yaz_sd(), build_framed(), File, uart_port_t, crc16_ccitt(), dbg_append(), gonder_paket_framed_dma() (+32 more)

### Community 4 - "DebugSnapshot IMU Raw/Cal"
Cohesion: 0.06
Nodes (35): DebugSnapshot, apo_A, apo_B, apo_D, bme_basinc_hpa, bme_nem, bme_sicaklik, cal_accel (+27 more)

### Community 5 - "DebugSnapshot Payload Sensors"
Cohesion: 0.06
Nodes (34): DebugSnapshot, anlik_dikey_hiz, cal_accel, cal_gyro, cal_mag, cal_sys, dongu_hz, dongu_sayaci (+26 more)

### Community 6 - "Video Parsed Firmware"
Cohesion: 0.07
Nodes (30): crc16_ccitt(), gonder_paket_framed(), hesapla_dikey_hiz(), logMesaj(), setup(), SimpleKalmanFilter, err_estimate, err_measure (+22 more)

### Community 7 - "Video Firmware"
Cohesion: 0.07
Nodes (27): hesapla_dikey_hiz(), logMesaj(), setup(), SimpleKalmanFilter, err_estimate, err_measure, first_run, kalman_gain (+19 more)

### Community 8 - "Payload Firmware (GorevYuku)"
Cohesion: 0.10
Nodes (21): GorevYukuPaket, basinc, es, gpsBoylam, gpsEnlem, gyroX, gyroY, gyroZ (+13 more)

### Community 9 - "Flight Logic Unit Tests"
Cohesion: 0.09
Nodes (7): hesapla_dikey_hiz_test(), i2c_read_reg(), test_dikey_hiz_ilk_cagri_sifir(), test_dikey_hiz_inis(), test_dikey_hiz_yukselis(), test_hw_bme280_chip_id(), test_hw_bno055_chip_id()

### Community 10 - "TelemetryPacket Struct"
Cohesion: 0.11
Nodes (18): TelemetryPacket, ayrilma1_durum, ayrilma2_durum, dikeyHiz, eglimAcisi, gpsBoylam, gpsEnlem, gyroX (+10 more)

### Community 11 - "SIT/SUT Python Test Harness"
Cohesion: 0.22
Nodes (8): decode_durum(), main(), Komut gonderir (opsiyonel) ve <saniye> boyunca gelen HER byte'i ham gosterir., 16 bitlik durum degerini aktif asama isimlerine cevirir., Sistemdeki aktif seri portları listeler ve kullanıcıya seçtirir., Tam sentetik ucus profili: kalkis -> apogee -> drogue -> ana parasut -> inis., RocketTester, select_serial_port()

### Community 12 - "Kalman Filter Test"
Cohesion: 0.16
Nodes (10): baslikBas(), loop(), setup(), SimpleKalmanFilter, err_estimate, err_measure, first_run, kalman_gain (+2 more)

### Community 13 - "SitPaketi Struct (A)"
Cohesion: 0.25
Nodes (8): 10. Hata Modları ve Etki Analizi (FMEA), 12. Yer İstasyonu (Ayrı Depo), 13. Doğrulama ve Test, 1. Özet ve Tasarım Felsefesi, 9. Güvenlik Tasarımı: Fünye Pin Yönetimi, 📋 İçindekiler, Mühendislik Tasarım Raporu ve Teknik Dokümantasyon, 🚀 Trakya Roket 2026 — Uçuş Yazılımı

### Community 14 - "SitPaketi Struct (B)"
Cohesion: 0.15
Nodes (13): SitPaketi, aciX, aciY, aciZ, basinc, checksum, footer1, footer2 (+5 more)

### Community 15 - "Kalman Filter Tests"
Cohesion: 0.25
Nodes (7): SimpleKalmanFilter, err_estimate, err_measure, first_run, kalman_gain, last_estimate, q

### Community 21 - "Igniter Driver (Funye)"
Cohesion: 0.70
Nodes (4): FunyePinYaz(), GeriSayimLog(), loop(), setup()

### Community 22 - "I2C Sensor Test"
Cohesion: 0.67
Nodes (6): bno_imu_moduna_al(), loop(), read_int16(), read_register(), setup(), write_register()

### Community 24 - "SD Card Ping-Pong Buffer"
Cohesion: 0.38
Nodes (7): bufferla_ve_yaz_sd(), File, format_csv_line(), ornek_paket(), sd_buffer_bosalt(), sd_state_reset(), test_hw_sd_pingpong_kayipsiz()

### Community 25 - "CRC16 Framing & Tests"
Cohesion: 0.33
Nodes (7): cercevele(), crc16_ccitt(), test_cerceve_baslik_ve_boyut(), test_cerceve_bozuk_payload_crc_yakalar(), test_cerceve_payload_bozulmadan(), test_crc16_bilinen_vektor(), test_crc16_tek_bit_farkli()

### Community 33 - "Final-review fixes"
Cohesion: 0.09
Nodes (21): Commit, Commit, Compilation Results, Concerns, Concerns, Edit 1: Serial (UART0) Başlat, Edit 2: Gösterge LED'lerini Başta Söndür, Edits Applied (+13 more)

### Community 34 - "TelemetryPacket"
Cohesion: 0.15
Nodes (13): SitPaketi, aciX, aciY, aciZ, basinc, checksum, footer1, footer2 (+5 more)

### Community 35 - "SİT/SUT'un Güncel Main'e Gömülmesi — Tasarım"
Cohesion: 0.15
Nodes (12): 1. Transport ayrımı (çakışmanın kalbi), 2. Sensör okuma, 3. Paket yapıları, 4. Mod state machine, 5. Güvenlik, Amaç, Başlangıç Durumu (iki ayrışmış sürüm), Doğrulama (+4 more)

### Community 36 - "Task 1 Report: SİT/SUT sabitleri, enum, global ve SitPaketi struct eklendi"
Cohesion: 0.14
Nodes (13): Files Created, Function Logic (Three Behavioral Branches), Function Signature, Git Commit, GREEN Phase (Step 3-4), Implementation Details, Notes, RED Phase (Step 1-2) (+5 more)

### Community 37 - "Task 4 Report — Task1code: mod reset, SİT basınç okuma, 10Hz TTL gönderim, durum bitleri"
Cohesion: 0.17
Nodes (11): Commit, Compile, Concerns, Edits applied, eglim_acisi / anlik_dikey_hiz placement — verified single & correctly relocated, Status: DONE, Step 1 — mode-transition reset (anchor: `void Task1code(void *pvParameters) {` / `for (;;) {`), Step 2 — sensor-read gating + SİT pressure read (anchor: existing sensor-read block, comment `// 1. IMU (BNO055) Verilerini Okuma` through the GPS `if (gps.location.isUpdated())` block) (+3 more)

### Community 38 - "Global Constraints"
Cohesion: 0.20
Nodes (9): Global Constraints, Self-Review Notlarım, SİT/SUT Main Entegrasyonu Implementasyon Planı, Task 1: SİT/SUT sabitleri, enum, global değişkenler ve SitPaketi struct, Task 2: Yardımcı fonksiyonlar (big-endian, yuvarla2, SİT & durum paketi gönderimi), Task 3: Fünye SUT→LED güvenlik mantığı, Task 4: Task1code — mod reset, SİT basınç okuma, 10Hz TTL gönderim, durum bitleri, Task 5: Task2code — TTL parser + gecikmeli aktivasyon (+1 more)

### Community 39 - "Task 3 Report: Fünye SUT→LED güvenlik mantığı"
Cohesion: 0.20
Nodes (9): Changes Made, Commit, Compilation, Concerns, Funye1Atesle() — Lines 376–385, Funye2Atesle() — Lines 387–394, Summary, Task 3 Report: Fünye SUT→LED güvenlik mantığı (+1 more)

### Community 40 - "Task 2: Yardımcı Fonksiyonlar — Rapor"
Cohesion: 0.25
Nodes (7): Concern investigated and resolved: unrelated pre-existing change in the working tree, Files changed, Manual hardware/SUT verification (not performed — out of scope for this agent), Self-review findings, Task 2 Report: main.cpp entegrasyonu (led_uygula, sistem_hazir, LED yazımlarını merkezileştirme), Test/build evidence, What was implemented

### Community 41 - "Task 5 Report: Task2code — TTL parser + gecikmeli aktivasyon"
Cohesion: 0.25
Nodes (7): Commit, Compile, Concerns, Edits applied, No ack/log prints, Queue body integrity check, Task 5 Report: Task2code — TTL parser + gecikmeli aktivasyon

### Community 45 - "task-2-brief.md"
Cohesion: 0.50
Nodes (3): Donanım / SUT Doğrulama (manuel — commit sonrası), Notlar, Task 2: `main.cpp` entegrasyonu (led_uygula, sistem_hazir, LED yazımlarını merkezleştirme)

### Community 102 - "main.cpp"
Cohesion: 0.06
Nodes (52): be32_to_float(), bufferla_ve_yaz_sd(), File, uart_port_t, crc16_ccitt(), float_to_be32(), Funye1Atesle(), Funye2Atesle() (+44 more)

### Community 103 - "LED Durum Göstergesi Tasarımı"
Cohesion: 0.17
Nodes (11): 1. Tek merkezli LED kontrolü — `led_guncelle()`, 2. `sistem_hazir` bayrağı, Amaç, Durum Tablosu (Normal uçuş — MOD_BEKLEME), Girdiler (LED'ler yalnızca bunlara bağlı), LED Durum Göstergesi Tasarımı, LED Pinleri (mevcut, değişmiyor), Mod geçişi ile etkileşim (+3 more)

### Community 104 - "SimpleKalmanFilter"
Cohesion: 0.22
Nodes (9): Filtreleme Politikası, Genel Yaklaşım, Görev Yükü Paket Alanları (32 bayt faydalı yük, little-endian), Görev Yükü Çerçeve Bayt Yerleşimi, Hava Yoğunluğu Hesabı, Konum Belirleyici Sistemler ve Yer İstasyonu Veri Alışveriş Mimarisi, Konum Belirleyiciler, Mimari Diyagram (+1 more)

### Community 105 - "hesapla_led_durumu"
Cohesion: 0.14
Nodes (13): 1. Hava yoğunluğu (nemli hava), 2. Görev yükü wire paketi: 24B → 32B, 3. SD kart loglama, 4.1 Mevcut hata: roket paketi quaternion'ı Euler sanıyor, 4.2 Parse ve ölçekler, 4.3 Arayüz, 4. Yer istasyonu, Amaç (+5 more)

### Community 106 - "SİT-SUT.cpp"
Cohesion: 0.24
Nodes (11): euler_to_quat_xy(), hesapla_eglim_acisi(), hesapla_eglim_acisi_quat(), test_eglim_dik_sifir(), test_eglim_guvenlik_gecer(), test_eglim_nan_uretmiyor(), test_eglim_pitch_sarmasi_bozuyor_ama_kapiyi_acmiyor(), test_eglim_quat_dik_sifir() (+3 more)

### Community 107 - "LED Durum Göstergesi Implementation Plan"
Cohesion: 0.29
Nodes (6): Donanım / SUT Doğrulama (manuel — commit sonrası), Global Constraints, LED Durum Göstergesi Implementation Plan, Notlar, Task 1: Saf LED karar fonksiyonu + host birim testi, Task 2: `main.cpp` entegrasyonu (led_uygula, sistem_hazir, LED yazımlarını merkezleştirme)

### Community 108 - "Bilimsel Görev Yükü — Görev Tanımı ve Yazılım Yeterliliği"
Cohesion: 0.15
Nodes (13): 1. Bilimsel Görev Tanımı, 2.1 Ölçüm katmanı, 2.2 Filtreleme, 2.3 Yer kalibrasyonu, 2.4 Bilimsel çıktı: nemli hava yoğunluğu (F2), 2.5 Kayıt ve iletim ayrımı, 2. Görevi Yerine Getiren Yazılım Mimarisi, 3. Veri Ürünleri (+5 more)

### Community 109 - "gorevyuku.cpp"
Cohesion: 0.18
Nodes (15): beacon_guncelle(), hesapla_dikey_hiz(), hesapla_hava_yogunlugu(), hesapla_led_durumu_bgy(), led_uygula(), log_dosyasi_sec(), lora_konfigurasyon(), lora_log() (+7 more)

### Community 110 - "GorevYukuWire"
Cohesion: 0.13
Nodes (15): GorevYukuWire, basinc, gpsBoylam, gpsEnlem, gyroX, gyroY, gyroZ, irtifa (+7 more)

### Community 111 - "LoRa Paket Küçültme (Fixed-Point) — Tasarım"
Cohesion: 0.18
Nodes (11): Doğrulanacak Varsayımlar, Gönderim Hızı, Kapsam Dışı (YAGNI), Kısıtlar, LoRa Paket Küçültme (Fixed-Point) — Tasarım, Mimari Karar, Problem, Test / Doğrulama (+3 more)

### Community 112 - "SimpleKalmanFilter"
Cohesion: 0.29
Nodes (8): bufferla_ve_yaz_sd(), File, uart_port_t, crc16_ccitt(), gonder_paket_framed_dma(), ondalik_virgulle(), sd_buffer_bosalt(), Task2code()

### Community 113 - "Global Constraints"
Cohesion: 0.22
Nodes (8): Global Constraints, LoRa Paket Küçültme (Fixed-Point) — Implementasyon Planı, Self-Review Notları, Task 1: Roket wire header + host test, Task 2: Roket wire'ı main.cpp'ye entegre et + gönderim hızı, Task 3: Görev yükü wire header + host test, Task 4: Görev yükü wire'ı gorevyuku.cpp'ye entegre et, Task 5: Yer istasyonu (Python) parse + simülasyon revizesi

### Community 114 - "Haberleşme Testi — Veri Paketi Yapısı"
Cohesion: 0.25
Nodes (8): Filtreleme Politikası, Gönderim Kadansı ve Bant Genişliği, Haberleşme Testi — Veri Paketi Yapısı, Nihai Haberleşme Sistemi, Paket Alanları (23 bayt faydalı yük, little-endian), Veri Akış Diyagramı, Veri Paketi Tasarımı, Çerçeve Bayt Yerleşimi

### Community 115 - "SimpleKalmanFilter"
Cohesion: 0.25
Nodes (7): SimpleKalmanFilter, err_estimate, err_measure, first_run, kalman_gain, last_estimate, q

### Community 116 - "gonder_paket_csv"
Cohesion: 0.33
Nodes (5): test_euler_yaw_sarmasi_kaydi_bozuyor(), test_kalman_gurultu_azaltir(), test_kalman_ilk_cagri(), test_kalman_nan_uretmiyor(), test_kalman_yakinsar()

### Community 117 - "gonder_paket_framed_dma"
Cohesion: 0.29
Nodes (7): 6.1 İki katmanlı veri gösterimi, 6.2 UKB telemetri paketi — 23 B (`'<7h2iB'`), 6.3 BGY telemetri paketi — 32 B (`'<HhHhH2i7h'`), 6.4 Tasarım kararları, 6.5 Çerçeveleme ve bütünlük, 6.6 E32-433T30D otomatik konfigürasyon, 6. Haberleşme: Fixed-Point Wire Protokolü

### Community 118 - "hesapla_led_durumu_bgy"
Cohesion: 0.40
Nodes (5): LedDurumBgy, beacon, led1, led2, led3

### Community 121 - "14. Kurulum ve Çalıştırma"
Cohesion: 0.40
Nodes (5): 14.1 Kurulum, 14.2 SİT/SUT testi, 14.3 Yer istasyonu, 14.4 Uçuş öncesi zorunlu kontroller (pre-flight), 14. Kurulum ve Çalıştırma

### Community 122 - "4. Uçuş Algoritması ve Matematiksel Modeller"
Cohesion: 0.40
Nodes (5): 4.1 Sensör füzyonu ve 1D Kalman filtreleri, 4.2 Anlık dikey hız (Vz) ve eğim açısı, 4.3 Apogee tespiti ve durum makinesi, 4.4 LED durum göstergesi, 4. Uçuş Algoritması ve Matematiksel Modeller

### Community 123 - "7. SD Kart Kara Kutu Loglama"
Cohesion: 0.40
Nodes (5): 7.1 Ping-pong tampon, 7.2 Türkçe Excel uyumlu CSV, 7.3 Her uçuşta yeni dosya, 7.4 Kayıt kapsamı, 7. SD Kart Kara Kutu Loglama

### Community 124 - "8. SİT / SUT Entegrasyon Protokolü"
Cohesion: 0.40
Nodes (5): 8.1 Fiziksel katman (Ek-7 Tablo 7), 8.2 ⚙️ Byte sırası: BIG ENDIAN (kritik), 8.3 Komut protokolü (5 byte), 8.4 Veri paketleri, 8. SİT / SUT Entegrasyon Protokolü

### Community 125 - "15. Dokümantasyon İndeksi"
Cohesion: 0.50
Nodes (4): 15. Dokümantasyon İndeksi, Kod haritası, Mühendislik raporu bölümleri, Tasarım kararları (spec) ve uygulama planları

### Community 126 - "2. Repo Yapısı ve Derleme Ortamları"
Cohesion: 0.50
Nodes (4): 2. Repo Yapısı ve Derleme Ortamları, Donanım doğrulama testleri, Teşhis firmware'leri, Uçuş firmware'leri

### Community 127 - "5. Bilimsel Görev Yükü (BGY)"
Cohesion: 0.50
Nodes (4): 5.1 Nemli hava yoğunluğu (F2), 5.2 Yer kalibrasyonu, 5.3 Kurtarma beacon'ı, 5. Bilimsel Görev Yükü (BGY)

### Community 128 - "11. Donanım Altyapısı ve Pinout"
Cohesion: 0.67
Nodes (3): 11.1 UKB — Uçuş Kontrol Bilgisayarı (`ucus`), 11.2 BGY — Bilimsel Görev Yükü (`gorevyuku`), 11. Donanım Altyapısı ve Pinout

### Community 129 - "3. Sistem Mimarisi: Çift Çekirdek ve FreeRTOS"
Cohesion: 0.67
Nodes (3): 3. Sistem Mimarisi: Çift Çekirdek ve FreeRTOS, Gerçek zamanlama, Görev dağılımı

## Knowledge Gaps
- **551 isolated node(s):** `err_measure`, `err_estimate`, `q`, `last_estimate`, `kalman_gain` (+546 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **68 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `🚀 Trakya Roket 2026 — Uçuş Yazılımı` connect `SitPaketi Struct (A)` to `11. Donanım Altyapısı ve Pinout`, `3. Sistem Mimarisi: Çift Çekirdek ve FreeRTOS`, `gonder_paket_framed_dma`, `README.md`, `14. Kurulum ve Çalıştırma`, `4. Uçuş Algoritması ve Matematiksel Modeller`, `7. SD Kart Kara Kutu Loglama`, `8. SİT / SUT Entegrasyon Protokolü`, `15. Dokümantasyon İndeksi`, `2. Repo Yapısı ve Derleme Ortamları`, `5. Bilimsel Görev Yükü (BGY)`?**
  _High betweenness centrality (0.016) - this node is a cross-community bridge._
- **Why does `DebugSnapshot` connect `DebugSnapshot IMU Raw/Cal` to `Debug Firmware (main_debug)`?**
  _High betweenness centrality (0.006) - this node is a cross-community bridge._
- **Why does `DebugSnapshot` connect `DebugSnapshot Payload Sensors` to `Payload Debug Firmware`?**
  _High betweenness centrality (0.005) - this node is a cross-community bridge._
- **What connects `err_measure`, `err_estimate`, `q` to the rest of the system?**
  _563 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Debug Firmware (main_debug)` be split into smaller, more focused modules?**
  _Cohesion score 0.06009783368273934 - nodes in this community are weakly interconnected._
- **Should `SIT/SUT Reference Firmware` be split into smaller, more focused modules?**
  _Cohesion score 0.05505279034690799 - nodes in this community are weakly interconnected._
- **Should `Main Flight Firmware` be split into smaller, more focused modules?**
  _Cohesion score 0.08695652173913043 - nodes in this community are weakly interconnected._