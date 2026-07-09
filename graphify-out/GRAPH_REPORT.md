# Graph Report - UcusYazilimi2026  (2026-07-09)

## Corpus Check
- 33 files · ~41,043 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 739 nodes · 815 edges · 100 communities (33 shown, 67 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `a7698056`
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

## God Nodes (most connected - your core abstractions)
1. `DebugSnapshot` - 37 edges
2. `DebugSnapshot` - 35 edges
3. `TelemetryPacket` - 24 edges
4. `GorevYukuPaket` - 23 edges
5. `TelemetryPacket` - 23 edges
6. `TelemetryPacket` - 22 edges
7. `TelemetryPacket` - 20 edges
8. `GorevYukuPaket` - 19 edges
9. `TelemetryPacket` - 17 edges
10. `TelemetryPacket` - 16 edges

## Surprising Connections (you probably didn't know these)
- `bufferla_ve_yaz_sd()` --references--> `TelemetryPacket`  [EXTRACTED]
  test/test_ucus/test_main.cpp → test/test_ucus/test_main.cpp  _Bridges community 10 → community 24_
- `cercevele()` --references--> `TelemetryPacket`  [EXTRACTED]
  test/test_ucus/test_main.cpp → test/test_ucus/test_main.cpp  _Bridges community 10 → community 25_

## Import Cycles
- None detected.

## Communities (100 total, 67 thin omitted)

### Community 0 - "Debug Firmware (main_debug)"
Cohesion: 0.06
Nodes (48): bufferla_ve_yaz_sd(), build_framed(), File, uart_port_t, crc16_ccitt(), dbg_append(), durum_adi(), Funye1Atesle() (+40 more)

### Community 1 - "SIT/SUT Reference Firmware"
Cohesion: 0.05
Nodes (45): HardwareSerial, Print, be32_to_float(), crc16_ccitt(), csv_alan(), float_to_be32(), Funye1Atesle(), Funye2Atesle() (+37 more)

### Community 2 - "Main Flight Firmware"
Cohesion: 0.06
Nodes (45): be32_to_float(), bufferla_ve_yaz_sd(), File, uart_port_t, crc16_ccitt(), float_to_be32(), Funye1Atesle(), Funye2Atesle() (+37 more)

### Community 3 - "Payload Debug Firmware"
Cohesion: 0.06
Nodes (44): beacon_guncelle(), bufferla_ve_yaz_sd(), build_framed(), File, uart_port_t, crc16_ccitt(), dbg_append(), gonder_paket_framed_dma() (+36 more)

### Community 4 - "DebugSnapshot IMU Raw/Cal"
Cohesion: 0.06
Nodes (35): DebugSnapshot, apo_A, apo_B, apo_D, bme_basinc_hpa, bme_nem, bme_sicaklik, cal_accel (+27 more)

### Community 5 - "DebugSnapshot Payload Sensors"
Cohesion: 0.05
Nodes (37): DebugSnapshot, anlik_dikey_hiz, cal_accel, cal_gyro, cal_mag, cal_sys, dongu_hz, dongu_sayaci (+29 more)

### Community 6 - "Video Parsed Firmware"
Cohesion: 0.07
Nodes (30): crc16_ccitt(), gonder_paket_framed(), hesapla_dikey_hiz(), logMesaj(), setup(), SimpleKalmanFilter, err_estimate, err_measure (+22 more)

### Community 7 - "Video Firmware"
Cohesion: 0.07
Nodes (27): hesapla_dikey_hiz(), logMesaj(), setup(), SimpleKalmanFilter, err_estimate, err_measure, first_run, kalman_gain (+19 more)

### Community 8 - "Payload Firmware (GorevYuku)"
Cohesion: 0.06
Nodes (37): beacon_guncelle(), bufferla_ve_yaz_sd(), File, uart_port_t, crc16_ccitt(), gonder_paket_framed_dma(), GorevYukuPaket, basinc (+29 more)

### Community 9 - "Flight Logic Unit Tests"
Cohesion: 0.08
Nodes (12): hesapla_dikey_hiz_test(), hesapla_eglim_acisi(), i2c_read_reg(), test_dikey_hiz_ilk_cagri_sifir(), test_dikey_hiz_inis(), test_dikey_hiz_yukselis(), test_eglim_dik_sifir(), test_eglim_guvenlik_gecer() (+4 more)

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
Cohesion: 0.07
Nodes (27): 1. Özet ve Tasarım Felsefesi, 2. Repo Yapısı ve Derleme Ortamları, 3. Sistem Mimarisi: Çift Çekirdek ve FreeRTOS, 4. Algoritmik Altyapı ve Matematiksel Modeller, 5. Haberleşme ve Veri İşleme Protokolleri, 6. SİT / SUT Entegrasyon Protokolü, 7. Hata Modları ve Etki Analizi (FMEA), 8. Donanım Altyapısı ve Pinout (+19 more)

### Community 14 - "SitPaketi Struct (B)"
Cohesion: 0.15
Nodes (13): SitPaketi, aciX, aciY, aciZ, basinc, checksum, footer1, footer2 (+5 more)

### Community 15 - "Kalman Filter Tests"
Cohesion: 0.15
Nodes (11): SimpleKalmanFilter, err_estimate, err_measure, first_run, kalman_gain, last_estimate, q, test_kalman_gurultu_azaltir() (+3 more)

### Community 21 - "Igniter Driver (Funye)"
Cohesion: 0.52
Nodes (5): FunyeAtesle(), FunyeGuncelle(), FunyePinYaz(), GeriSayimLog(), loop()

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
Cohesion: 0.15
Nodes (12): Changes Made, Compilation Result, Concerns, Git Commit, Step 1: SİT/SUT Define Bloğu (Insertion point: after line 144), Step 2: SitSutModu Enum (Insertion point: after line 180), Step 3: Global Değişkenler (Insertion point: after line 185), Step 4: Basinç Değişkeni (Insertion point: after line 253) (+4 more)

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
Cohesion: 0.22
Nodes (8): Commit Bilgisi, Derleme Sonucu, Değişiklikler, Dosya: `src/main.cpp`, Endianness ve Veri Formatı Doğrulaması, Herhangi bir Endişe veya Not, Task 2: Yardımcı Fonksiyonlar — Rapor, Özet

### Community 41 - "Task 5 Report: Task2code — TTL parser + gecikmeli aktivasyon"
Cohesion: 0.25
Nodes (7): Commit, Compile, Concerns, Edits applied, No ack/log prints, Queue body integrity check, Task 5 Report: Task2code — TTL parser + gecikmeli aktivasyon

## Knowledge Gaps
- **433 isolated node(s):** `err_measure`, `err_estimate`, `q`, `last_estimate`, `kalman_gain` (+428 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **67 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `DebugSnapshot` connect `DebugSnapshot Payload Sensors` to `Payload Debug Firmware`?**
  _High betweenness centrality (0.009) - this node is a cross-community bridge._
- **Why does `DebugSnapshot` connect `DebugSnapshot IMU Raw/Cal` to `Debug Firmware (main_debug)`?**
  _High betweenness centrality (0.009) - this node is a cross-community bridge._
- **What connects `err_measure`, `err_estimate`, `q` to the rest of the system?**
  _445 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Debug Firmware (main_debug)` be split into smaller, more focused modules?**
  _Cohesion score 0.06033182503770739 - nodes in this community are weakly interconnected._
- **Should `SIT/SUT Reference Firmware` be split into smaller, more focused modules?**
  _Cohesion score 0.053877551020408164 - nodes in this community are weakly interconnected._
- **Should `Main Flight Firmware` be split into smaller, more focused modules?**
  _Cohesion score 0.05697278911564626 - nodes in this community are weakly interconnected._
- **Should `Payload Debug Firmware` be split into smaller, more focused modules?**
  _Cohesion score 0.06471631205673758 - nodes in this community are weakly interconnected._