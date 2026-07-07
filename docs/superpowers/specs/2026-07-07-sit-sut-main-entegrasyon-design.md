# SİT/SUT'un Güncel Main'e Gömülmesi — Tasarım

**Tarih:** 2026-07-07
**Dosya:** `src/main.cpp` (tek dosya, tek çıktı)

## Amaç

SİT (Sensör İzleme Testi) ve SUT (Sentetik Uçuş Testi) yeteneğini, güncel
optimize `src/main.cpp` içine — DMA/ping-pong/slim-paket optimizasyonlarının
hiçbirini kaybetmeden — gömmek.

## Başlangıç Durumu (iki ayrışmış sürüm)

- **`src/main.cpp` (güncel, taban olacak):** LoRa için ESP-IDF DMA sürücüsü
  (UART1, E32 M0/M1 config, `uart_write_bytes`), ping-pong SD buffer, slim
  `TelemetryPacket` (yalnız `irtifa` okunur; `basinc`/`sicaklik`/`nem` yok),
  CSV ayracı `,`. **SİT/SUT yok.** UART0/`Serial` hiç kullanılmıyor.
- **`SİT_SUT/SİT-SUT.cpp` (referans, DMA öncesi tabana kurulu):** SİT/SUT tam
  implementasyonu — `Serial1` (bloklayan Arduino) ile LoRa, full BME280,
  CSV ayracı `;`, TTL komutları `Serial` (UART0 @115200). Kaynak olarak
  kullanılacak; buradaki SİT/SUT mantığı porttan geçirilecek.

**Karar (onaylı):** Güncel `main.cpp` mimari taban kalır; SİT/SUT üstüne eklenir.

## Tasarım Kararları

### 1. Transport ayrımı (çakışmanın kalbi)

- **LoRa telemetrisi — değişmez:** IDF DMA sürücüsü, UART1, E32 config,
  `gonder_paket_framed_dma`. Dokunulmaz.
- **SİT/SUT TTL — `Serial` (UART0 @115200):** Güncel main'de UART0 boş; test
  cihazına adanır. UART0'dan: komut alma (5B), SİT telemetri (36B `SitPaketi`),
  SUT durum paketi (6B).
- **Test-modu onay mesajları — GÖNDERİLMEZ (onaylı):** Referanstaki
  `Serial1.println("[SIT] onaylandi...")` / `[SUT]` / `[STOP]` çağrıları port
  EDİLMEZ. Ne LoRa'dan ne TTL'den yollanır. `lora_log` sadece mevcut setup
  teşhis mesajları için kullanılmaya devam eder.

### 2. Sensör okuma

- Uçuş 100 Hz döngüsü **slim kalır** (yalnız `irtifa`) — optimizasyon korunur.
- **SİT modunda** ek olarak `basinc` okunur (Tablo 3 SİT paketi ister). Sadece
  `sitSutMod == MOD_SIT` iken; gerçek uçuş/HAZIR performansına etki yok.
  İvme ve açılar (roll/pitch/yaw) zaten her döngüde okunuyor.
- **SUT modunda** Task1 donanım okumasını atlar; `irtifa`/`basinc`/`ivme`/açı
  globalleri TTL'den (UART0, 36B veri paketi) Task2 tarafından enjekte edilir.

### 3. Paket yapıları

- LoRa `TelemetryPacket` (slim) **değişmez**. `basinc` global bir değişken
  olarak geri gelir ama LoRa paketine EKLENMEZ (sadece SİT paketi kullanır).
- SİT/SUT kendi ayrı structlarını kullanır:
  - `SitPaketi` (36B, `0xAB` header, 8×FLOAT32 big-endian + checksum + `0x0D 0x0A`)
    → UART0/`Serial`.
  - SUT durum paketi (6B, `0xAA` header, Data1/Data2 bit alanları + checksum +
    footer) → UART0/`Serial`.
- Big-endian dönüşüm (`float_to_be32` / `be32_to_float`), `crc16`/`yuvarla2`,
  checksum yardımcıları referanstan port edilir.

### 4. Mod state machine

- `enum SitSutModu { MOD_BEKLEME, MOD_SIT, MOD_SUT }` + `volatile SitSutModu
  sitSutMod = MOD_BEKLEME`.
- **Task2 (Core 1):** TTL parser eklenir — 5B komut ve 36B SUT-veri çerçeveleri;
  iki checksum konvansiyonu (Header+Cmd ve Cmd+0x6C) kabul; 100 ms frame
  timeout; komut onayından 1 sn sonra gecikmeli aktivasyon.
- **Task1 (Core 0):** mod-geçiş reset (yeni SUT için uçuş algoritması sıfırlanır);
  SİT modunda `basinc` okuma + 10 Hz `SitPaketi` gönderimi; SUT modunda 10 Hz
  durum paketi; durum bitleri latch mantığı (Tablo 5).

### 5. Güvenlik

- `#define SUT_FUNYE_YERINE_LED 1` **korunur.** SUT tezgah testinde gerçek fünye
  pini sürülmez, yerine gösterge LED yakılır. SİT ve gerçek uçuşta fünye normal
  çalışır. Aksaray gerçek test cihazı ölçümü için bu bayrak 0 yapılır.
- Fünye çıkışları setup'ta LOW; non-blocking fünye zamanlaması (`FUNYE_SURE_MS`)
  korunur.

## Etkilenen Kod

- **Tek dosya:** `src/main.cpp`.
- Eklenecek define blokları: SİT/SUT komut protokolü, checksum sabitleri,
  zamanlama, durum bitleri, tezgah-testi bayrağı.
- Eklenecek global: `sitSutMod`, `durum_bitleri`, `basinc`, mod/aktivasyon
  değişkenleri.
- Eklenecek struct: `SitPaketi`.
- Eklenecek fonksiyonlar: `float_to_be32`, `be32_to_float`, `yuvarla2`,
  `gonder_sit_paketi`, `gonder_durum_paketi`.
- Değişecek: `Funye1Atesle`/`Funye2Atesle` (SUT→LED bayrak mantığı), `Task1code`
  (mod reset + SİT basinc + 10Hz TTL gönderim + durum bitleri), `Task2code`
  (TTL parser + aktivasyon), `setup` (`Serial.begin(115200)` + gösterge LED init).
- `SİT_SUT/SİT-SUT.cpp` kaynak referans olarak kalır (silinmez).

## Kapsam Dışı (YAGNI)

- LoRa transportunu bloklayan `Serial1`'e geri döndürmek — HAYIR, IDF DMA kalır.
- Slim `TelemetryPacket`'e `basinc`/`sicaklik`/`nem` geri eklemek — HAYIR.
- Test-modu debug/onay mesajları — HAYIR, gönderilmez.
- Yeni donanım/pin değişikliği — YOK, mevcut pin haritası korunur.

## Doğrulama

- PlatformIO derlemesi (`pio run`) hatasız geçmeli.
- SİT komutu (5B) → UART0'dan 10 Hz 36B `SitPaketi` çıkışı; big-endian + checksum
  doğru.
- SUT komutu → Task1 donanımı atlar, TTL'den enjekte edilen değerlerle uçuş
  algoritması ilerler, 10 Hz durum paketi + doğru latch bitleri.
- DURDUR → anında `MOD_BEKLEME`, bekleyen aktivasyon iptal.
- SUT tezgah testinde (`SUT_FUNYE_YERINE_LED=1`) gerçek fünye pini sürülmez.
