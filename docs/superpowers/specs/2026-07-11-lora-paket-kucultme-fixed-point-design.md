# LoRa Paket Küçültme (Fixed-Point) — Tasarım

**Tarih:** 2026-07-11
**Durum:** Onaylandı (implementasyon planı bekleniyor)

## Problem

LoRa telemetri linki (E32-433T30D, 9.6k air-rate, FEC açık) hava tavanına
(~500-700 B/s efektif) dayanmış durumda. Roket çerçevesi 64B, ~10 Hz'de
gönderiliyor ve modülün iç TX buffer'ı periyodik taşıp paket ortasında byte
düşürüyor → yer istasyonunda **CRC hataları**. Kullanıcı gönderim hızını
10 Hz'in üstüne çıkarmak istiyor ama **RF/fiziksel parametrelere (SPED, OPTION,
baud) dokunulamıyor**. Tek kaldıraç: paket boyutunu küçültmek.

## Kısıtlar

- RF/fiziksel ayarlar (air-rate 9.6k, FEC açık, UART 9600) **değişmeyecek**.
- Uçuş güvenliği etkilenmeyecek: apogee/ayrılma kararları onboard **tam float**
  ile alınıyor; quantize yalnızca havadan giden byte'ları etkiler.
- SD kara-kutu loglaması **tam float** (CSV) kalacak — hassasiyet kaybı olmayacak.
- Little-endian korunacak (ESP32 native LE; yer istasyonu Python `'<'`).
- CRC16-CCITT (poly=0x1021, init=0xFFFF), SYNC `0xAA 0x55`, `[SYNC][SYNC][LEN]
  [payload][CRC_HI][CRC_LO]` çerçeve yapısı ve resync mantığı **korunacak**.

## Mimari Karar

İç struct'lar (`TelemetryPacket`, `GorevYukuPaket`) **float olarak kalır** ve
queue + SD loglama bunları kullanır. LoRa'ya giderken ayrı bir **packed "wire"
struct**'a quantize edilir. Böylece tek değişen şey havadan giden temsil olur.

- `TelemetryPacket` (float) → `pack_wire()` → `TelemetryWire` (packed) → çerçeve
- `GorevYukuPaket` (float) → `pack_wire()` → `GorevYukuWire` (packed) → çerçeve

SD ve uçuş algoritması dokunulmadan tam float kullanmaya devam eder.

## Wire Format — Roket (UKB / src/main.cpp)

Eski: `TelemetryPacket` = 14×float + 3×byte = **59B**, çerçeve **64B**,
Python format `'<14f3B'`.

Yeni `TelemetryWire` (packed, little-endian):

| Alan | Tip | Ölçek | Byte | Not |
|---|---|---|---|---|
| ivmeX, ivmeY, ivmeZ | int16 | ×100 | 6 | ±327 m/s² (~±33g) |
| gyroX, gyroY, gyroZ | int16 | ×10 | 6 | ±3276 dps (deg/s varsayımı) |
| roll, pitch | int16 | ×100 | 4 | ±327° |
| yaw | uint16 | ×100 | 2 | 0–655° (BNO heading 0-360) |
| irtifa | int16 | ×10 | 2 | 0.1 m, ±3276 m |
| dikeyHiz | int16 | ×10 | 2 | 0.1 m/s, ±3276 m/s |
| eglimAcisi | int16 | ×100 | 2 | 0–180° |
| gpsEnlem | int32 | ×1e7 | 4 | ~1 cm |
| gpsBoylam | int32 | ×1e7 | 4 | ~1 cm |
| durum | uint8 | bitfield | 1 | bit0=ayrilma1, bit1=ayrilma2, bit2-4=ucus_durumu (0-4) |

**Toplam payload: 33B**, çerçeve **38B** (−41%).
Python struct format: `'<8hH3h2iB'` (= 33B).

Alan sırası (struct içi): ivmeX,Y,Z, gyroX,Y,Z, roll, pitch, yaw, irtifa,
dikeyHiz, eglimAcisi, gpsEnlem, gpsBoylam, durum.

## Wire Format — Görev Yükü (GorevYukuYazilimi/gorevyuku.cpp)

Eski: `GorevYukuPaket` = 12×float = **48B**, çerçeve **53B**,
Python format `'<12f'`.

Yeni `GorevYukuWire` (packed, little-endian):

| Alan | Tip | Ölçek | Byte | Not |
|---|---|---|---|---|
| basinc | uint16 | hPa×10 | 2 | 0.1 hPa çöz., 0–6553 hPa (firmware basinc'i hPa tutar) |
| sicaklik | int16 | ×100 | 2 | °C |
| nem | uint16 | ×100 | 2 | %RH |
| irtifa | int16 | ×10 | 2 | 0.1 m |
| gpsEnlem | int32 | ×1e7 | 4 | ~1 cm |
| gpsBoylam | int32 | ×1e7 | 4 | ~1 cm |
| ivmeX, ivmeY, ivmeZ | int16 | ×100 | 6 | m/s² |
| gyroX, gyroY, gyroZ | int16 | ×10 | 6 | dps |

**Toplam payload: 28B**, çerçeve **33B** (−38%).
Python struct format: `'<HhHh2i6h'` (= 28B).

## Gönderim Hızı

RF'e dokunmadan, roket 38B çerçeve ile ~12.5 Hz güvenli:

- `LORA_GONDERIM_ORANI = 8` → 100 Hz queue ÷ 8 ≈ **12.5 Hz** → ~475 B/s (güvenli, >10 ✓)
- Saha testi temizse `=7` (~14 Hz) / `=6` (~16.7 Hz) diye indirilebilir — tek satır tunable.

Not: Görev yükü ayrı kanal/modül; kendi `LORA_GONDERIM_ORANI`'sı da aynı mantıkla
ayarlanır (33B çerçeve daha da rahat).

## Yer İstasyonu (../YerIstasyonu26/YerIstasyonu2026.py)

- `ROCKET_PACKET_FORMAT` → `'<8hH3h2iB'`; `PAYLOAD_PACKET_FORMAT` → `'<HhHh2i6h'`.
- `parse_rocket_frame` / `parse_payload_frame`: `struct.unpack` ham int'leri
  döndürür; her alan ilgili ölçeğe **bölünerek** float'a geri çevrilir, sonuç
  dict'i (GUI'nin beklediği anahtarlar) aynı kalır. `durum` byte'ı bit-maskeyle
  ayrilma1/ayrilma2/ucus_durumu'na açılır.
- Kendi kendine test/simülasyon pack tarafı (mevcut ~satır 415-444) yeni
  formatlara ve ölçeklere göre güncellenir (float → int quantize → pack).
- CRC16-CCITT, SYNC, LEN kontrolü ve resync döngüsü **değişmez**; sadece
  `*_PACKET_SIZE` / `*_FRAME_SIZE` `struct.calcsize` üzerinden otomatik güncellenir.

## Test / Doğrulama

- **Host birim testi (C++):** `pack_wire()` round-trip — bilinen float
  değerleri → wire → beklenen int; ve ölçek sınırlarında klips davranışı.
  Mevcut `test/test_ucus/` host test altyapısına eklenir.
- **Python round-trip:** aynı örnek değerlerle firmware `pack` ↔ Python
  `parse` byte-uyumu (tercihen ortak bir örnek vektör).
- **Manuel:** yer istasyonunun kendi simülasyon göndericisi ile parse edip
  GUI'de değerlerin makul göründüğü doğrulanır.

## Kapsam Dışı (YAGNI)

- Bit-packing (int16 altı) yapılmayacak — int16/int32 yeterli kazanç sağlıyor.
- RF parametre değişikliği (air-rate/FEC) yapılmayacak.
- Çerçeve/CRC/SYNC protokolü yeniden tasarlanmayacak.

## Doğrulanacak Varsayımlar

- **gyro birimi**: deg/s varsayıldı (×10 → ±3276 dps güvenli). Kodda rad/s
  çıkarsa gyro ölçeği ×100'e çekilir (implementasyon sırasında teyit edilecek).
- **irtifa ×10** 3276 m'de klipslenir; hedef apogee bunun altında. Üstündeyse
  ölçek ×5'e düşürülür (yalnız gösterge; uçuş kararı float).
