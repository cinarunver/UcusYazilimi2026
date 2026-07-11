# LoRa Paket Küçültme (Fixed-Point) — Implementasyon Planı

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** LoRa telemetri paketlerini fixed-point'e çevirerek küçültmek (roket 59B→33B, görev yükü 48B→28B), böylece RF ayarına dokunmadan gönderim hızını >10 Hz'e çıkarmak ve CRC hatalarını gidermek.

**Architecture:** İç float struct'lar (queue + SD loglama) değişmez. LoRa'ya giderken ayrı bir packed "wire" struct'a quantize edilir. Quantize + çerçeveleme mantığı, `led_durum.h` desenindeki gibi bir header'a konur; hem sketch'e include edilir hem g++ host testiyle test edilir. Yer istasyonu (Python) parse/sim tarafı yeni formata çevrilir.

**Tech Stack:** C++11 (ESP32 Arduino/PlatformIO firmware + native g++ host testleri), Python 3 (PyQt6 yer istasyonu, `struct` modülü).

## Global Constraints

- RF/fiziksel ayarlar **değişmeyecek**: `LORA_SPED=0x1C` (9.6k air-rate), `LORA_OPTION=0x44` (FEC açık), UART 9600. Bu `#define`'lara dokunma.
- Uçuş algoritması ve SD loglama **tam float** kullanmaya devam edecek. Sadece LoRa'ya giden temsil değişir.
- Little-endian korunur (ESP32 native LE; Python `struct` format prefix `'<'`).
- Çerçeve yapısı korunur: `[0xAA][0x55][LEN][payload][CRC_HI][CRC_LO]`, CRC16-CCITT (poly=0x1021, init=0xFFFF), CRC yalnız `payload` üzerinden.
- Roket wire struct format (Python): `'<8hH3h2iB'` = 33B. Görev yükü: `'<HhHh2i6h'` = 28B.
- Ölçekler: ivme ×100, gyro ×10, roll/pitch/eglim ×100, yaw ×100 (uint16), irtifa ×10, dikeyHiz ×10, GPS ×1e7 (int32), basinc hPa×10 (uint16), sicaklik ×100, nem ×100 (uint16).
- **Repoda auto-commit hook'u var:** bazı commit'ler otomatik ("son çivi N") atılabilir. Bir `git commit` "nothing to commit" derse `git log --oneline -3` ile değişikliğin zaten commit'lendiğini doğrula, panik yapma.
- **Co-author ekleme:** commit mesajlarına `Co-Authored-By: Claude` **ekleme** (kullanıcı kuralı).

---

### Task 1: Roket wire header + host test

Quantize + çerçeveleme mantığını tek header'a koy (`src/lora_wire.h`) ve g++ host testiyle doğrula. Bu task firmware'e dokunmaz — saf mantık ve testi.

**Files:**
- Create: `src/lora_wire.h`
- Test (create): `test/host_lora_wire.cpp`

**Interfaces:**
- Produces:
  - `struct TelemetryPacket` (float; queue/SD için — main.cpp buraya taşınacak)
  - `struct TelemetryWire` (packed int; 33B)
  - `int16_t q16(float v, float scale)`, `uint16_t qu16(float v, float scale)`, `int32_t q32(double v, double scale)` — clamp + round quantize yardımcıları
  - `uint16_t crc16_ccitt(const uint8_t* data, size_t len)`
  - `void pack_telemetry_wire(TelemetryWire& w, const TelemetryPacket& p)`
  - `size_t build_rocket_frame(uint8_t* out, const TelemetryPacket& p)` → çerçeve uzunluğu döner (38)

- [ ] **Step 1: Host testini yaz (henüz header yok → derlenmez)**

`test/host_lora_wire.cpp`:

```cpp
// Roket wire paketleme host birim testi (framework yok).
// Calistir: g++ -std=c++11 test/host_lora_wire.cpp -o /tmp/test_wire && /tmp/test_wire
#include "../src/lora_wire.h"
#include <cstdio>
#include <cstring>

static int gecti = 0, kaldi = 0;
#define KONTROL(ad, kosul) do { \
    if (kosul) { gecti++; } \
    else { kaldi++; printf("  [KALDI] %s\n", ad); } \
} while (0)

static TelemetryPacket ornek() {
    TelemetryPacket p; memset(&p, 0, sizeof(p));
    p.ivmeX = 1.23f; p.ivmeY = -2.50f; p.ivmeZ = 98.10f;
    p.gyroX = 12.3f; p.gyroY = -45.6f; p.gyroZ = 300.0f;
    p.roll = -12.34f; p.pitch = 5.00f; p.yaw = 359.99f;
    p.irtifa = 1234.5f; p.dikeyHiz = -12.3f; p.eglimAcisi = 7.77f;
    p.gpsEnlem = 41.0123456f; p.gpsBoylam = 28.9876543f;
    p.ayrilma1_durum = true; p.ayrilma2_durum = false; p.ucus_durumu = 3;
    return p;
}

int main() {
    printf("== Roket wire testleri ==\n");

    // Boyut: wire 33B, cerceve 38B
    KONTROL("wire boyutu 33", sizeof(TelemetryWire) == 33);

    TelemetryWire w; pack_telemetry_wire(w, ornek());
    KONTROL("ivmeX x100", w.ivmeX == 123);
    KONTROL("ivmeZ x100", w.ivmeZ == 9810);
    KONTROL("gyroZ x10",  w.gyroZ == 3000);
    KONTROL("roll x100 negatif", w.roll == -1234);
    KONTROL("yaw uint16 x100", w.yaw == 35999);
    KONTROL("irtifa x10", w.irtifa == 12345);
    KONTROL("dikeyHiz x10 negatif", w.dikeyHiz == -123);
    // GPS kaynagi float32 (~0.3m); tam int esitligi degil, tolerans (double bolme):
    KONTROL("gpsEnlem ~41.0123", fabs((double)w.gpsEnlem / 1e7 - 41.0123456) < 5e-6);
    KONTROL("gpsBoylam ~28.9877", fabs((double)w.gpsBoylam / 1e7 - 28.9876543) < 5e-6);
    KONTROL("durum bitfield", w.durum == (1 | (3 << 2))); // ayrilma1=1, ucus=3

    // Klips: asiri deger int16 sinirinda kirpilir (crash/overflow yok)
    TelemetryPacket big; memset(&big, 0, sizeof(big));
    big.ivmeX = 99999.0f;
    TelemetryWire wb; pack_telemetry_wire(wb, big);
    KONTROL("pozitif klips 32767", wb.ivmeX == 32767);
    big.ivmeX = -99999.0f;
    pack_telemetry_wire(wb, big);
    KONTROL("negatif klips -32768", wb.ivmeX == -32768);

    // Cerceve: uzunluk 38, sync + len dogru, CRC payload uzerinden
    uint8_t f[64];
    size_t n = build_rocket_frame(f, ornek());
    KONTROL("cerceve uzunlugu 38", n == 38);
    KONTROL("sync1 0xAA", f[0] == 0xAA);
    KONTROL("sync2 0x55", f[1] == 0x55);
    KONTROL("len 33", f[2] == 33);
    uint16_t crc_cerceve = ((uint16_t)f[36] << 8) | f[37];
    KONTROL("crc payload ile uyumlu", crc_cerceve == crc16_ccitt(&f[3], 33));

    // CRC bilinen vektor (regresyon)
    KONTROL("crc \"123456789\" = 0x29B1",
            crc16_ccitt((const uint8_t*)"123456789", 9) == 0x29B1);

    printf("\nGECTI: %d  KALDI: %d\n", gecti, kaldi);
    return kaldi == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Testi çalıştır, derleme hatasıyla başarısız olduğunu gör**

Run: `g++ -std=c++11 test/host_lora_wire.cpp -o /tmp/test_wire`
Expected: FAIL — `fatal error: ../src/lora_wire.h: No such file or directory`

- [ ] **Step 3: `src/lora_wire.h` header'ını yaz**

```cpp
#pragma once
// ============================================================
//  LORA WIRE FORMAT (fixed-point) — roket telemetri
//  Ic float struct (queue/SD) DEGISMEZ; LoRa'ya giderken bu
//  packed int wire struct'a quantize edilir. Little-endian.
//  Cerceve: [0xAA][0x55][LEN][wire][CRC_HI][CRC_LO]
// ============================================================
#include <stdint.h>
#include <string.h>
#include <math.h>

// --- Olcek carpanlari (yer istasyonu ters cevirir) ---
#define WIRE_OLCEK_IVME    100.0f   // m/s^2 x100
#define WIRE_OLCEK_GYRO     10.0f   // dps   x10
#define WIRE_OLCEK_ACI     100.0f   // derece x100
#define WIRE_OLCEK_IRTIFA   10.0f   // m     x10
#define WIRE_OLCEK_HIZ      10.0f   // m/s   x10
#define WIRE_OLCEK_GPS      1e7     // derece x1e7 (int32)

// --- Ic float paket (main.cpp buraya tasindi; queue + SD kullanir) ---
#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float irtifa;
    float dikeyHiz;
    float eglimAcisi;
    float gpsEnlem, gpsBoylam;
    bool  ayrilma1_durum;
    bool  ayrilma2_durum;
    uint8_t ucus_durumu;
};

// --- Havadan giden packed wire paket (33B) ---
// Python format: '<8hH3h2iB'
struct TelemetryWire {
    int16_t  ivmeX, ivmeY, ivmeZ;
    int16_t  gyroX, gyroY, gyroZ;
    int16_t  roll, pitch;
    uint16_t yaw;
    int16_t  irtifa, dikeyHiz, eglimAcisi;
    int32_t  gpsEnlem, gpsBoylam;
    uint8_t  durum;   // bit0=ayrilma1, bit1=ayrilma2, bit2-4=ucus_durumu
};
#pragma pack(pop)

// --- Quantize yardimcilari (clamp + round; overflow yok) ---
static inline int16_t q16(float v, float scale) {
    float x = roundf(v * scale);
    if (x >  32767.0f) x =  32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)x;
}
static inline uint16_t qu16(float v, float scale) {
    float x = roundf(v * scale);
    if (x > 65535.0f) x = 65535.0f;
    if (x < 0.0f)     x = 0.0f;
    return (uint16_t)x;
}
static inline int32_t q32(double v, double scale) {
    double x = round(v * scale);
    if (x >  2147483647.0) x =  2147483647.0;
    if (x < -2147483648.0) x = -2147483648.0;
    return (int32_t)x;
}

// --- CRC16-CCITT (poly=0x1021, init=0xFFFF) ---
static inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// --- Float paket -> wire paket ---
static inline void pack_telemetry_wire(TelemetryWire& w, const TelemetryPacket& p) {
    w.ivmeX = q16(p.ivmeX, WIRE_OLCEK_IVME);
    w.ivmeY = q16(p.ivmeY, WIRE_OLCEK_IVME);
    w.ivmeZ = q16(p.ivmeZ, WIRE_OLCEK_IVME);
    w.gyroX = q16(p.gyroX, WIRE_OLCEK_GYRO);
    w.gyroY = q16(p.gyroY, WIRE_OLCEK_GYRO);
    w.gyroZ = q16(p.gyroZ, WIRE_OLCEK_GYRO);
    w.roll  = q16(p.roll,  WIRE_OLCEK_ACI);
    w.pitch = q16(p.pitch, WIRE_OLCEK_ACI);
    w.yaw   = qu16(p.yaw,  WIRE_OLCEK_ACI);
    w.irtifa     = q16(p.irtifa,     WIRE_OLCEK_IRTIFA);
    w.dikeyHiz   = q16(p.dikeyHiz,   WIRE_OLCEK_HIZ);
    w.eglimAcisi = q16(p.eglimAcisi, WIRE_OLCEK_ACI);
    w.gpsEnlem  = q32(p.gpsEnlem,  WIRE_OLCEK_GPS);
    w.gpsBoylam = q32(p.gpsBoylam, WIRE_OLCEK_GPS);
    w.durum = (p.ayrilma1_durum ? 0x01 : 0)
            | (p.ayrilma2_durum ? 0x02 : 0)
            | ((p.ucus_durumu & 0x07) << 2);
}

// --- Wire paketi cerceveler; cerceve uzunlugu doner (38) ---
static inline size_t build_rocket_frame(uint8_t* out, const TelemetryPacket& p) {
    TelemetryWire w;
    pack_telemetry_wire(w, p);
    const uint8_t* payload = (const uint8_t*)&w;
    const size_t   len     = sizeof(TelemetryWire);
    uint16_t       crc     = crc16_ccitt(payload, len);
    size_t idx = 0;
    out[idx++] = 0xAA;
    out[idx++] = 0x55;
    out[idx++] = (uint8_t)len;
    memcpy(&out[idx], payload, len);
    idx += len;
    out[idx++] = (uint8_t)(crc >> 8);
    out[idx++] = (uint8_t)(crc & 0xFF);
    return idx;
}
```

- [ ] **Step 4: Testi çalıştır, geçtiğini gör**

Run: `g++ -std=c++11 test/host_lora_wire.cpp -o /tmp/test_wire && /tmp/test_wire`
Expected: PASS — `GECTI: 19  KALDI: 0`, çıkış kodu 0

- [ ] **Step 5: Commit**

```bash
git add src/lora_wire.h test/host_lora_wire.cpp
git commit -m "feat(lora): roket fixed-point wire format header + host test"
```

---

### Task 2: Roket wire'ı main.cpp'ye entegre et + gönderim hızı

Firmware'i yeni header'ı kullanacak şekilde bağla: yerel `TelemetryPacket` ve `crc16_ccitt` tanımlarını kaldır, LoRa çerçevesini wire'dan kur, gönderim oranını 8'e çek (~12.5 Hz).

**Files:**
- Modify: `src/main.cpp` (struct silme, include, crc silme, `gonder_paket_framed_dma`, `LORA_GONDERIM_ORANI`)

**Interfaces:**
- Consumes: `TelemetryPacket`, `build_rocket_frame` (Task 1, `src/lora_wire.h`)

- [ ] **Step 1: Header'ı include et, yerel `TelemetryPacket` tanımını kaldır**

`src/main.cpp` içinde `#include "led_durum.h"` satırının hemen altına ekle:

```cpp
#include "lora_wire.h"       // TelemetryPacket (float) + fixed-point wire format
```

Sonra mevcut yerel struct tanımını (yaklaşık satır 330–343) **sil**:

```cpp
// SİLİNECEK BLOK (artik lora_wire.h icinde):
#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    ...
    uint8_t ucus_durumu;
};
#pragma pack(pop)
```

Not: `TelemetryPacket` üstündeki açıklama yorumu kalabilir; yalnız `#pragma pack ... struct ... pop` bloğunu sil.

- [ ] **Step 2: Yerel `crc16_ccitt` tanımını kaldır**

`src/main.cpp` içindeki `crc16_ccitt` fonksiyonunu (yaklaşık satır 460–471, `--- CRC16-CCITT ...` yorumu dahil) **sil** — artık `lora_wire.h`'den geliyor:

```cpp
// SİLİNECEK:
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    ...
    return crc;
}
```

- [ ] **Step 3: `gonder_paket_framed_dma`'yı wire çerçevesi kullanacak şekilde değiştir**

`src/main.cpp` içindeki fonksiyonun gövdesini şununla değiştir:

```cpp
void gonder_paket_framed_dma(uart_port_t uart_num, const TelemetryPacket& pkt) {
    static uint8_t frame_buf[64]; // wire cerceve 38B; 64 fazlasiyla yeter
    size_t idx = build_rocket_frame(frame_buf, pkt);
    // uart_write_bytes: veriyi ring buffer'a atar ve hemen doner (DMA-like).
    uart_write_bytes(uart_num, (const char*)frame_buf, idx);
}
```

- [ ] **Step 4: Gönderim oranını 8'e çek (~12.5 Hz)**

`src/main.cpp` içindeki `LORA_GONDERIM_ORANI` satırını değiştir:

```cpp
// --- LORA GÖNDERİM HIZI ---
// Wire cerceve 38B @ 9600 baud, FEC acik hava tavani ~500-700 B/s.
// 100 Hz queue ÷ 8 ≈ 12.5 Hz → ~475 B/s (guvenli, >10 Hz).
// Saha testi temizse 7 (~14 Hz) / 6 (~16.7 Hz) denenebilir.
#define LORA_GONDERIM_ORANI    8
```

- [ ] **Step 5: Firmware'in derlendiğini doğrula**

Run: `pio run -e ucus 2>&1 | tail -20`
Expected: `SUCCESS` (derleme hatası yok). Toolchain/PlatformIO yoksa bu adımı atla ve host testine (Task 1) güven; ama mümkünse derlemeyi çalıştır.

- [ ] **Step 6: Commit**

```bash
git add src/main.cpp
git commit -m "feat(lora): main.cpp fixed-point wire cerceve + 12.5 Hz gonderim"
```

---

### Task 3: Görev yükü wire header + host test

Roketin aynısını görev yükü için yap: quantize + çerçeveleme header'ı + host testi.

**Files:**
- Create: `GorevYukuYazilimi/lora_wire_bgy.h`
- Test (create): `GorevYukuYazilimi/host_lora_wire_bgy.cpp`

**Interfaces:**
- Produces:
  - `struct GorevYukuPaket` (float; queue/SD — gorevyuku.cpp buraya taşınacak)
  - `struct GorevYukuWire` (packed int; 28B)
  - `q16/qu16/q32/crc16_ccitt` (roket header'ıyla birebir aynı mantık, ayrı translation unit)
  - `void pack_gorevyuku_wire(GorevYukuWire& w, const GorevYukuPaket& p)`
  - `size_t build_payload_frame(uint8_t* out, const GorevYukuPaket& p)` → çerçeve uzunluğu (33)

- [ ] **Step 1: Host testini yaz (header yok → derlenmez)**

`GorevYukuYazilimi/host_lora_wire_bgy.cpp`:

```cpp
// Gorev yuku wire paketleme host birim testi (framework yok).
// Calistir: g++ -std=c++11 GorevYukuYazilimi/host_lora_wire_bgy.cpp -o /tmp/test_bgy_wire && /tmp/test_bgy_wire
#include "lora_wire_bgy.h"
#include <cstdio>
#include <cstring>

static int gecti = 0, kaldi = 0;
#define KONTROL(ad, kosul) do { \
    if (kosul) { gecti++; } else { kaldi++; printf("  [KALDI] %s\n", ad); } \
} while (0)

int main() {
    printf("== Gorev yuku wire testleri ==\n");
    KONTROL("wire boyutu 28", sizeof(GorevYukuWire) == 28);

    GorevYukuPaket p; memset(&p, 0, sizeof(p));
    p.basinc = 1013.25f;   // hPa
    p.sicaklik = 23.45f;
    p.nem = 55.5f;
    p.irtifa = 1234.5f;
    p.gpsEnlem = 41.0123456f; p.gpsBoylam = 28.9876543f;
    p.ivmeX = 1.23f; p.ivmeZ = 98.10f;
    p.gyroZ = 300.0f;

    GorevYukuWire w; pack_gorevyuku_wire(w, p);
    KONTROL("basinc hPa x10", w.basinc == 10133);   // round(1013.25*10)=10133
    KONTROL("sicaklik x100", w.sicaklik == 2345);
    KONTROL("nem x100", w.nem == 5550);
    KONTROL("irtifa x10", w.irtifa == 12345);
    // GPS kaynagi float32; tolerans:
    KONTROL("gpsEnlem ~41.0123", fabs((double)w.gpsEnlem / 1e7 - 41.0123456) < 5e-6);
    KONTROL("ivmeX x100", w.ivmeX == 123);
    KONTROL("gyroZ x10", w.gyroZ == 3000);

    uint8_t f[64];
    size_t n = build_payload_frame(f, p);
    KONTROL("cerceve uzunlugu 33", n == 33);
    KONTROL("sync1 0xAA", f[0] == 0xAA);
    KONTROL("len 28", f[2] == 28);
    uint16_t crc = ((uint16_t)f[31] << 8) | f[32];
    KONTROL("crc payload ile uyumlu", crc == crc16_ccitt(&f[3], 28));

    printf("\nGECTI: %d  KALDI: %d\n", gecti, kaldi);
    return kaldi == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Testi çalıştır, derleme hatasıyla başarısız olduğunu gör**

Run: `g++ -std=c++11 GorevYukuYazilimi/host_lora_wire_bgy.cpp -o /tmp/test_bgy_wire`
Expected: FAIL — `fatal error: lora_wire_bgy.h: No such file or directory`

- [ ] **Step 3: `GorevYukuYazilimi/lora_wire_bgy.h` header'ını yaz**

```cpp
#pragma once
// ============================================================
//  LORA WIRE FORMAT (fixed-point) — gorev yuku telemetri
//  Ic float struct (queue/SD) DEGISMEZ; LoRa'ya giderken bu
//  packed int wire struct'a quantize edilir. Little-endian.
//  Cerceve: [0xAA][0x55][LEN][wire][CRC_HI][CRC_LO]
// ============================================================
#include <stdint.h>
#include <string.h>
#include <math.h>

#define WIRE_OLCEK_BASINC  10.0f    // hPa x10 (uint16)
#define WIRE_OLCEK_SICAK  100.0f    // C   x100
#define WIRE_OLCEK_NEM    100.0f    // %   x100 (uint16)
#define WIRE_OLCEK_IRTIFA  10.0f    // m   x10
#define WIRE_OLCEK_IVME   100.0f    // m/s^2 x100
#define WIRE_OLCEK_GYRO    10.0f    // dps x10
#define WIRE_OLCEK_GPS     1e7      // derece x1e7 (int32)

#pragma pack(push, 1)
struct GorevYukuPaket {
    float basinc, sicaklik, nem, irtifa;   // BME280 (basinc hPa)
    float gpsEnlem, gpsBoylam;             // GPS
    float ivmeX, ivmeY, ivmeZ;             // BNO055 lineer ivme
    float gyroX, gyroY, gyroZ;             // BNO055 gyro
};

// Havadan giden packed wire paket (28B). Python format: '<HhHh2i6h'
struct GorevYukuWire {
    uint16_t basinc;
    int16_t  sicaklik;
    uint16_t nem;
    int16_t  irtifa;
    int32_t  gpsEnlem, gpsBoylam;
    int16_t  ivmeX, ivmeY, ivmeZ;
    int16_t  gyroX, gyroY, gyroZ;
};
#pragma pack(pop)

static inline int16_t q16(float v, float scale) {
    float x = roundf(v * scale);
    if (x >  32767.0f) x =  32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    return (int16_t)x;
}
static inline uint16_t qu16(float v, float scale) {
    float x = roundf(v * scale);
    if (x > 65535.0f) x = 65535.0f;
    if (x < 0.0f)     x = 0.0f;
    return (uint16_t)x;
}
static inline int32_t q32(double v, double scale) {
    double x = round(v * scale);
    if (x >  2147483647.0) x =  2147483647.0;
    if (x < -2147483648.0) x = -2147483648.0;
    return (int32_t)x;
}

static inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static inline void pack_gorevyuku_wire(GorevYukuWire& w, const GorevYukuPaket& p) {
    w.basinc   = qu16(p.basinc,   WIRE_OLCEK_BASINC);
    w.sicaklik = q16(p.sicaklik,  WIRE_OLCEK_SICAK);
    w.nem      = qu16(p.nem,      WIRE_OLCEK_NEM);
    w.irtifa   = q16(p.irtifa,    WIRE_OLCEK_IRTIFA);
    w.gpsEnlem  = q32(p.gpsEnlem,  WIRE_OLCEK_GPS);
    w.gpsBoylam = q32(p.gpsBoylam, WIRE_OLCEK_GPS);
    w.ivmeX = q16(p.ivmeX, WIRE_OLCEK_IVME);
    w.ivmeY = q16(p.ivmeY, WIRE_OLCEK_IVME);
    w.ivmeZ = q16(p.ivmeZ, WIRE_OLCEK_IVME);
    w.gyroX = q16(p.gyroX, WIRE_OLCEK_GYRO);
    w.gyroY = q16(p.gyroY, WIRE_OLCEK_GYRO);
    w.gyroZ = q16(p.gyroZ, WIRE_OLCEK_GYRO);
}

static inline size_t build_payload_frame(uint8_t* out, const GorevYukuPaket& p) {
    GorevYukuWire w;
    pack_gorevyuku_wire(w, p);
    const uint8_t* payload = (const uint8_t*)&w;
    const size_t   len     = sizeof(GorevYukuWire);
    uint16_t       crc     = crc16_ccitt(payload, len);
    size_t idx = 0;
    out[idx++] = 0xAA;
    out[idx++] = 0x55;
    out[idx++] = (uint8_t)len;
    memcpy(&out[idx], payload, len);
    idx += len;
    out[idx++] = (uint8_t)(crc >> 8);
    out[idx++] = (uint8_t)(crc & 0xFF);
    return idx;
}
```

- [ ] **Step 4: Testi çalıştır, geçtiğini gör**

Run: `g++ -std=c++11 GorevYukuYazilimi/host_lora_wire_bgy.cpp -o /tmp/test_bgy_wire && /tmp/test_bgy_wire`
Expected: PASS — `GECTI: 12  KALDI: 0`, çıkış kodu 0

- [ ] **Step 5: Commit**

```bash
git add GorevYukuYazilimi/lora_wire_bgy.h GorevYukuYazilimi/host_lora_wire_bgy.cpp
git commit -m "feat(bgy): gorev yuku fixed-point wire format header + host test"
```

---

### Task 4: Görev yükü wire'ı gorevyuku.cpp'ye entegre et

**Files:**
- Modify: `GorevYukuYazilimi/gorevyuku.cpp` (struct silme, include, crc silme, `gonder_paket_framed_dma`)

**Interfaces:**
- Consumes: `GorevYukuPaket`, `build_payload_frame` (Task 3, `lora_wire_bgy.h`)

- [ ] **Step 1: Header'ı include et, yerel `GorevYukuPaket` tanımını kaldır**

`GorevYukuYazilimi/gorevyuku.cpp`'de dosyanın include bloğuna ekle:

```cpp
#include "lora_wire_bgy.h"   // GorevYukuPaket (float) + fixed-point wire format
```

Sonra yerel struct tanımını (yaklaşık satır 161–169, `--- GOREV YUKU TELEMETRI PAKETI ...` yorumu altındaki `#pragma pack ... struct GorevYukuPaket ... pop` bloğu) **sil**.

- [ ] **Step 2: Yerel `crc16_ccitt` tanımını kaldır**

`GorevYukuYazilimi/gorevyuku.cpp` içindeki `crc16_ccitt` fonksiyonunu (yaklaşık satır 182–191) **sil** — artık `lora_wire_bgy.h`'den geliyor.

- [ ] **Step 3: `gonder_paket_framed_dma`'yı wire çerçevesi kullanacak şekilde değiştir**

`GorevYukuYazilimi/gorevyuku.cpp` içindeki fonksiyon gövdesini değiştir:

```cpp
void gonder_paket_framed_dma(uart_port_t uart_num, const GorevYukuPaket& pkt) {
    static uint8_t frame_buf[64]; // wire cerceve 33B; 64 fazlasiyla yeter
    size_t idx = build_payload_frame(frame_buf, pkt);
    uart_write_bytes(uart_num, (const char*)frame_buf, idx);
}
```

- [ ] **Step 4: Firmware'in derlendiğini doğrula**

Run: `pio run -e gorevyuku 2>&1 | tail -20`
Expected: `SUCCESS`. PlatformIO yoksa atla ve host testine (Task 3) güven.

- [ ] **Step 5: Commit**

```bash
git add GorevYukuYazilimi/gorevyuku.cpp
git commit -m "feat(bgy): gorevyuku.cpp fixed-point wire cerceve"
```

---

### Task 5: Yer istasyonu (Python) parse + simülasyon revizesi

Yer istasyonunu yeni formatlara çevir: format string'ler, parse fonksiyonları (int → ölçekle böl), ve kendi simülasyon gönderici (float → quantize → pack). Round-trip test scriptiyle doğrula.

**Files:**
- Modify: `../YerIstasyonu26/YerIstasyonu2026.py` (format sabitleri, `parse_rocket_frame`, `parse_payload_frame`, simülasyon pack blokları)
- Test (create): `../YerIstasyonu26/test_paket_roundtrip.py`

Not: Bu dosyalar mevcut repo dizininin **dışında** (`../YerIstasyonu26/`). Yollar oradaki repoya göre. Ayrı git deposu olabilir; commit'i o dizinde yap.

**Interfaces:**
- Consumes: firmware wire layout (Task 1 & 3) — format `'<8hH3h2iB'` (roket), `'<HhHh2i6h'` (görev yükü) ve aynı ölçekler.

- [ ] **Step 1: Round-trip testini yaz (henüz format eski → başarısız)**

`../YerIstasyonu26/test_paket_roundtrip.py`:

```python
# Firmware wire format <-> yer istasyonu parse round-trip testi.
# Calistir: python3 test_paket_roundtrip.py   (YerIstasyonu26/ dizininde)
import struct

ROCKET_FORMAT = '<8hH3h2iB'
PAYLOAD_FORMAT = '<HhHh2i6h'

def test_boyutlar():
    assert struct.calcsize(ROCKET_FORMAT) == 33, struct.calcsize(ROCKET_FORMAT)
    assert struct.calcsize(PAYLOAD_FORMAT) == 28, struct.calcsize(PAYLOAD_FORMAT)

def test_roket_olcek():
    # firmware quantize esdegeri: irtifa x10, ivme x100, gps x1e7, yaw x100
    raw = struct.pack(ROCKET_FORMAT,
        123, -250, 9810,      # ivme x100
        123, -456, 3000,      # gyro x10
        -1234, 500,           # roll,pitch x100
        35999,                # yaw x100 (uint16)
        12345, -123, 777,     # irtifa x10, dikeyHiz x10, eglim x100
        410123456, 289876543, # gps x1e7
        (1 | (3 << 2)))       # durum: ayrilma1=1, ucus=3
    v = struct.unpack(ROCKET_FORMAT, raw)
    assert abs(v[9] / 10.0 - 1234.5) < 1e-6
    assert abs(v[0] / 100.0 - 1.23) < 1e-6
    assert abs(v[8] / 100.0 - 359.99) < 1e-6
    assert abs(v[12] / 1e7 - 41.0123456) < 1e-6
    durum = v[16]
    assert bool(durum & 1) is True
    assert bool(durum & 2) is False
    assert (durum >> 2) & 0x07 == 3

def test_gorevyuku_olcek():
    raw = struct.pack(PAYLOAD_FORMAT,
        10133, 2345, 5550, 12345,   # basinc hPa x10, sicaklik x100, nem x100, irtifa x10
        410123456, 289876543,       # gps x1e7
        123, 0, 9810, 0, 0, 3000)   # ivme x100, gyro x10
    v = struct.unpack(PAYLOAD_FORMAT, raw)
    assert abs(v[0] / 10.0 - 1013.3) < 1e-6
    assert abs(v[3] / 10.0 - 1234.5) < 1e-6
    assert abs(v[11] / 10.0 - 300.0) < 1e-6

if __name__ == '__main__':
    test_boyutlar(); test_roket_olcek(); test_gorevyuku_olcek()
    print('OK: tum round-trip testleri gecti')
```

- [ ] **Step 2: Testi çalıştır, geçtiğini gör (bu test self-contained)**

Run: `cd ../YerIstasyonu26 && python3 test_paket_roundtrip.py`
Expected: PASS — `OK: tum round-trip testleri gecti`

Not: Bu test firmware ölçekleriyle byte-uyumunu bağımsız doğrular (drift kalkanı). Şimdi GUI dosyasını bu formatlara çevireceğiz.

- [ ] **Step 3: Format sabitlerini değiştir**

`../YerIstasyonu26/YerIstasyonu2026.py`'de değiştir:

```python
ROCKET_PACKET_FORMAT = '<8hH3h2iB'   # fixed-point wire (33B)
ROCKET_PACKET_SIZE = struct.calcsize(ROCKET_PACKET_FORMAT) # 33 byte
ROCKET_FRAME_SIZE = 2 + 1 + ROCKET_PACKET_SIZE + 2 # 38 byte
```

```python
PAYLOAD_PACKET_FORMAT = '<HhHh2i6h'  # fixed-point wire (28B)
PAYLOAD_PACKET_SIZE = struct.calcsize(PAYLOAD_PACKET_FORMAT) # 28 byte
PAYLOAD_FRAME_SIZE = 2 + 1 + PAYLOAD_PACKET_SIZE + 2 # 33 byte
```

- [ ] **Step 4: `parse_rocket_frame`'in dict dönüşünü ölçekli hale getir**

`return { ... }` bloğunu şununla değiştir (CRC/SYNC/LEN kontrolleri **aynı kalır**):

```python
    v = struct.unpack(ROCKET_PACKET_FORMAT, payload)
    durum = v[16]
    return {
        'ivmeX': v[0] / 100.0, 'ivmeY': v[1] / 100.0, 'ivmeZ': v[2] / 100.0,
        'gyroX': v[3] / 10.0,  'gyroY': v[4] / 10.0,  'gyroZ': v[5] / 10.0,
        'roll':  v[6] / 100.0, 'pitch': v[7] / 100.0, 'yaw':   v[8] / 100.0,
        'irtifa':     v[9]  / 10.0,
        'dikeyHiz':   v[10] / 10.0,
        'eglimAcisi': v[11] / 100.0,
        'gpsEnlem':   v[12] / 1e7,
        'gpsBoylam':  v[13] / 1e7,
        'ayrilma1_durum': bool(durum & 0x01),
        'ayrilma2_durum': bool(durum & 0x02),
        'ucus_durumu':    (durum >> 2) & 0x07,
    }
```

- [ ] **Step 5: `parse_payload_frame`'in dict dönüşünü ölçekli hale getir**

```python
    v = struct.unpack(PAYLOAD_PACKET_FORMAT, payload)
    return {
        'basinc':      v[0] / 10.0,     # hPa
        'bmeSicaklik': v[1] / 100.0,
        'nem':         v[2] / 100.0,
        'irtifa':      v[3] / 10.0,
        'gpsEnlem':    v[4] / 1e7,
        'gpsBoylam':   v[5] / 1e7,
        'ivmeX': v[6] / 100.0, 'ivmeY': v[7] / 100.0, 'ivmeZ': v[8] / 100.0,
        'gyroX': v[9] / 10.0,  'gyroY': v[10] / 10.0, 'gyroZ': v[11] / 10.0,
    }
```

- [ ] **Step 6: Simülasyon gönderici pack bloklarını quantize et**

`if self.identifier == "rocket":` altındaki `payload = struct.pack(ROCKET_PACKET_FORMAT, ...)` çağrısını değiştir:

```python
                payload = struct.pack(ROCKET_PACKET_FORMAT,
                    0, 0, int(round(acc * 100)),                 # ivme x100
                    0, 0, 0,                                     # gyro x10
                    int(round(r * 100)), int(round(p * 100)),    # roll,pitch x100
                    int(round((yaw_a % 360) * 100)),             # yaw x100 (uint16)
                    int(round(sent_alt * 10)),                   # irtifa x10
                    int(round(vel * 10)),                        # dikeyHiz x10
                    int(round((p % 90) * 100)),                  # eglim x100
                    int(round(lat * 1e7)), int(round(lon * 1e7)),# gps x1e7
                    (int(ayrilma1) & 1) | ((int(ayrilma2) & 1) << 1) | ((int(ucus_durumu) & 0x07) << 2)
                )
```

`else:` (görev yükü) altındaki `payload = struct.pack(PAYLOAD_PACKET_FORMAT, ...)` çağrısını değiştir:

```python
                gy_ivmeZ = acc
                basinc_hpa = 1013.25 - sent_alt * 0.12
                sicaklik_c = 25.0 - sent_alt * 0.006
                payload = struct.pack(PAYLOAD_PACKET_FORMAT,
                    int(round(basinc_hpa * 10)),                 # basinc hPa x10 (uint16)
                    int(round(sicaklik_c * 100)),                # sicaklik x100
                    int(round(45.0 * 100)),                      # nem x100 (uint16)
                    int(round(sent_alt * 10)),                   # irtifa x10
                    int(round(lat * 1e7)), int(round(lon * 1e7)),# gps x1e7
                    0, 0, int(round(gy_ivmeZ * 100)),            # ivme x100
                    0, 0, 0                                       # gyro x10
                )
```

- [ ] **Step 7: Round-trip testini tekrar çalıştır (regresyon)**

Run: `cd ../YerIstasyonu26 && python3 test_paket_roundtrip.py`
Expected: PASS — `OK: tum round-trip testleri gecti`

- [ ] **Step 8: GUI'nin import/syntax hatası olmadan yüklendiğini doğrula**

Run: `cd ../YerIstasyonu26 && python3 -c "import ast; ast.parse(open('YerIstasyonu2026.py', encoding='utf-8').read()); print('syntax OK')"`
Expected: PASS — `syntax OK` (PyQt6 kurulu değilse tam import başarısız olabilir; syntax kontrolü yeterli).

- [ ] **Step 9: Commit (YerIstasyonu26 dizininde)**

```bash
cd ../YerIstasyonu26 && git add YerIstasyonu2026.py test_paket_roundtrip.py && \
  git commit -m "feat(yeristasyonu): fixed-point wire parse + simulasyon revizesi (roket 33B, gorev yuku 28B)"
```

Not: `../YerIstasyonu26` git deposu değilse veya ayrı yönetiliyorsa, commit atlanır; kullanıcıya bildir.

---

## Self-Review Notları

- **Spec kapsamı:** Roket wire (Task 1-2), görev yükü wire (Task 3-4), yer istasyonu parse+sim (Task 5), gönderim hızı (Task 2 Step 4), host testleri (Task 1/3), Python round-trip (Task 5) — spec'in her maddesi bir task'a bağlı.
- **Format tutarlılığı:** Roket `'<8hH3h2iB'`=33B, görev yükü `'<HhHh2i6h'`=28B; C++ struct alan sırası Python format sırasıyla birebir. `q16/qu16/q32` ölçekleri C++ header'ları ve Python parse/sim arasında birebir.
- **Kapsam dışı (dokunulmayacak):** RF define'ları (SPED/OPTION/baud); iç float struct'ların SD/uçuş kullanımı; `test/test_ucus/test_main.cpp`'deki mevcut float-struct Unity testleri (kendi kopyalarını test eder, geçmeye devam eder — yeni wire yolu host testleriyle kaplanır).
- **Doğrulanacak varsayım:** gyro birimi deg/s kabul edildi (×10). Task 2/4 derleme sonrası, gerçek uçuş verisinde gyro rad/s çıkarsa `WIRE_OLCEK_GYRO` header'da ×100'e çekilir + Python parse böleni 100'e güncellenir (tek satır, iki yerde).
```
