# SİT/SUT Main Entegrasyonu Implementasyon Planı

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SİT/SUT test yeteneğini, güncel optimize `src/main.cpp` içine — DMA LoRa, ping-pong SD ve slim telemetri paketi optimizasyonlarını bozmadan — gömmek.

**Architecture:** LoRa telemetrisi olduğu gibi (IDF DMA sürücüsü, UART1) kalır. SİT/SUT test protokolü ayrı bir kanala, `Serial`/UART0 @115200'e, adanır: komut alma + SİT telemetri paketi (36B) + SUT durum paketi (6B). Uçuş döngüsü slim kalır; basınç yalnızca SİT modunda okunur. SUT modunda donanım okuması atlanır, değerler TTL'den globallere enjekte edilir.

**Tech Stack:** PlatformIO, Arduino-ESP32, FreeRTOS (çift çekirdek task), ESP-IDF UART sürücüsü.

## Global Constraints

- Hedef ve tek çıktı dosyası: `src/main.cpp`. Başka dosya değişmez; `SİT_SUT/SİT-SUT.cpp` referans olarak silinmeden kalır.
- Derleme ortamı: `pio run -e ucus` (build_src_filter yalnız `src/` derler).
- LoRa transportu **değişmez**: IDF DMA sürücüsü + `gonder_paket_framed_dma` + E32 M0/M1 config. Bloklayan `Serial1`'e DÖNÜLMEZ.
- Slim `TelemetryPacket` **değişmez** (basinc/sicaklik/nem LoRa paketine EKLENMEZ).
- Test-modu onay/debug mesajları **hiç gönderilmez** (ne LoRa ne TTL). Referanstaki `Serial1.println("[SIT]...")` çağrıları port EDİLMEZ.
- Basınç yalnızca `sitSutMod == MOD_SIT` iken okunur.
- Güvenlik bayrağı `SUT_FUNYE_YERINE_LED 1` korunur: SUT tezgah testinde gerçek fünye pini sürülmez, gösterge LED yanar. SİT ve gerçek uçuşta fünye normal çalışır.
- Referans kaynak: `SİT_SUT/SİT-SUT.cpp` (protokol/mantık buradan port edilir, transport hariç).

---

### Task 1: SİT/SUT sabitleri, enum, global değişkenler ve SitPaketi struct

**Files:**
- Modify: `src/main.cpp` (define bloğu ~144, enum ~180, global ~197, struct ~274 civarı)

**Interfaces:**
- Produces: `enum SitSutModu { MOD_BEKLEME, MOD_SIT, MOD_SUT }`; `volatile SitSutModu sitSutMod`; `float basinc`; `volatile uint16_t durum_bitleri`; `unsigned long kalkis_zaman`; `struct SitPaketi` (36B); makrolar `BAUD_TTL`, `SITSUT_*`, `CMD_*`, `CHK_*`, `KOMUT_CHK_OFFSET`, `TEST_AKTIVASYON_GECIKME_MS`, `SITSUT_GONDERIM_PERIYOT_MS`, `SITSUT_FRAME_TIMEOUT_MS`, `ST_BIT_*`, `DURUM_*`, `MOTOR_YANMA_SURE_MS`, `SUT_FUNYE_YERINE_LED`, `PIN_LED_DROGUE`, `PIN_LED_ANA`.

- [ ] **Step 1: SİT/SUT define bloğunu ekle**

`src/main.cpp` içinde `#define LORA_GONDERIM_ORANI    10` satırının hemen ALTINA (main ~144) şu bloğu ekle:

```cpp
// --- SİT/SUT TTL (UART0 / Serial) ---
#define BAUD_TTL             115200   // UART0 — SİT/SUT komut alma (Ek-7 Tablo 7 zorunlu)

// --- SİT/SUT KOMUT PROTOKOLÜ ---
// Format: [HEADER=0xAA][COMMAND:1B][CHECKSUM:1B][FOOTER1=0x0D][FOOTER2=0x0A]
// Checksum iki konvansiyon da kabul: (a) 0xAA+cmd  (b) cmd+0x6C (Ek-7 Tablo 1)
#define SITSUT_HEADER        0xAA
#define SITSUT_DATA_HEADER   0xAB
#define SITSUT_FOOTER1       0x0D
#define SITSUT_FOOTER2       0x0A
#define SITSUT_PAKET_BOYUT   5
#define SITSUT_DATA_BOYUT    36

#define CMD_SIT_BASLAT       0x20  // Sensör İzleme Testi Başlat
#define CMD_SUT_BASLAT       0x22  // Sentetik Uçuş Testi Başlat
#define CMD_DURDUR           0x24  // Testi Durdur

#define CHK_SIT_T1           0x8C  // 0x20 + 0x6C
#define CHK_SUT_T1           0x8E  // 0x22 + 0x6C
#define CHK_DURDUR_T1        0x90  // 0x24 + 0x6C
#define KOMUT_CHK_OFFSET     0x6C  // Tablo 1 checksum ofseti (Command + 0x6C)

// --- ORTAK ZAMANLAMA (Ek-7) ---
#define TEST_AKTIVASYON_GECIKME_MS  1000  // Komut onayindan sonra modun aktif olma gecikmesi
#define SITSUT_GONDERIM_PERIYOT_MS   100  // SİT telemetri + SUT durum paketi periyodu = 10 Hz
#define SITSUT_FRAME_TIMEOUT_MS      100  // Yarim kalan cerceve icin parser sifirlama suresi

// --- SUT DURUM BİLGİLENDİRME PAKETİ (Tablo 5 & 6) ---
#define SITSUT_DURUM_BOYUT   6
#define ST_BIT_KALKIS        (1u << 0)
#define ST_BIT_MOTOR_YANMA   (1u << 1)
#define ST_BIT_MIN_IRTIFA    (1u << 2)
#define ST_BIT_ACI_ESIK      (1u << 3)
#define ST_BIT_ALCALMA       (1u << 4)
#define ST_BIT_DROGUE_EMIR   (1u << 5)
#define ST_BIT_ANA_IRTIFA    (1u << 6)
#define ST_BIT_ANA_EMIR      (1u << 7)

#define DURUM_MIN_IRTIFA_ESIGI  100.0  // m      - Bit 2 esigi
#define DURUM_ACI_ESIGI          45.0  // derece - Bit 3 esigi
#define MOTOR_YANMA_SURE_MS      3000  // ms     - Bit 1: kalkistan sonra motor yanma onlem suresi

// --- SUT TEZGAH TESTI: FÜNYE YERINE LED (GÜVENLİK) ---
// 1 = SUT modunda gercek fünye pinini SÜRME, sadece gosterge LED yak (tezgah — GÜVENLİ)
// 0 = SUT'ta gercek fünyeyi de atesle (Aksaray GERÇEK test cihazi olcumu icin)
//   NOT: SİT ve gercek ucusta bu bayraktan bagimsiz olarak fünye normal calisir.
#define SUT_FUNYE_YERINE_LED   1
#define PIN_LED_DROGUE   PIN_LED_1   // GPIO26 — 1. ayrilma (drogue) gostergesi
#define PIN_LED_ANA      PIN_LED_2   // GPIO4  — 2. ayrilma (ana parasut) gostergesi
```

- [ ] **Step 2: SitSutModu enum'unu ekle**

`UcusDurumu durum = HAZIR;` satırının (main ~180) hemen ALTINA ekle:

```cpp
// SİT/SUT Mod Durum Makinesi (Task2 tarafindan yonetilir)
enum SitSutModu {
    MOD_BEKLEME   = 0, // Komut bekleniyor, test aktif degil
    MOD_SIT       = 1, // Sensör İzleme Testi aktif
    MOD_SUT       = 2  // Sentetik Uçuş Testi aktif
};
volatile SitSutModu sitSutMod = MOD_BEKLEME;
```

- [ ] **Step 3: Global değişkenleri ekle**

`float max_irtifa_degeri = 0.0;` satırının (main ~185) hemen ALTINA ekle:

```cpp
unsigned long kalkis_zaman = 0;          // Kalkis aninin millis() degeri (durum biti 1 icin)
volatile uint16_t durum_bitleri = 0;     // SUT durum bilgilendirme bitleri (Tablo 5, latch)
```

`float irtifa = 0.0;` satırının (main ~253, "Barometre (BME280) Verileri" altında) hemen ALTINA ekle:

```cpp
float basinc = 0.0; // Yalniz SİT modunda okunur (Tablo 3 SİT paketi icin)
```

- [ ] **Step 4: SitPaketi struct'ini ekle**

`TelemetryPacket` struct'inin `#pragma pack(pop)` kapanışının (main ~274) hemen ALTINA ekle:

```cpp
// --- SİT TELEMETRİ PAKETİ (Tablo 3 — 36 byte) ---
// [0xAB][İRTİFA:4B][BASINÇ:4B][İVME_X/Y/Z:4B×3][AÇI_X/Y/Z:4B×3][CHK:1B][0x0D][0x0A]
#pragma pack(push, 1)
struct SitPaketi {
    uint8_t header;     // 0xAB
    float   irtifa;
    float   basinc;
    float   ivmeX;
    float   ivmeY;
    float   ivmeZ;
    float   aciX;       // Roll
    float   aciY;       // Pitch
    float   aciZ;       // Yaw
    uint8_t checksum;
    uint8_t footer1;    // 0x0D
    uint8_t footer2;    // 0x0A
};
#pragma pack(pop)
// sizeof(SitPaketi) = 36 byte
```

- [ ] **Step 5: Derle**

Run: `pio run -e ucus`
Expected: Derleme BAŞARILI (`SUCCESS`). Kullanılmayan sabit/struct uyarısı olabilir, hata olmamalı.

- [ ] **Step 6: Commit**

```bash
git add src/main.cpp docs/superpowers/specs/2026-07-07-sit-sut-main-entegrasyon-design.md docs/superpowers/plans/2026-07-07-sit-sut-main-entegrasyon.md
git commit -m "feat(sitsut): SİT/SUT sabitleri, enum, global ve SitPaketi struct eklendi"
```

---

### Task 2: Yardımcı fonksiyonlar (big-endian, yuvarla2, SİT & durum paketi gönderimi)

**Files:**
- Modify: `src/main.cpp` (yardımcı fonksiyon bölgesi, `crc16_ccitt` sonrası ~367)

**Interfaces:**
- Consumes: Task 1'den `SITSUT_*`, `ST_BIT_*`, `durum_bitleri`, `basinc`; mevcut globaller `irtifa`, `ivmeX/Y/Z`, `roll/pitch/yaw`.
- Produces: `void float_to_be32(float, uint8_t*)`; `float be32_to_float(const uint8_t*)`; `float yuvarla2(float)`; `void gonder_sit_paketi()`; `void gonder_durum_paketi()`.

- [ ] **Step 1: Yardımcı fonksiyonları ekle**

`crc16_ccitt(...)` fonksiyonunun kapanış `}`'inin (main ~367) hemen ALTINA ekle:

```cpp
// FLOAT'i virgülden sonra 2 basamaga yuvarlar (Ek-7 s.7)
static inline float yuvarla2(float v) {
    return roundf(v * 100.0f) / 100.0f;
}

// --- ENDIANNESS (Ek-7: veri BIG ENDIAN gonderilir) ---
static inline void float_to_be32(float v, uint8_t* out) {
    uint8_t tmp[4];
    memcpy(tmp, &v, 4);   // native little-endian
    out[0] = tmp[3]; out[1] = tmp[2]; out[2] = tmp[1]; out[3] = tmp[0]; // MSB first
}
static inline float be32_to_float(const uint8_t* in) {
    uint8_t tmp[4];
    tmp[0] = in[3]; tmp[1] = in[2]; tmp[2] = in[1]; tmp[3] = in[0];
    float v; memcpy(&v, tmp, 4);
    return v;
}

// --- SİT PAKETİ GÖNDERME (TTL / UART0 / Serial) — Tablo 3, 36 byte ---
// FLOAT32 alanlar BIG ENDIAN. Checksum = Header + 32B payload (mod 256).
void gonder_sit_paketi() {
    uint8_t pkt[SITSUT_DATA_BOYUT];
    pkt[0] = 0xAB;
    float_to_be32(yuvarla2(irtifa),          &pkt[1]);   // İRTİFA
    float_to_be32(yuvarla2(basinc / 100.0f), &pkt[5]);   // BASINÇ (Pascal→mBar)
    float_to_be32(yuvarla2(ivmeX),           &pkt[9]);
    float_to_be32(yuvarla2(ivmeY),           &pkt[13]);
    float_to_be32(yuvarla2(ivmeZ),           &pkt[17]);
    float_to_be32(yuvarla2(roll),            &pkt[21]);  // AÇI X
    float_to_be32(yuvarla2(pitch),           &pkt[25]);  // AÇI Y
    float_to_be32(yuvarla2(yaw),             &pkt[29]);  // AÇI Z
    uint8_t chk = 0;
    for (int i = 0; i < 33; i++) chk += pkt[i];
    pkt[33] = chk;
    pkt[34] = 0x0D;
    pkt[35] = 0x0A;
    Serial.write(pkt, SITSUT_DATA_BOYUT);
}

// --- SUT DURUM PAKETİ GÖNDERME (TTL / UART0 / Serial) — Tablo 6, 6 byte ---
// [0xAA][Data1][Data2][CHK][0x0D][0x0A]; CHK=(Header+Data1+Data2)&0xFF.
void gonder_durum_paketi() {
    uint8_t data1 = (uint8_t)(durum_bitleri & 0xFF);
    uint8_t data2 = (uint8_t)((durum_bitleri >> 8) & 0xFF);
    uint8_t chk   = (uint8_t)(SITSUT_HEADER + data1 + data2);
    uint8_t paket[SITSUT_DURUM_BOYUT] = {
        SITSUT_HEADER, data1, data2, chk, SITSUT_FOOTER1, SITSUT_FOOTER2
    };
    Serial.write(paket, SITSUT_DURUM_BOYUT);
}
```

- [ ] **Step 2: Derle**

Run: `pio run -e ucus`
Expected: Derleme BAŞARILI. `basinc` artık kullanıldığı için Task 1'deki "unused" uyarısı kaybolur.

- [ ] **Step 3: Commit**

```bash
git add src/main.cpp
git commit -m "feat(sitsut): big-endian, SİT paketi ve durum paketi gonderim fonksiyonlari"
```

---

### Task 3: Fünye SUT→LED güvenlik mantığı

**Files:**
- Modify: `src/main.cpp` — `Funye1Atesle()` ve `Funye2Atesle()` (main ~294-310)

**Interfaces:**
- Consumes: Task 1'den `SUT_FUNYE_YERINE_LED`, `PIN_LED_DROGUE`, `PIN_LED_ANA`, `sitSutMod`, `MOD_SUT`.
- Produces: değişen davranış — SUT tezgah modunda gerçek fünye pini sürülmez.

- [ ] **Step 1: Funye1Atesle'yi güncelle**

Mevcut `Funye1Atesle()` gövdesini şununla değiştir:

```cpp
void Funye1Atesle(){
    if (!funye1_aktif) {
        // Gorsel gosterge: drogue LED her modda yakilir (latch — Durdur'a kadar yanik)
        digitalWrite(PIN_LED_DROGUE, HIGH);
        // SUT tezgah testinde (bayrak 1) gercek fünyeyi ATESLEME — sadece LED.
        if (!(SUT_FUNYE_YERINE_LED && sitSutMod == MOD_SUT)) {
            digitalWrite(PIN_FUNYE_1, HIGH);
        }
        funye1_baslangic = millis();
        funye1_aktif = true;
        ayrilma1 = true;
    }
}
```

- [ ] **Step 2: Funye2Atesle'yi güncelle**

Mevcut `Funye2Atesle()` gövdesini şununla değiştir:

```cpp
void Funye2Atesle(){
    if (!funye2_aktif) {
        digitalWrite(PIN_LED_ANA, HIGH);
        if (!(SUT_FUNYE_YERINE_LED && sitSutMod == MOD_SUT)) {
            digitalWrite(PIN_FUNYE_2, HIGH);
        }
        funye2_baslangic = millis();
        funye2_aktif = true;
        ayrilma2 = true;
    }
}
```

- [ ] **Step 3: Derle**

Run: `pio run -e ucus`
Expected: Derleme BAŞARILI.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "feat(sitsut): SUT tezgah testinde funye yerine gosterge LED"
```

---

### Task 4: Task1code — mod reset, SİT basınç okuma, 10Hz TTL gönderim, durum bitleri

**Files:**
- Modify: `src/main.cpp` — `Task1code()` (main ~475-619)

**Interfaces:**
- Consumes: Task 1 sabitleri/globalleri, Task 2 `gonder_sit_paketi`/`gonder_durum_paketi`, `basinc`.
- Produces: SİT modunda 10 Hz `SitPaketi` (UART0); SUT modunda 10 Hz durum paketi; SUT modunda donanım okuması atlanır; `kalkis_zaman` ve `durum_bitleri` doldurulur.

- [ ] **Step 1: Task1 başına mod-geçiş reset ekle**

`Task1code`'un `for (;;) {` satırının hemen ÜSTÜNE (fonksiyon gövdesinin başına) statik değişken ekle ve döngü başına reset bloğu ekle. `Task1code`'un başını şu hale getir:

```cpp
void Task1code(void *pvParameters) {

  static SitSutModu onceki_mod = MOD_BEKLEME;

  for (;;) {
    // 0. MOD GEÇİŞ KONTROLÜ — yeni SUT baslayinca ucus algoritmasini sifirla
    SitSutModu simdiki_mod = sitSutMod;
    if (simdiki_mod != onceki_mod) {
        if (simdiki_mod == MOD_SUT) {
            durum             = HAZIR;
            ayrilma1          = false;
            ayrilma2          = false;
            max_irtifa_degeri = 0.0;
            durum_bitleri     = 0;
            kalkis_zaman      = 0;
            onceki_zaman      = 0;      // dikey hiz hesaplayici yeniden baslasin
            anlik_dikey_hiz   = 0.0;
            funye1_aktif = false;
            funye2_aktif = false;
            digitalWrite(PIN_FUNYE_1, LOW);
            digitalWrite(PIN_FUNYE_2, LOW);
            digitalWrite(PIN_LED_DROGUE, LOW);
            digitalWrite(PIN_LED_ANA, LOW);
        }
        onceki_mod = simdiki_mod;
    }
```

- [ ] **Step 2: Sensör okumalarını SUT modunda atla + SİT'te basınç oku**

Mevcut sensör okuma bloğunu (BNO055 ivme/gyro/euler + BME irtifa + GPS, main ~483-530) şu şekilde `if (sitSutMod != MOD_SUT)` içine al ve SİT için basınç ekle. Bloğun tamamını şununla değiştir:

```cpp
    // 1. Sensör okuma — SUT modunda degilsek donanimdan oku
    if (sitSutMod != MOD_SUT) {
        // IMU (BNO055)
        sensors_event_t a, g, o;
        bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
        ivmeX = kf_ivmeX.updateEstimate(a.acceleration.x);
        ivmeY = kf_ivmeY.updateEstimate(a.acceleration.y);
        ivmeZ = kf_ivmeZ.updateEstimate(a.acceleration.z);

        bno.getEvent(&g, Adafruit_BNO055::VECTOR_GYROSCOPE);
        gyroX = kf_gyroX.updateEstimate(g.gyro.x);
        gyroY = kf_gyroY.updateEstimate(g.gyro.y);
        gyroZ = kf_gyroZ.updateEstimate(g.gyro.z);

        bno.getEvent(&o, Adafruit_BNO055::VECTOR_EULER);
        yaw   = kf_yaw.updateEstimate(o.orientation.x);
        roll  = kf_roll.updateEstimate(o.orientation.y);
        pitch = kf_pitch.updateEstimate(o.orientation.z);

        // 2. Barometre (BME280) — irtifa her zaman; basinc YALNIZ SİT modunda (Tablo 3)
        irtifa = kf_irtifa.updateEstimate(bme.readAltitude(referans_basinc));
        if (sitSutMod == MOD_SIT) {
            basinc = bme.readPressure(); // Pascal
        }

        // 3. GPS
        while (Serial2.available() > 0) {
            gps.encode(Serial2.read());
        }
        if (gps.location.isUpdated()) {
            gpsEnlem = gps.location.lat();
            gpsBoylam = gps.location.lng();
        }
    }
    // MOD_SUT: donanim atlandi; irtifa/basinc/ivme/aci degerleri Task2 tarafindan
    // TTL'den enjekte ediliyor.
```

Not: bu blok, mevcut kodda `eglim_acisi` hesabından ÖNCE biter; `eglim_acisi` ve `anlik_dikey_hiz` hesapları (main ~503-518) OLDUĞU GİBİ kalır (SUT'ta enjekte edilen globalleri kullanır).

- [ ] **Step 3: HAZIR durumunda kalkis_zaman'i kaydet**

Uçuş algoritmasındaki `case HAZIR:` bloğunda (main ~538-544) `durum = YUKSELIYOR;` satırının hemen ALTINA ekle:

```cpp
                kalkis_zaman = millis(); // Motor yanma onlem suresi (durum biti 1) icin referans
```

- [ ] **Step 4: Durum bitleri + 10Hz TTL gönderimini ekle**

Uçuş algoritması `switch (durum)` bloğunun kapanış `}`'inin (main ~598, `case INDI` sonrası) hemen ALTINA, `// --- STRUCT DOLDURMA` yorumundan ÖNCE ekle:

```cpp
    // --- SUT DURUM BİTLERİ (Tablo 5, latch) ---
    if (durum >= YUKSELIYOR)                            durum_bitleri |= ST_BIT_KALKIS;
    if ((durum_bitleri & ST_BIT_KALKIS) &&
        (millis() - kalkis_zaman >= MOTOR_YANMA_SURE_MS)) durum_bitleri |= ST_BIT_MOTOR_YANMA;
    if (irtifa > DURUM_MIN_IRTIFA_ESIGI)               durum_bitleri |= ST_BIT_MIN_IRTIFA;
    if (eglim_acisi > DURUM_ACI_ESIGI)                 durum_bitleri |= ST_BIT_ACI_ESIK;
    if (durum >= INIS_1)                               durum_bitleri |= ST_BIT_ALCALMA;
    if (ayrilma1)                                      durum_bitleri |= ST_BIT_DROGUE_EMIR;
    if ((durum >= INIS_1) && (irtifa < AYRILMA2_MESAFE)) durum_bitleri |= ST_BIT_ANA_IRTIFA;
    if (ayrilma2)                                      durum_bitleri |= ST_BIT_ANA_EMIR;

    // --- 10 Hz TTL GÖNDERİM (Ek-7 s.7) ---
    //   MOD_SIT → Tablo 3 SİT telemetri (36B) ; MOD_SUT → Tablo 6 durum (6B)
    static unsigned long son_ttl_gonderim = 0;
    if (millis() - son_ttl_gonderim >= SITSUT_GONDERIM_PERIYOT_MS) {
        son_ttl_gonderim = millis();
        if (sitSutMod == MOD_SIT) {
            gonder_sit_paketi();
        } else if (sitSutMod == MOD_SUT) {
            gonder_durum_paketi();
        }
    }
```

- [ ] **Step 5: Derle**

Run: `pio run -e ucus`
Expected: Derleme BAŞARILI.

- [ ] **Step 6: Commit**

```bash
git add src/main.cpp
git commit -m "feat(sitsut): Task1 mod reset, SİT basinc, 10Hz TTL gonderim, durum bitleri"
```

---

### Task 5: Task2code — TTL parser + gecikmeli aktivasyon

**Files:**
- Modify: `src/main.cpp` — `Task2code()` (main ~634-671)

**Interfaces:**
- Consumes: Task 1 sabitleri/globalleri; Task 2 `be32_to_float`; mevcut `telemetryQueue`, `gonder_paket_framed_dma`, ping-pong SD fonksiyonları.
- Produces: `Serial` (UART0) üzerinden 5B komut ve 36B SUT-veri çerçevelerini parse eder; `sitSutMod`'u 1 sn gecikmeli günceller; SUT verisini globallere enjekte eder.

- [ ] **Step 1: Task2 başına TTL parser durumu ekle**

`Task2code`'da `uint32_t lora_sayac = 0;` satırının hemen ALTINA ekle:

```cpp
  // TTL komut/veri okuma buffer'i (UART0)
  uint8_t ttl_buf[SITSUT_DATA_BOYUT]; // Max 36 byte
  uint8_t ttl_idx = 0;
  uint8_t beklenen_boyut = SITSUT_PAKET_BOYUT;
  unsigned long son_ttl_byte_zamani = 0;

  // 1 sn gecikmeli aktivasyon
  SitSutModu    bekleyen_mod        = MOD_BEKLEME;
  bool          aktivasyon_bekliyor = false;
  unsigned long aktivasyon_zamani   = 0;
```

- [ ] **Step 2: Kuyruk okumadan önce TTL parser + aktivasyon bloğunu ekle**

`Task2code`'un `for (;;) {` satırının hemen ALTINA (mevcut `if (xQueueReceive(...))` satırından ÖNCE) ekle:

```cpp
    // ─── 1. TTL OKUMA (Komutlar + SUT Verisi) — UART0/Serial ───────
    while (Serial.available() > 0) {
      uint8_t b = Serial.read();
      son_ttl_byte_zamani = millis();

      if (ttl_idx == 0) {
        if (b == SITSUT_HEADER) {
            beklenen_boyut = SITSUT_PAKET_BOYUT; // 5
            ttl_buf[ttl_idx++] = b;
        } else if (b == SITSUT_DATA_HEADER) {
            beklenen_boyut = SITSUT_DATA_BOYUT;  // 36
            ttl_buf[ttl_idx++] = b;
        }
      } else {
        ttl_buf[ttl_idx++] = b;

        if (ttl_idx >= beklenen_boyut) {
          // KOMUT (5B)
          if (beklenen_boyut == SITSUT_PAKET_BOYUT) {
              uint8_t cmd = ttl_buf[1];
              uint8_t chk = ttl_buf[2];
              uint8_t chk_hdr_cmd = (uint8_t)(SITSUT_HEADER + cmd);    // (a)
              uint8_t chk_tablo1  = (uint8_t)(cmd + KOMUT_CHK_OFFSET); // (b)
              if (ttl_buf[3] == SITSUT_FOOTER1 && ttl_buf[4] == SITSUT_FOOTER2 &&
                  (chk == chk_hdr_cmd || chk == chk_tablo1)) {
                  switch (cmd) {
                      case CMD_SIT_BASLAT:
                          bekleyen_mod        = MOD_SIT;
                          aktivasyon_bekliyor = true;
                          aktivasyon_zamani   = millis() + TEST_AKTIVASYON_GECIKME_MS;
                          break;
                      case CMD_SUT_BASLAT:
                          bekleyen_mod        = MOD_SUT;
                          aktivasyon_bekliyor = true;
                          aktivasyon_zamani   = millis() + TEST_AKTIVASYON_GECIKME_MS;
                          break;
                      case CMD_DURDUR:
                          aktivasyon_bekliyor = false;
                          sitSutMod           = MOD_BEKLEME;
                          break;
                  }
              }
          }
          // SUT VERİSİ (36B)
          else if (beklenen_boyut == SITSUT_DATA_BOYUT) {
              if (ttl_buf[34] == SITSUT_FOOTER1 && ttl_buf[35] == SITSUT_FOOTER2) {
                  uint8_t chk_payload = 0;
                  for (int i = 1; i <= 32; i++) chk_payload += ttl_buf[i];
                  uint8_t chk_header = chk_payload + ttl_buf[0];
                  if (ttl_buf[33] == chk_header || ttl_buf[33] == chk_payload) {
                      // Gelen FLOAT32'ler BIG ENDIAN (Ek-7)
                      irtifa = be32_to_float(&ttl_buf[1]);
                      basinc = be32_to_float(&ttl_buf[5]);
                      ivmeX  = be32_to_float(&ttl_buf[9]);
                      ivmeY  = be32_to_float(&ttl_buf[13]);
                      ivmeZ  = be32_to_float(&ttl_buf[17]);
                      roll   = be32_to_float(&ttl_buf[21]);
                      pitch  = be32_to_float(&ttl_buf[25]);
                      yaw    = be32_to_float(&ttl_buf[29]);
                  }
              }
          }
          ttl_idx = 0;
        }
      }
    }

    // ─── 1b. FRAME TIMEOUT (100 ms) — yarim cerceveyi sifirla ──
    if (ttl_idx > 0 && (millis() - son_ttl_byte_zamani) > SITSUT_FRAME_TIMEOUT_MS) {
        ttl_idx = 0;
    }

    // ─── 1c. GECİKMELİ AKTİVASYON — onaydan 1 sn sonra modu etkinlestir ──
    if (aktivasyon_bekliyor && (long)(millis() - aktivasyon_zamani) >= 0) {
        sitSutMod           = bekleyen_mod;
        aktivasyon_bekliyor = false;
    }
```

- [ ] **Step 3: Kuyruk okumasını bloklamayan hale getir**

Mevcut `if (xQueueReceive(telemetryQueue, &incomingPacket, portMAX_DELAY) == pdTRUE) {` satırını (main ~641) şununla değiştir (böylece TTL sürekli okunabilir):

```cpp
    if (xQueueReceive(telemetryQueue, &incomingPacket, pdMS_TO_TICKS(10)) == pdTRUE) {
```

- [ ] **Step 4: Derle**

Run: `pio run -e ucus`
Expected: Derleme BAŞARILI.

- [ ] **Step 5: Commit**

```bash
git add src/main.cpp
git commit -m "feat(sitsut): Task2 TTL parser, gecikmeli aktivasyon, non-blocking kuyruk"
```

---

### Task 6: setup — Serial (UART0) başlat + gösterge LED init; tam derleme

**Files:**
- Modify: `src/main.cpp` — `setup()` (main ~673-687)

**Interfaces:**
- Consumes: Task 1 `BAUD_TTL`, `PIN_LED_DROGUE`, `PIN_LED_ANA`.
- Produces: UART0 @115200 aktif (SİT/SUT TTL kanalı); gösterge LED'leri başta sönük.

- [ ] **Step 1: Serial (UART0) başlat**

`setup()` başındaki `// 1. TTL kullanılmıyor — Serial başlatılmadı, pinler boşta` yorumunu (main ~674) şununla değiştir:

```cpp
    // 1. TTL — SİT/SUT komut alma icin Serial (UART0) baslatiliyor
    Serial.begin(BAUD_TTL);
```

- [ ] **Step 2: Gösterge LED'lerini başta söndür**

`setup()` içinde `pinMode(PIN_LED_3, OUTPUT);` satırının (main ~686) hemen ALTINA ekle:

```cpp
    // Drogue/Ana gosterge LED'lerini basta sondur (PIN_LED_1/PIN_LED_2 zaten OUTPUT)
    digitalWrite(PIN_LED_DROGUE, LOW);
    digitalWrite(PIN_LED_ANA, LOW);
```

- [ ] **Step 3: Tam derleme doğrulaması**

Run: `pio run -e ucus`
Expected: Derleme BAŞARILI, `SUCCESS`. RAM/Flash kullanım özeti basılır. Hata/uyarı-hata olmamalı.

- [ ] **Step 4: Referans sürümün hâlâ derlendiğini doğrula (regresyon yok)**

Run: `pio run -e sitsut`
Expected: Derleme BAŞARILI (referans dosyaya dokunulmadı).

- [ ] **Step 5: Commit**

```bash
git add src/main.cpp
git commit -m "feat(sitsut): setup UART0 TTL baslatma + gosterge LED init; entegrasyon tamam"
```

---

## Self-Review Notlarım

- **Spec kapsamı:** Transport ayrımı (Task 1,2,5,6), SİT'te basınç (Task 4 Step 2), onay mesajı gönderilmemesi (parser'da `Serial1.println` port edilmedi — Task 5), slim paket korunumu (hiçbir taskta TelemetryPacket değişmedi), mod state machine (Task 1,4,5), güvenlik bayrağı (Task 3) — hepsi bir taska bağlandı.
- **Placeholder taraması:** Yok; her kod adımı tam içerik içeriyor.
- **Tip tutarlılığı:** `sitSutMod`/`MOD_*`, `durum_bitleri`, `SitPaketi`, `gonder_sit_paketi`/`gonder_durum_paketi`, `be32_to_float`/`float_to_be32` isimleri tüm tasklarda tutarlı.
- **Doğrulama modeli:** Gömülü proje; her task derleme (`pio run -e ucus`) ile doğrulanır. Donanım/HIL davranış testi (SİT komutu→36B çıkışı, SUT enjeksiyonu) fiziksel test cihazıyla ayrıca yapılır — spec "Doğrulama" bölümünde listelendi.
