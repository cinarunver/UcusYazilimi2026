/**
 * ============================================================
 * TRAKYA ROKET 2026 - BILIMSEL GOREV YUKU (BGY) YAZILIMI
 * ============================================================
 * main.cpp'den FORKLANMISTIR. Farklar (SADECE bunlar cikarildi):
 *   - UCUS ALGORITMASI YOK (state machine / apogee / funye YOK)
 *   - BNO055 (IMU) YOK
 * Kalan mimari main.cpp ile AYNI:
 *   - BME280 (basinc/sicaklik/nem/irtifa) + GPS okunur, Kalman'dan gecer
 *   - Cift cekirdek (Core0 oku / Core1 haberlesme) + FreeRTOS Queue
 *   - Paket cerceveleme (SYNC + LEN + CRC16-CCITT) + DMA LoRa gonderim
 *   - SD karta ping-pong buffer ile CSV loglama
 *
 * NOT (RF): Ana roket (UKB) ile ayni anda yayin yapacagi icin GERCEK
 *   ucusta bu modul FARKLI bir LoRa kanalinda (Kanal B) olmalidir.
 *   -> en bastaki LORA_CHAN'i UKB'den farkli sec!
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <FS.h>
#include "driver/uart.h"
#include "esp_heap_caps.h"

// ============================================================
//  KURTARMA BEACON + LED KARAR MANTIGI (eski led_durum_bgy.h — koda gomuldu)
// ============================================================
// LED flash ile buzzer bip AYNI millis() fazindan beslenir -> dogal senkron.
#define BEACON_BIP_MS       200   // ms - beacon "acik" suresi
#define BEACON_PERIYOT_MS  1000   // ms - beacon periyodu (BIP_MS acik + kalani sonuk)

// Gorev yukunun 4 gosterge cikisi: 3 durum LED'i (26/4/25) + kurtarma beacon LED'i (13)
struct LedDurumBgy { bool led1; bool led2; bool led3; bool beacon; };

// LED karar mantigi — hardware'siz, saf. Yalniz sistem_hazir + uctu + indi'ye bagli.
//   sistem_hazir: setup() bitince true; false iken 3 durum LED'i 1 Hz blink (config)
//   uctu        : irtifa/ivme esigini gecince true (ucusa gecti)
//   indi        : indikten sonra latch true -> kurtarma beacon (en oncelikli)
//   now_ms      : millis()  (blink/flash fazi icin)
static inline LedDurumBgy hesapla_led_durumu_bgy(bool sistem_hazir, bool uctu,
                                                 bool indi, unsigned long now_ms) {
    LedDurumBgy d = {false, false, false, false};

    // 1) Indi (en oncelikli): 4 cikis da buzzer ritminde senkron flash = kurtarma beacon
    if (indi) {
        bool flash = (now_ms % BEACON_PERIYOT_MS) < BEACON_BIP_MS;
        d.led1 = d.led2 = d.led3 = d.beacon = flash;
        return d;
    }

    // 2) Config bitmedi -> 3 durum LED'i 1 Hz blink (beacon kapali)
    if (!sistem_hazir) {
        bool acik = ((now_ms / 500) % 2 == 0);
        d.led1 = d.led2 = d.led3 = acik;
        return d;
    }

    // 3) Config bitti: bekliyor (uctu=false) -> 3 LED sabit ; ucusta -> hepsi kapali
    if (!uctu) {
        d.led1 = d.led2 = d.led3 = true;
    }
    return d;
}

// ============================================================
//  >>> YARISMA ALANI - LORA ADRES & KANAL AYARLARI <<<
//  E32-433T30D parametreleri. Yarisma alaninda SADECE buradan ayarla.
//  ONEMLI: UKB (main.cpp) ile Gorev Yuku FARKLI kanalda olmali (RF cakismasi)!
//  Frekans (MHz) = 410 + LORA_CHAN   (CHAN: 0x00..0x1F ; 0x17 = 433 MHz)
// ============================================================
#define LORA_ADDH    0x00   // Adres yuksek byte
#define LORA_ADDL    0x02   // Adres dusuk byte
#define LORA_CHAN    0x16   // Kanal (UKB'den FARKLI sec!) frekans=410+CHAN MHz
#define LORA_SPED    0x1C   // UART 9600 8N1 + hava hizi (degistirmeyin)

// --- OPTION byte bilesenleri (E32-433T30D datasheet §7.5) ---
// OPTION = TX_MODE | IO_DRIVE | WOR_TIME | FEC | TX_POWER  (asagidan otomatik birlesir)
#define LORA_TX_MODE   0x00   // bit7  0x00=SEFFAF (transparent) | 0x80=SABIT (fixed)
#define LORA_IO_DRIVE  0x40   // bit6  0x40=push-pull/pull-up (varsayilan) | 0x00=open-collector
#define LORA_WOR_TIME  0x00   // bit5-3 uyanma(WOR) suresi: 0x00=250 0x08=500 0x10=750 0x18=1000
                              //                            0x20=1250 0x28=1500 0x30=1750 0x38=2000 ms
#define LORA_FEC       0x04   // bit2  0x04=FEC ACIK (varsayilan) | 0x00=FEC kapali
#define LORA_TX_POWER  0x00   // bit1-0 TX gucu: 0x00=30dBm(1W) 0x01=27 0x02=24 0x03=21 dBm

#define LORA_OPTION  (LORA_TX_MODE | LORA_IO_DRIVE | LORA_WOR_TIME | LORA_FEC | LORA_TX_POWER)  // = 0x44 (seffaf, FEC acik, 250ms, 30dBm)

// --- Pinler (main.cpp ile birebir) ---
#define PIN_SPI_CS 5
#define PIN_GPS_RX 17
#define PIN_GPS_TX 16
#define PIN_SPI_CLK 18
#define PIN_SPI_MISO 19
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_SPI_MOSI 23
#define PIN_LORA_TX 33
#define PIN_LORA_RX 32
#define PIN_SDKART_DET 35
#define LORA_M0 15
#define LORA_M1 2
#define PIN_BUZZER 12   // Kurtarma beacon buzzer (main ile ayni)
#define PIN_LED 13      // Kurtarma beacon LED (main ile ayni)
#define PIN_LED_1 26    // Durum gosterge LED 1 (UKB ile ayni pin)
#define PIN_LED_2 4     // Durum gosterge LED 2
#define PIN_LED_3 25    // Durum gosterge LED 3

// --- Haberlesme sabitleri ---
#define BAUD_GPS               9600
#define BAUD_LORA              9600

// --- Cerceve protokolu ---
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55
#define LORA_GONDERIM_ORANI    1   // her 1. paket -> ~10 Hz

// --- Sensor sabitleri ---
#define BME280_ADDR_PRIMARY   0x76
#define BME280_ADDR_SECONDARY 0x77
#define BNO055_DEF   55
#define BNO055_ADDR  0x28

// --- INIS TESPITI & BEACON SABITLERI (ayarlanabilir) ---
// Not: bunlar "ucus algoritmasi" DEGIL; sadece "uctu mu / durdu mu" beacon mantigi.
#define KALKIS_YUKSEKLIK      50.0   // m     - irtifa bunu gecince "uctu" (yerdeki yanlis tetigi onler)
#define KALKIS_IVME_ESIGI     20.0   // m/s^2 - ivmeZ bunu asinca da "uctu" (alternatif tetik)
#define REST_HIZ_ESIGI         1.0   // m/s   - |dikey hiz| bunun altinda = baro sabit
#define REST_IVME_ESIGI        1.5   // m/s^2 - |dogrusal ivme| bunun altinda = durgun (IMU)
#define REST_GYRO_ESIGI        0.2   // rad/s - |gyro| bunun altinda = durgun (IMU)
#define INIS_STABIL_SURE_MS   5000   // ms    - bu sure boyunca kesintisiz durgun -> "indi"
// BEACON_BIP_MS / BEACON_PERIYOT_MS -> yukarida gomulu bolumde (LED flash + buzzer ayni fazdan senkron)

// --- FreeRTOS sabitleri ---
#define TASK_STACK_SIZE 10000
#define TASK1_PRIORITY 2
#define TASK2_PRIORITY 1
#define TELEMETRY_QUEUE_LEN 10

TaskHandle_t Task1;
TaskHandle_t Task2;

// referans_basinc: setup() icinde BME280'den otomatik olculur (yer kalibrasyonu)
float referans_basinc = 1013.25;

// --- KALMAN FILTRESI (main.cpp ile birebir) ---
class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) :
      err_measure(mea_e), err_estimate(est_e), q(q), last_estimate(0), kalman_gain(0), first_run(true) {}
    float updateEstimate(float mea) {
      if (first_run) { last_estimate = mea; first_run = false; }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      float ce = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabs(last_estimate - ce) * q;
      last_estimate = ce;
      return ce;
    }
  private:
    float err_measure, err_estimate, q, last_estimate, kalman_gain;
    bool  first_run;
};

// BME280 icin Kalman filtreleri (IMU filtreleri YOK)
// NOT: irtifa apogee icin kullanilmadigindan hafif tutuldu (main'deki agir 16.3 DEGIL).
SimpleKalmanFilter kf_basinc(2.0, 2.0, 0.1);
SimpleKalmanFilter kf_sicaklik(0.5, 0.5, 0.01);
SimpleKalmanFilter kf_nem(1.0, 1.0, 0.1);
SimpleKalmanFilter kf_irtifa(1.5, 1.5, 0.1);

// --- Sensor nesneleri ---
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_DEF, BNO055_ADDR);
TinyGPSPlus gps;
bool bnoOk = false;   // BNO bulundu mu? (yoksa inis tespiti yalniz baro ile)

// --- Sensor veri degiskenleri ---
float basinc = 0, sicaklik = 0, nem = 0, irtifa = 0;
float gpsEnlem = 0, gpsBoylam = 0;

// --- INIS TESPITI DURUMU (yalniz lokal; pakete girmez) ---
float max_irtifa = 0.0;
bool  uctu = false;                 // gercekten ucti mi? (yerdeki yanlis tetigi onler)
bool  indi = false;                 // indi mi? (latch — bulana kadar acik)
unsigned long rest_baslangic = 0;   // durgunluk penceresi baslangici (0 = durgun degil)

// setup() bitince true; false iken 3 durum LED'i config blink yapar
volatile bool sistem_hazir = false;

// Dikey hiz (baro turevi) — inis tespitindeki "baro sabit" kontrolu icin
float onceki_irtifa = 0.0;
unsigned long onceki_zaman = 0;
float anlik_dikey_hiz = 0.0;

// --- GOREV YUKU TELEMETRI PAKETI (BME280 + GPS + BNO055 IMU; ucus alanlari YOK) ---
#pragma pack(push, 1)
struct GorevYukuPaket {
    float basinc, sicaklik, nem, irtifa;   // BME280 (basinc hPa)
    float gpsEnlem, gpsBoylam;             // GPS
    float ivmeX, ivmeY, ivmeZ;             // BNO055 lineer ivme (m/s^2)
    float ivmeToplam;                      // bileske ivme = sqrt(x^2+y^2+z^2) (core0'da hesaplanir)
    float gyroX, gyroY, gyroZ;             // BNO055 gyro (rad/s)
};  // 13 float = 52 byte
#pragma pack(pop)

// --- HAVADAN GİDEN FIXED-POINT WIRE PAKET (24 byte) ---
// GorevYukuPaket (float, 52B) yalniz queue + SD icin; LoRa'ya giderken bu
// packed int wire pakete quantize edilir (paket kucultme). Little-endian.
// Yer istasyonu Python format: '<HhHh2i4h'.
// Olcekler: basinc hPa x10, sicaklik x100, nem x100, irtifa x10, GPS x1e7,
//           ivme x100, gyro x10.
// NOT: ivmeX/Y/Z tek tek havadan GONDERILMEZ; yerine bileske (toplam) ivme
//      tek int16 slot ile gider (28B->24B). Ham 3 eksen SD'de tam float kalir.
#pragma pack(push, 1)
struct GorevYukuWire {
    uint16_t basinc;
    int16_t  sicaklik;
    uint16_t nem;
    int16_t  irtifa;
    int32_t  gpsEnlem, gpsBoylam;
    int16_t  ivmeToplam;                    // bileske ivme buyuklugu sqrt(x^2+y^2+z^2)
    int16_t  gyroX, gyroY, gyroZ;
};
#pragma pack(pop)

// SD ping-pong buffer
#define SD_DMA_BUF_SIZE 512
char* sd_dma_buf_A;
char* sd_dma_buf_B;
volatile int active_sd_buf = 0;
volatile int sd_buf_idx = 0;

QueueHandle_t telemetryQueue;
File logFile;
bool sdOk = false;

// --- CRC16-CCITT (main.cpp ile birebir) ---
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// --- FIXED-POINT QUANTIZE YARDIMCILARI (clamp + round; overflow yok) ---
#define WIRE_OLCEK_BASINC  10.0f    // hPa x10 (uint16)
#define WIRE_OLCEK_SICAK  100.0f    // C   x100
#define WIRE_OLCEK_NEM    100.0f    // %   x100 (uint16)
#define WIRE_OLCEK_IRTIFA  10.0f    // m   x10
#define WIRE_OLCEK_IVME   100.0f    // m/s^2 x100
#define WIRE_OLCEK_GYRO    10.0f    // dps/rad-s x10
#define WIRE_OLCEK_GPS     1e7      // derece x1e7 (int32)

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

// GorevYukuPaket (float) -> GorevYukuWire (packed int)
static inline void pack_gorevyuku_wire(GorevYukuWire& w, const GorevYukuPaket& p) {
    w.basinc   = qu16(p.basinc,   WIRE_OLCEK_BASINC);
    w.sicaklik = q16(p.sicaklik,  WIRE_OLCEK_SICAK);
    w.nem      = qu16(p.nem,      WIRE_OLCEK_NEM);
    w.irtifa   = q16(p.irtifa,    WIRE_OLCEK_IRTIFA);
    w.gpsEnlem  = q32(p.gpsEnlem,  WIRE_OLCEK_GPS);
    w.gpsBoylam = q32(p.gpsBoylam, WIRE_OLCEK_GPS);
    w.ivmeToplam = q16(p.ivmeToplam, WIRE_OLCEK_IVME);  // bileske ivme; ham eksenler havadan gitmez
    w.gyroX = q16(p.gyroX, WIRE_OLCEK_GYRO);
    w.gyroY = q16(p.gyroY, WIRE_OLCEK_GYRO);
    w.gyroZ = q16(p.gyroZ, WIRE_OLCEK_GYRO);
}

// --- BUFFERLI (PING-PONG) SD YAZMA ---
void bufferla_ve_yaz_sd(File& file, const GorevYukuPaket& pkt) {
    char temp_line[192];
    int line_len = snprintf(temp_line, sizeof(temp_line),
        "%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        pkt.basinc, pkt.sicaklik, pkt.nem, pkt.irtifa, pkt.gpsEnlem, pkt.gpsBoylam,
        pkt.ivmeX, pkt.ivmeY, pkt.ivmeZ, pkt.gyroX, pkt.gyroY, pkt.gyroZ);

    char* current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
    if (sd_buf_idx + line_len >= SD_DMA_BUF_SIZE) {
        if (file) file.write((const uint8_t*)current_buf, sd_buf_idx);
        active_sd_buf = (active_sd_buf == 0) ? 1 : 0;
        current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
        sd_buf_idx = 0;
    }
    memcpy(&current_buf[sd_buf_idx], temp_line, line_len);
    sd_buf_idx += line_len;
}

// --- PING-PONG TAMPONUNU DISKE BOSALT ---
void sd_buffer_bosalt(File& file) {
    if (file && sd_buf_idx > 0) {
        char* current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
        file.write((const uint8_t*)current_buf, sd_buf_idx);
        sd_buf_idx = 0;
    }
    if (file) file.flush();
}

// --- CERCEVELI PAKET GONDERME (DMA DESTEKLI UART) ---
// Float paket fixed-point wire pakete quantize edilir, sonra cerçevelenir.
// Cerçeve: [0xAA][0x55][LEN=24][GorevYukuWire 24B][CRC16_HI][CRC16_LO] = 29B.
void gonder_paket_framed_dma(uart_port_t uart_num, const GorevYukuPaket& pkt) {
    static uint8_t frame_buf[64];

    GorevYukuWire wire;
    pack_gorevyuku_wire(wire, pkt);

    const uint8_t* payload = (const uint8_t*)&wire;
    const size_t   len     = sizeof(GorevYukuWire);   // 28
    uint16_t       crc     = crc16_ccitt(payload, len);

    size_t idx = 0;
    frame_buf[idx++] = SYNC_BYTE_1;
    frame_buf[idx++] = SYNC_BYTE_2;
    frame_buf[idx++] = (uint8_t)len;
    memcpy(&frame_buf[idx], payload, len);
    idx += len;
    frame_buf[idx++] = (uint8_t)(crc >> 8);
    frame_buf[idx++] = (uint8_t)(crc & 0xFF);

    uart_write_bytes(uart_num, (const char*)frame_buf, idx);
}

// --- E32 LORA KONFIGURASYONU (main.cpp ile birebir) ---
void lora_konfigurasyon() {
    // Parametreler en bastaki YARISMA ALANI define blogundan gelir
    static const uint8_t configPacket[6] = {0xC0, LORA_ADDH, LORA_ADDL, LORA_SPED, LORA_CHAN, LORA_OPTION};
    digitalWrite(LORA_M0, HIGH);
    digitalWrite(LORA_M1, HIGH);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    uart_flush(UART_NUM_1);
    uart_write_bytes(UART_NUM_1, (const char*)configPacket, sizeof(configPacket));
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(200));
    vTaskDelay(200 / portTICK_PERIOD_MS);
    uart_flush(UART_NUM_1);
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

// --- SETUP TESHIS MESAJI -> LoRa (Serial1 kullanilamaz) ---
void lora_log(const char* msg) {
    uart_write_bytes(UART_NUM_1, msg, strlen(msg));
    uart_write_bytes(UART_NUM_1, "\r\n", 2);
}

// --- ANLIK DIKEY HIZ (baro irtifa turevi) ---
// Inis tespitinde "baro sabit mi?" kontrolu icin kullanilir.
float hesapla_dikey_hiz(float guncel_irtifa) {
    unsigned long su_an = micros();
    if (onceki_zaman == 0) { onceki_zaman = su_an; onceki_irtifa = guncel_irtifa; return 0.0; }
    float dt = (float)(su_an - onceki_zaman) / 1000000.0f;
    if (dt <= 0.0) return anlik_dikey_hiz;
    float dh = guncel_irtifa - onceki_irtifa;
    onceki_zaman = su_an; onceki_irtifa = guncel_irtifa;
    return dh / dt;
}

// --- NON-BLOCKING KURTARMA BEACON (yalniz BUZZER) ---
// indi=true olunca buzzer aralikli biper; degilse susar.
// LED'ler (beacon LED 13 dahil) artik led_uygula() tarafindan surulur; buzzer ile
// ayni millis() fazindan beslendikleri icin dogal senkron flash olur.
void beacon_guncelle() {
    if (!indi) { digitalWrite(PIN_BUZZER, LOW); return; }
    bool bip = (millis() % BEACON_PERIYOT_MS) < BEACON_BIP_MS;
    digitalWrite(PIN_BUZZER, bip ? HIGH : LOW);
}

// --- LED GOSTERGE SURUCUSU (tek merkez: 3 durum LED'i 26/4/25 + beacon LED 13) ---
// Karar mantigi yukaridaki gomulu saf fonksiyonda; burada sadece pinlere yazilir.
void led_uygula() {
    LedDurumBgy d = hesapla_led_durumu_bgy(sistem_hazir, uctu, indi, millis());
    digitalWrite(PIN_LED_1, d.led1 ? HIGH : LOW);
    digitalWrite(PIN_LED_2, d.led2 ? HIGH : LOW);
    digitalWrite(PIN_LED_3, d.led3 ? HIGH : LOW);
    digitalWrite(PIN_LED,   d.beacon ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────
// CORE 0: Sensor okuma (BME280 + GPS), Kalman, paketleme
// UCUS ALGORITMASI YOK.
// ─────────────────────────────────────────────────────────────
void Task1code(void *pvParameters) {
  for (;;) {
    // 1. BME280 (basinc/sicaklik/nem/irtifa)
    sicaklik = kf_sicaklik.updateEstimate(bme.readTemperature());
    basinc   = kf_basinc.updateEstimate(bme.readPressure() / 100.0f); // Pa -> hPa
    nem      = kf_nem.updateEstimate(bme.readHumidity());
    irtifa   = kf_irtifa.updateEstimate(bme.readAltitude(referans_basinc));

    // 2. GPS
    while (Serial2.available() > 0) gps.encode(Serial2.read());
    if (gps.location.isUpdated()) {
        gpsEnlem = gps.location.lat();
        gpsBoylam = gps.location.lng();
    }

    // 2.5 BNO055 (IMU) — inis tespiti icin okunur ve ARTIK pakete de basilir.
    float ivmeX = 0, ivmeY = 0, ivmeZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
    if (bnoOk) {
        sensors_event_t a, g;
        bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL); // yercekimsiz -> yerde ~0
        ivmeX = a.acceleration.x; ivmeY = a.acceleration.y; ivmeZ = a.acceleration.z;
        bno.getEvent(&g, Adafruit_BNO055::VECTOR_GYROSCOPE);
        gyroX = g.gyro.x; gyroY = g.gyro.y; gyroZ = g.gyro.z;
    }

    // 2.6 INIS TESPITI (baro + IMU) — "uctu mu / durdu mu" (ucus algoritmasi DEGIL)
    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);
    if (irtifa > max_irtifa) max_irtifa = irtifa;
    // Kalkis: gercekten yukseldi VEYA guclu ivme -> yerdeki yanlis tetigi onler
    if (!uctu && (max_irtifa > KALKIS_YUKSEKLIK || ivmeZ > KALKIS_IVME_ESIGI)) {
        uctu = true;
    }
    // Inis: uctuktan sonra INIS_STABIL_SURE_MS boyunca kesintisiz durgun kalirsa (latch)
    if (uctu && !indi) {
        float acc_mag  = sqrtf(ivmeX*ivmeX + ivmeY*ivmeY + ivmeZ*ivmeZ);
        float gyro_mag = sqrtf(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
        bool baro_sabit = fabsf(anlik_dikey_hiz) < REST_HIZ_ESIGI;
        bool imu_durgun = (acc_mag < REST_IVME_ESIGI) && (gyro_mag < REST_GYRO_ESIGI);
        // IMU yoksa yalniz baro sabitligine bakilir
        bool durgun = baro_sabit && (!bnoOk || imu_durgun);
        if (durgun) {
            if (rest_baslangic == 0) rest_baslangic = millis();
            else if (millis() - rest_baslangic >= INIS_STABIL_SURE_MS) indi = true;
        } else {
            rest_baslangic = 0;   // durgunluk bozuldu -> pencereyi sifirla
        }
    }

    // 2.7 Kurtarma beacon (buzzer) + LED gostergeleri (durum makinesine gore)
    beacon_guncelle();
    led_uygula();

    // 3. Paketle ve Core 1'e gonder
    GorevYukuPaket packet;
    packet.basinc = basinc; packet.sicaklik = sicaklik; packet.nem = nem; packet.irtifa = irtifa;
    packet.gpsEnlem = gpsEnlem; packet.gpsBoylam = gpsBoylam;
    packet.ivmeX = ivmeX; packet.ivmeY = ivmeY; packet.ivmeZ = ivmeZ;
    // Bileske (toplam) ivme buyuklugu — core0'da hesaplanir, havadan tek slot ile gider.
    packet.ivmeToplam = sqrtf(ivmeX * ivmeX + ivmeY * ivmeY + ivmeZ * ivmeZ);
    packet.gyroX = gyroX; packet.gyroY = gyroY; packet.gyroZ = gyroZ;

    xQueueSend(telemetryQueue, &packet, 0);

    vTaskDelay(10 / portTICK_PERIOD_MS); // ~100 Hz
  }
}

// ─────────────────────────────────────────────────────────────
// CORE 1: SD loglama (100 Hz) + LoRa DMA gonderim (~10 Hz)
// ─────────────────────────────────────────────────────────────
void Task2code(void *pvParameters) {
  GorevYukuPaket incomingPacket;
  uint32_t lora_sayac = 0;

  for (;;) {
    if (xQueueReceive(telemetryQueue, &incomingPacket, portMAX_DELAY) == pdTRUE) {
        if (sdOk && logFile) {
            bufferla_ve_yaz_sd(logFile, incomingPacket);
            static int flush_sayac = 0;
            if (++flush_sayac >= 100) { sd_buffer_bosalt(logFile); flush_sayac = 0; }
        }
        if (++lora_sayac >= LORA_GONDERIM_ORANI) {
            gonder_paket_framed_dma(UART_NUM_1, incomingPacket);
            lora_sayac = 0;
        }
    }
  }
}

void setup() {
    // 1. Pinler
    pinMode(PIN_SDKART_DET, INPUT_PULLUP);
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

    // Kurtarma beacon cikislari — baslangicta kapali
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_LED, LOW);

    // Durum gosterge LED'leri (26/4/25) — baslangicta kapali
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);
    digitalWrite(PIN_LED_1, LOW);
    digitalWrite(PIN_LED_2, LOW);
    digitalWrite(PIN_LED_3, LOW);

    // 2. DMA bellek + protokoller
    sd_dma_buf_A = (char*)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    sd_dma_buf_B = (char*)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    // 3. GPS (UART2)
    Serial2.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // 4. LoRa (UART1) DMA
    uart_config_t uart_config = {
        .baud_rate = BAUD_LORA,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, PIN_LORA_TX, PIN_LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    lora_konfigurasyon();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    lora_log("--- GOREV YUKU SISTEMI BASLATILIYOR ---");

    // 5. SD kart
    if (!SD.begin(PIN_SPI_CS)) {
        lora_log("UYARI: SD Kart baslatilamadi!");
        sdOk = false;
    } else {
        logFile = SD.open("/gorevyuku_log.csv", FILE_APPEND);
        if (logFile) {
            if (logFile.size() == 0)
                logFile.println("basinc,sicaklik,nem,irtifa,lat,lng,ivmeX,ivmeY,ivmeZ,gyroX,gyroY,gyroZ");
            sdOk = true;
            lora_log("SD Kart baslatildi.");
        } else {
            lora_log("HATA: Log dosyasi acilamadi!");
            sdOk = false;
        }
    }

    // 6. BME280 (0x76 veya 0x77)
    if (!bme.begin(BME280_ADDR_PRIMARY) && !bme.begin(BME280_ADDR_SECONDARY)) {
        lora_log("KRITIK: BME280 bulunamadi! Sistem durduruluyor.");
        while (true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    lora_log("BME280 baslatildi.");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,   // Sicaklik
                    Adafruit_BME280::SAMPLING_X16,  // Basinc
                    Adafruit_BME280::SAMPLING_X1,   // Nem
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    // 6.1 Yer kalibrasyonu: anlik yer seviyesi basincini olc (20 ornek ortalamasi)
    vTaskDelay(200 / portTICK_PERIOD_MS);
    lora_log("Yer kalibrasyonu yapiliyor...");
    float basinc_toplam = 0.0;
    for (int i = 0; i < 20; i++) {
        basinc_toplam += bme.readPressure() / 100.0F; // Pa -> hPa
        vTaskDelay(50 / portTICK_PERIOD_MS);
        led_uygula(); // config blink (sistem_hazir henuz false)
    }
    referans_basinc = basinc_toplam / 20.0;
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Referans Basinc (hPa): %.2f", referans_basinc);
        lora_log(buf);
    }

    // 6.2 BNO055 (IMU) — YALNIZ inis tespiti icin. Bulunamazsa sistem DURMAZ:
    //     telemetri devam eder, inis tespiti yalniz baro ile yapilir.
    if (bno.begin(OPERATION_MODE_IMUPLUS)) {
        bnoOk = true;
        lora_log("BNO055 baslatildi (IMU mode 0x08) - inis tespiti icin.");
    } else {
        bnoOk = false;
        lora_log("UYARI: BNO055 bulunamadi! Inis tespiti yalniz baro ile yapilacak.");
    }

    // Tum baslangic isleri bitti -> LED'ler artik durum makinesine gore (bekliyor = solid)
    sistem_hazir = true;

    // 7. Kuyruk
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(GorevYukuPaket));
    if (telemetryQueue == NULL) {
        lora_log("KRITIK: Kuyruk olusturulamadi!");
        while (true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }

    // 8. Gorevler
    xTaskCreatePinnedToCore(Task1code, "GYSensor", TASK_STACK_SIZE, NULL, TASK1_PRIORITY, &Task1, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(Task2code, "GYHaberlesme", TASK_STACK_SIZE, NULL, TASK2_PRIORITY, &Task2, 1);

    lora_log("Setup Tamam. Gorevler Dagitildi.");
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
