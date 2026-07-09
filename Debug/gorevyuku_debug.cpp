/*
================================================================================
  TRAKYA ROKET 2026 - UCUS YAZILIMI  ***GOREV YUKU DEBUG / TESHIS FORK'U***
================================================================================
  Bu dosya GorevYukuYazilimi/gorevyuku.cpp'nin BIREBIR davranisini korur
  (BME280 -> Kalman -> telemetryQueue -> SD + LoRa, BNO055 ile inis tespiti),
  fakat AYRICA HER SEYI USB seri monitore (115200) detaylica basar:
    - Baslangicta: pin haritasi, I2C bus taramasi, tum modul baglanti raporu
      (BME280 / SD / GPS / BNO055) adresleriyle, referans basinc, kuyruk.
    - ~10 Hz periyodik: ham vs Kalman sensor degerleri, GPS, inis tespiti,
      GorevYukuPaket icerigi, LoRa cercevesinin ham hex'i + CRC, kuyruk
      dolulugu, SD ping-pong durumu, free heap.

  DERLEME:  pio run -e gorevyukudebug --target upload
  MONITOR:  pio device monitor -b 115200

  NOT: GorevYukuYazilimi/gorevyuku.cpp'ye DOKUNULMAZ. Bu ayri bir PlatformIO
       ortami (gorevyukudebug).
================================================================================
*/

#include "driver/uart.h"   // DMA destekli UART surucusu
#include "esp_heap_caps.h" // DMA uyumlu bellek yonetimi
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <stdarg.h> // dbg_append(...) icin va_list

// ============================================================
//  >>> DEBUG AYARLARI <<<
// ============================================================
#define DBG_PRINT_MS 100 // Periyodik tam durum dokumu araligi (ms) ~10 Hz

// --- USB SERIAL CIKIS MODU ---
// 0 = NORMAL DEBUG: USB Serial'e insan-okunur tam durum dokumu + GPS ham +
// setup loglari. 1 = TEMIZ BINARY: USB Serial'e SADECE framed binary cerceve
// (AA 55 LEN payload CRC16,
//     ~10 Hz) basilir; TUM metin (dump, GPS ham, inis mesajlari, i2c/pin/setup
//     loglari) susturulur. Yer istasyonu parser'ini USB kablosundan dogrudan
//     besleyip test etmek icin. LoRa (UART1) tarafi bu bayraktan BAGIMSIZ,
//     kendi moduyla calisir.
#define SERIAL_FRAMED_OUTPUT 1

// Insan-okunur metin makrolari: SERIAL_FRAMED_OUTPUT=1 iken hicbir metin USB'ye
// yazilmaz. (Framed binary cerceve dogrudan Serial.write ile basilir, bu
// makrolardan bagimsiz.)
#if SERIAL_FRAMED_OUTPUT
#define DBGF(...) ((void)0)
#define DBGLN(x) ((void)0)
#define DBGPR(x) ((void)0)
#define DBGWR(x) ((void)0)
#else
#define DBGF(...) Serial.printf(__VA_ARGS__)
#define DBGLN(x) Serial.println(x)
#define DBGPR(x) Serial.print(x)
#define DBGWR(x) Serial.write(x)
#endif

// ============================================================
//  >>> YARISMA ALANI - LORA ADRES & KANAL AYARLARI <<<
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

// Uyarlanabilir Sabitler
float referans_basinc = 1013.25;

// Uyarlanabilir Pinler
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
#define PIN_BUZZER 12 // Kurtarma beacon buzzer
#define PIN_LED 13    // Kurtarma beacon LED

// Haberleşme Sabitleri
#define BAUD_GPS 9600 // UART2
#define GPS_HAM_VERI                                                           \
  1 // 1: GPS'ten gelen ham NMEA cumlelerini Serial'e bas, 0: kapat
#define BAUD_LORA 9600 // UART1
#define UART_BUFFER_SIZE 1024

// --- ÇERÇEVE PROTOKOLÜ (Framed Binary) ---
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

// --- LORA GÖNDERİM MODU SEÇİMİ ---
//   FRAMED : AA 55 LEN <payload> CRC16  -> ham binary, kompakt, CRC korumali
//   STRING : "$TELEGY,...*\r\n" CSV metin -> insan-okunur, basit alici (parse
//   kolay)
#define LORA_MODE_FRAMED 0
#define LORA_MODE_STRING 1
#define LORA_GONDERIM_MODU LORA_MODE_STRING

// STRING modunda LoRa'ya, serial monitordeki TAM debug dokumunun AYNISI
// basilir. Ama 9600 baud havada ~960 B/sn tasir; tam dokum ~1.5 KB oldugundan
// 10 Hz SIGMAZ. Bu yuzden LoRa'ya dokum su araligla (ms) gonderilir (serial
// yine tam hizda):
#define LORA_DUMP_MS 2000

// --- LORA GÖNDERİM HIZI ---
#define LORA_GONDERIM_ORANI 10

// Sensör Sabitleri
#define BME280_ADDR_PRIMARY 0x76
#define BME280_ADDR_SECONDARY 0x77
#define BNO055_DEF 55
#define BNO055_ADDR 0x28

// --- İNİŞ TESPİTİ & BEACON SABİTLERİ ---
#define KALKIS_YUKSEKLIK 50.0    // m
#define KALKIS_IVME_ESIGI 20.0   // m/s²
#define REST_HIZ_ESIGI 1.0       // m/s
#define REST_IVME_ESIGI 1.5      // m/s²
#define REST_GYRO_ESIGI 0.2      // rad/s
#define INIS_STABIL_SURE_MS 5000 // ms
#define BEACON_BIP_MS 200        // ms
#define BEACON_PERIYOT_MS 1000   // ms

// FreeRTOS Sabitleri
#define TASK_STACK_SIZE 10000
#define TASK1_PRIORITY 2
#define TASK2_PRIORITY 1
#define TELEMETRY_QUEUE_LEN 10

// Görev takipçisi (Task Handle) tanımları
TaskHandle_t Task1;
TaskHandle_t Task2;

// --- KALMAN FİLTRESİ SINIFI ---
class SimpleKalmanFilter {
public:
  SimpleKalmanFilter(float mea_e, float est_e, float q)
      : err_measure(mea_e), err_estimate(est_e), q(q), last_estimate(0),
        kalman_gain(0), first_run(true) {}

  float updateEstimate(float mea) {
    if (first_run) {
      last_estimate = mea;
      first_run = false;
    }
    kalman_gain = err_estimate / (err_estimate + err_measure);
    float current_estimate =
        last_estimate + kalman_gain * (mea - last_estimate);
    err_estimate = (1.0f - kalman_gain) * err_estimate +
                   fabs(last_estimate - current_estimate) * q;
    last_estimate = current_estimate;
    return current_estimate;
  }

private:
  float err_measure;
  float err_estimate;
  float q;
  float last_estimate;
  float kalman_gain;
  bool first_run;
};

// --- KALMAN FİLTRESİ NESNELERİ ---
SimpleKalmanFilter kf_basinc(2.0, 2.0, 0.1);
SimpleKalmanFilter kf_sicaklik(0.5, 0.5, 0.01);
SimpleKalmanFilter kf_nem(1.0, 1.0, 0.1);
SimpleKalmanFilter kf_irtifa(1.5, 1.5, 0.1);

// --- SENSÖR NESNELERİ ---
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_DEF, BNO055_ADDR);
TinyGPSPlus gps;
bool bnoOk = false;

// --- SENSÖR VERİ DEĞİŞKENLERİ ---
float basinc = 0.0, sicaklik = 0.0, nem = 0.0, irtifa = 0.0;
float gpsEnlem = 0.0, gpsBoylam = 0.0;

// --- İNİŞ TESPİTİ DURUMU ---
float max_irtifa = 0.0;
bool uctu = false;
bool indi = false;
unsigned long rest_baslangic = 0;

// Hız Hesaplama Değişkenleri
float onceki_irtifa = 0.0;
unsigned long onceki_zaman = 0;
float anlik_dikey_hiz = 0.0;

// --- GOREV YUKU TELEMETRİ YAPISI VE KUYRUK ---
#pragma pack(push, 1)
struct GorevYukuPaket {
  float basinc, sicaklik, nem, irtifa; // BME280
  float gpsEnlem, gpsBoylam;           // GPS
  float ivmeX, ivmeY, ivmeZ;           // BNO055 lineer ivme (m/s^2)
  float gyroX, gyroY, gyroZ;           // BNO055 gyro (rad/s)
}; // 48 byte
#pragma pack(pop)

// SD Kart Ping-Pong Buffer Tanımları
#define SD_DMA_BUF_SIZE 512
char *sd_dma_buf_A;
char *sd_dma_buf_B;
volatile int active_sd_buf = 0;
volatile int sd_buf_idx = 0;

QueueHandle_t telemetryQueue;
File logFile;
bool sdOk = false;

// ============================================================
//  >>> DEBUG PAYLASILAN DURUM (SNAPSHOT) <<<
//  Core 0 (Task1) doldurur, Core 1 (Task2) basar.
// ============================================================
struct DebugSnapshot {
  // Ham (filtresiz) sensor okumalari
  float ham_basinc, ham_sicaklik, ham_nem, ham_irtifa;
  // BNO055 (IMU) ham verileri
  float ham_ivmeX, ham_ivmeY, ham_ivmeZ;
  float ham_gyroX, ham_gyroY, ham_gyroZ;
  uint8_t cal_sys, cal_gyro, cal_accel, cal_mag;
  // GPS detay
  bool gps_valid;
  uint32_t gps_sat;
  float gps_hdop;
  float gps_alt;
  uint32_t gps_age;
  // GPS zaman/tarih (UTC)
  bool gps_time_valid;
  uint8_t gps_saat, gps_dakika, gps_saniye;
  uint8_t gps_gun, gps_ay;
  uint16_t gps_yil;
  // Inis Durumlari
  float max_irtifa;
  float anlik_dikey_hiz;
  bool uctu;
  bool indi;
  unsigned long rest_baslangic;
  // Zamanlama
  uint32_t dongu_sayaci;
  float dongu_hz;
};
volatile DebugSnapshot dbg = {};

// LoRa cercevesinin son hex dokumu (Task2'de doldurulur, ayni yerde basilir)
static uint8_t son_frame[200];
static size_t son_frame_len = 0;
static uint16_t son_crc = 0;

// --- DEBUG DOKUM BUFFER'I ---
static char dbg_buf[1536];
static int dbg_off = 0;

// printf gibi calisir ama dbg_buf'a ekler (tasma korumali).
static void dbg_append(const char *fmt, ...) {
  if (dbg_off >= (int)sizeof(dbg_buf) - 1)
    return;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(dbg_buf + dbg_off, sizeof(dbg_buf) - dbg_off, fmt, ap);
  va_end(ap);
  if (n < 0)
    return;
  dbg_off += n;
  if (dbg_off > (int)sizeof(dbg_buf) - 1)
    dbg_off = sizeof(dbg_buf) - 1; // kirpildi
}

// ============================================================
//  Hazır Kullanılacak Metodlar/Fonksiyonlar
// ============================================================

// Anlık Dikey Hız (Vz) Hesaplama
float hesapla_dikey_hiz(float guncel_irtifa) {
  unsigned long suanki_zaman = micros();
  if (onceki_zaman == 0) {
    onceki_zaman = suanki_zaman;
    onceki_irtifa = guncel_irtifa;
    return 0.0;
  }
  float delta_t = (float)(suanki_zaman - onceki_zaman) / 1000000.0f;
  if (delta_t <= 0.0) {
    return anlik_dikey_hiz;
  }
  float delta_irtifa = guncel_irtifa - onceki_irtifa;
  float hiz_z = delta_irtifa / delta_t;
  onceki_zaman = suanki_zaman;
  onceki_irtifa = guncel_irtifa;
  return hiz_z;
}

// --- CRC16-CCITT (poly=0x1021, init=0xFFFF) ---
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// --- BUFFERLI (PING-PONG) SD YAZMA ---
void bufferla_ve_yaz_sd(File &file, const GorevYukuPaket &pkt) {
  char temp_line[192];
  int line_len = snprintf(
      temp_line, sizeof(temp_line),
      "%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      pkt.basinc, pkt.sicaklik, pkt.nem, pkt.irtifa, pkt.gpsEnlem, pkt.gpsBoylam,
      pkt.ivmeX, pkt.ivmeY, pkt.ivmeZ, pkt.gyroX, pkt.gyroY, pkt.gyroZ);

  char *current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;

  if (sd_buf_idx + line_len >= SD_DMA_BUF_SIZE) {
    if (file) {
      file.write((const uint8_t *)current_buf, sd_buf_idx);
    }
    active_sd_buf = (active_sd_buf == 0) ? 1 : 0;
    current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
    sd_buf_idx = 0;
  }

  memcpy(&current_buf[sd_buf_idx], temp_line, line_len);
  sd_buf_idx += line_len;
}

// --- PING-PONG TAMPONUNU DISKE BOSALT ---
void sd_buffer_bosalt(File &file) {
  if (file && sd_buf_idx > 0) {
    char *current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
    file.write((const uint8_t *)current_buf, sd_buf_idx);
    sd_buf_idx = 0;
  }
  if (file)
    file.flush();
}

// --- ÇERÇEVE KUR (AA 55 LEN payload CRC16) ---
size_t build_framed(const GorevYukuPaket &pkt, uint8_t *out) {
  const uint8_t *payload = (const uint8_t *)&pkt;
  const size_t len = sizeof(GorevYukuPaket);
  uint16_t crc = crc16_ccitt(payload, len);

  size_t idx = 0;
  out[idx++] = SYNC_BYTE_1;
  out[idx++] = SYNC_BYTE_2;
  out[idx++] = (uint8_t)len;
  memcpy(&out[idx], payload, len);
  idx += len;
  out[idx++] = (uint8_t)(crc >> 8);
  out[idx++] = (uint8_t)(crc & 0xFF);

  // DEBUG: son kurulan cerceveyi sakla
  memcpy(son_frame, out, idx);
  son_frame_len = idx;
  son_crc = crc;
  return idx;
}

// --- ÇERÇEVELI PAKET GÖNDERME (DMA DESTEKLI UART) ---
void gonder_paket_framed_dma(uart_port_t uart_num, const GorevYukuPaket &pkt) {
  static uint8_t frame_buf[64];
  size_t idx = build_framed(pkt, frame_buf);
  uart_write_bytes(uart_num, (const char *)frame_buf, idx);
}

// --- ÇERÇEVELI PAKET GÖNDERME (USB SERIAL / UART0) ---
void gonder_paket_framed_serial(const GorevYukuPaket &pkt) {
  static uint8_t frame_buf[64];
  size_t idx = build_framed(pkt, frame_buf);
  Serial.write(frame_buf, idx);
}

// --- CSV METIN PAKET GÖNDERME (STRING modu) ---
void gonder_paket_string_dma(uart_port_t uart_num, const GorevYukuPaket &pkt) {
  static char str_buf[192];

  int n = snprintf(
      str_buf, sizeof(str_buf),
      "$TELEGY,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f*\r\n",
      pkt.basinc, pkt.sicaklik, pkt.nem, pkt.irtifa, pkt.gpsEnlem, pkt.gpsBoylam,
      pkt.ivmeX, pkt.ivmeY, pkt.ivmeZ, pkt.gyroX, pkt.gyroY, pkt.gyroZ);

  if (n < 0)
    return;
  if (n >= (int)sizeof(str_buf))
    n = sizeof(str_buf) - 1;

  uart_write_bytes(uart_num, str_buf, n);

  // DEBUG: son gonderileni sakla
  size_t cpy = ((size_t)n < sizeof(son_frame)) ? (size_t)n : sizeof(son_frame);
  memcpy(son_frame, str_buf, cpy);
  son_frame_len = cpy;
  son_crc = 0; // string modda CRC yok
}

// --- MOD SECIMLI LORA GONDERIM ---
void gonder_paket_lora(uart_port_t uart_num, const GorevYukuPaket &pkt) {
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
  gonder_paket_string_dma(uart_num, pkt);
#else
  gonder_paket_framed_dma(uart_num, pkt);
#endif
}

// --- E32-433T30D LORA MODUL KONFIGURASYONU ---
void lora_konfigurasyon() {
  static const uint8_t configPacket[6] = {0xC0,      LORA_ADDH, LORA_ADDL,
                                          LORA_SPED, LORA_CHAN, LORA_OPTION};

  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  uart_flush(UART_NUM_1);

  uart_write_bytes(UART_NUM_1, (const char *)configPacket,
                   sizeof(configPacket));
  uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(200));

  vTaskDelay(200 / portTICK_PERIOD_MS);
  uart_flush(UART_NUM_1);

  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  DBGF("  LoRa config paketi (6B): C0 %02X %02X %02X %02X %02X\n", LORA_ADDH,
       LORA_ADDL, LORA_SPED, LORA_CHAN, LORA_OPTION);
}

// --- SETUP TESHIS MESAJLARINI LORA UZERINDEN GONDER ---
void lora_log(const char *msg) {
  uart_write_bytes(UART_NUM_1, msg, strlen(msg));
  uart_write_bytes(UART_NUM_1, "\r\n", 2);
  DBGPR("[LoRa-log] ");
  DBGLN(msg);
}

// --- NON-BLOCKING BEACON BEHAVIOR ---
void beacon_guncelle() {
  if (!indi) {
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }
  digitalWrite(PIN_LED, HIGH);
  bool bip = (millis() % BEACON_PERIYOT_MS) < BEACON_BIP_MS;
  digitalWrite(PIN_BUZZER, bip ? HIGH : LOW);
}

// ============================================================
//  >>> DEBUG YARDIMCILARI <<<
// ============================================================

// I2C bus taramasi
void i2c_bus_tara() {
#if !SERIAL_FRAMED_OUTPUT
  Serial.println("\n--- I2C BUS TARAMASI (SDA=21, SCL=22) ---");
  int bulunan = 0;
  for (uint8_t adr = 0x01; adr < 0x7F; adr++) {
    Wire.beginTransmission(adr);
    uint8_t hata = Wire.endTransmission();
    if (hata == 0) {
      const char *etiket = "";
      if (adr == BNO055_ADDR)
        etiket = "  <- BNO055 (IMU - inis tespiti)";
      else if (adr == BME280_ADDR_PRIMARY)
        etiket = "  <- BME280 (barometre, primary)";
      else if (adr == BME280_ADDR_SECONDARY)
        etiket = "  <- BME280 (barometre, secondary)";
      Serial.printf("  CIHAZ BULUNDU: 0x%02X%s\n", adr, etiket);
      bulunan++;
    }
  }
  if (bulunan == 0)
    Serial.println(
        "  !! HICBIR I2C CIHAZI BULUNAMADI - kablolama/pull-up kontrol et!");
  else
    Serial.printf("  Toplam %d I2C cihazi bulundu.\n", bulunan);
  Serial.println("-----------------------------------------");
#endif
}

// Pin haritasini basar
void pin_haritasi_bas() {
#if !SERIAL_FRAMED_OUTPUT
  Serial.println("\n--- PIN HARITASI (GOREV YUKU) ---");
  Serial.printf("  I2C   : SDA=%d SCL=%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
  Serial.printf("  SPI/SD: CLK=%d MISO=%d MOSI=%d CS=%d  SD_DET=%d\n",
                PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS,
                PIN_SDKART_DET);
  Serial.printf("  GPS   : RX=%d TX=%d (UART2 @ %d)\n", PIN_GPS_RX, PIN_GPS_TX,
                BAUD_GPS);
  Serial.printf("  LoRa  : TX=%d RX=%d M0=%d M1=%d (UART1 @ %d)\n", PIN_LORA_TX,
                PIN_LORA_RX, LORA_M0, LORA_M1, BAUD_LORA);
  Serial.printf("  BEACON: LED=%d BUZZER=%d\n", PIN_LED, PIN_BUZZER);
  Serial.println("---------------------------------");
#endif
}

// Core 0'da çalışacak olan görevin fonsiyonu (Sensor Okuma, Kalman, Inis
// Tespiti)
void Task1code(void *pvParameters) {
  uint32_t sayac = 0;
  unsigned long son_hz_zaman = millis();
  uint32_t son_hz_sayac = 0;

  for (;;) {
    // 1. BME280 Verilerini Okuma
    float raw_temp = bme.readTemperature();
    float raw_pres = bme.readPressure() / 100.0f; // Pa -> hPa
    float raw_hum = bme.readHumidity();
    float raw_alt = bme.readAltitude(referans_basinc);

    dbg.ham_sicaklik = raw_temp;
    dbg.ham_basinc = raw_pres;
    dbg.ham_nem = raw_hum;
    dbg.ham_irtifa = raw_alt;

    sicaklik = kf_sicaklik.updateEstimate(raw_temp);
    basinc = kf_basinc.updateEstimate(raw_pres);
    nem = kf_nem.updateEstimate(raw_hum);
    irtifa = kf_irtifa.updateEstimate(raw_alt);

    // 2. GPS Verilerini Okuma
    while (Serial2.available() > 0) {
      char c = Serial2.read();
#if GPS_HAM_VERI && !SERIAL_FRAMED_OUTPUT
      Serial.write(c); // GPS ham verisini Serial'e yansit
#endif
      gps.encode(c);
    }
    if (gps.location.isUpdated()) {
      gpsEnlem = gps.location.lat();
      gpsBoylam = gps.location.lng();
    }
    // GPS detaylari
    dbg.gps_valid = gps.location.isValid();
    dbg.gps_sat = gps.satellites.value();
    dbg.gps_hdop = gps.hdop.hdop();
    dbg.gps_alt = gps.altitude.meters();
    dbg.gps_age = gps.location.age();
    // GPS saat/tarih (UTC)
    dbg.gps_time_valid = gps.time.isValid();
    dbg.gps_saat = gps.time.hour();
    dbg.gps_dakika = gps.time.minute();
    dbg.gps_saniye = gps.time.second();
    dbg.gps_gun = gps.date.day();
    dbg.gps_ay = gps.date.month();
    dbg.gps_yil = gps.date.year();

#if GPS_HAM_VERI && !SERIAL_FRAMED_OUTPUT
    if (gps.location.isUpdated()) {
      uint8_t saat_tr = (dbg.gps_saat + 3) % 24; // UTC -> Turkiye (UTC+3)
      Serial.printf(
          "\n[GPS] lat:%.6f lng:%.6f uydu:%lu hdop:%.2f alt:%.1f valid:%s | "
          "saat(UTC):%02u:%02u:%02u  TR:%02u:%02u:%02u  tarih:%02u/%02u/%04u\n",
          gpsEnlem, gpsBoylam, dbg.gps_sat, dbg.gps_hdop, dbg.gps_alt,
          dbg.gps_valid ? "EVET" : "HAYIR", dbg.gps_saat, dbg.gps_dakika,
          dbg.gps_saniye, saat_tr, dbg.gps_dakika, dbg.gps_saniye, dbg.gps_gun,
          dbg.gps_ay, dbg.gps_yil);
    }
#endif

    // 2.5 BNO055 (IMU)
    float ivmeX = 0.0, ivmeY = 0.0, ivmeZ = 0.0;
    float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
    if (bnoOk) {
      sensors_event_t a, g;
      bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
      ivmeX = a.acceleration.x;
      ivmeY = a.acceleration.y;
      ivmeZ = a.acceleration.z;
      bno.getEvent(&g, Adafruit_BNO055::VECTOR_GYROSCOPE);
      gyroX = g.gyro.x;
      gyroY = g.gyro.y;
      gyroZ = g.gyro.z;

      uint8_t cs = 0, cg = 0, ca = 0, cm = 0;
      bno.getCalibration(&cs, &cg, &ca, &cm);
      dbg.cal_sys = cs;
      dbg.cal_gyro = cg;
      dbg.cal_accel = ca;
      dbg.cal_mag = cm;
    } else {
      dbg.cal_sys = 0;
      dbg.cal_gyro = 0;
      dbg.cal_accel = 0;
      dbg.cal_mag = 0;
    }
    dbg.ham_ivmeX = ivmeX;
    dbg.ham_ivmeY = ivmeY;
    dbg.ham_ivmeZ = ivmeZ;
    dbg.ham_gyroX = gyroX;
    dbg.ham_gyroY = gyroY;
    dbg.ham_gyroZ = gyroZ;

    // 2.6 İniş Tespiti
    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);
    if (irtifa > max_irtifa)
      max_irtifa = irtifa;

    // Kalkis tespiti
    if (!uctu && (max_irtifa > KALKIS_YUKSEKLIK || ivmeZ > KALKIS_IVME_ESIGI)) {
      uctu = true;
      DBGLN("\n>>> HAREKET ALGILANDI: UCUS BASLADI! (irtifa > 50m veya ivmeZ > "
            "20)\n");
    }

    // Inis tespiti
    if (uctu && !indi) {
      float acc_mag = sqrtf(ivmeX * ivmeX + ivmeY * ivmeY + ivmeZ * ivmeZ);
      float gyro_mag = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
      bool baro_sabit = fabsf(anlik_dikey_hiz) < REST_HIZ_ESIGI;
      bool imu_durgun =
          (acc_mag < REST_IVME_ESIGI) && (gyro_mag < REST_GYRO_ESIGI);
      bool durgun = baro_sabit && (!bnoOk || imu_durgun);
      if (durgun) {
        if (rest_baslangic == 0)
          rest_baslangic = millis();
        else if (millis() - rest_baslangic >= INIS_STABIL_SURE_MS) {
          indi = true;
          DBGLN("\n>>> DUSTU VE DURDU: INIS TESPIT EDILDI! Kurtarma beacon "
                "aktif.\n");
        }
      } else {
        rest_baslangic = 0;
      }
    }
    dbg.max_irtifa = max_irtifa;
    dbg.anlik_dikey_hiz = anlik_dikey_hiz;
    dbg.uctu = uctu;
    dbg.indi = indi;
    dbg.rest_baslangic = rest_baslangic;

    // 2.7 Beacon LED / Buzzer
    beacon_guncelle();

    // 3. Paketle ve Core 1'e Gönder
    GorevYukuPaket packet;
    packet.basinc = basinc;
    packet.sicaklik = sicaklik;
    packet.nem = nem;
    packet.irtifa = irtifa;
    packet.gpsEnlem = gpsEnlem;
    packet.gpsBoylam = gpsBoylam;
    packet.ivmeX = ivmeX;
    packet.ivmeY = ivmeY;
    packet.ivmeZ = ivmeZ;
    packet.gyroX = gyroX;
    packet.gyroY = gyroY;
    packet.gyroZ = gyroZ;

    xQueueSend(telemetryQueue, &packet, 0);

    // Frekans hesabı
    sayac++;
    dbg.dongu_sayaci = sayac;
    if (millis() - son_hz_zaman >= 1000) {
      dbg.dongu_hz =
          (sayac - son_hz_sayac) * 1000.0f / (millis() - son_hz_zaman);
      son_hz_zaman = millis();
      son_hz_sayac = sayac;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // ~100 Hz
  }
}

// Core 1'de çalışacak olan görevin fonsiyonu (SD Log, LoRa & Debug Dökümü)
void Task2code(void *pvParameters) {
  GorevYukuPaket incomingPacket;
  uint32_t lora_sayac = 0;
  unsigned long son_basim = 0;

  for (;;) {
    if (xQueueReceive(telemetryQueue, &incomingPacket, portMAX_DELAY) ==
        pdTRUE) {
      // 1. SD Kart bufferlama
      if (sdOk && logFile) {
        bufferla_ve_yaz_sd(logFile, incomingPacket);
        static int flush_sayac = 0;
        if (++flush_sayac >= 100) {
          sd_buffer_bosalt(logFile);
          flush_sayac = 0;
        }
      }

      // 2. LoRa asenkron gönderimi
      lora_sayac++;
      bool tx_bu_dongu = false;
#if LORA_GONDERIM_MODU == LORA_MODE_FRAMED
      if (lora_sayac >= LORA_GONDERIM_ORANI) {
        gonder_paket_lora(UART_NUM_1, incomingPacket);
        lora_sayac = 0;
        tx_bu_dongu = true;
      }
#endif

#if SERIAL_FRAMED_OUTPUT
      // USB Serial'e framed binary cerceve bas.
      static uint32_t serial_frame_sayac = 0;
      if (++serial_frame_sayac >= LORA_GONDERIM_ORANI) {
        gonder_paket_framed_serial(incomingPacket);
        serial_frame_sayac = 0;
      }
#endif

      // 3. DEBUG: periyodik tam durum dokumu (~10 Hz)
      static unsigned long son_lora_dump = 0;
      if (millis() - son_basim >= DBG_PRINT_MS) {
        son_basim = millis();
        dbg_off = 0;

        bool lora_dump_bu_dongu = false;
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
        if (millis() - son_lora_dump >= LORA_DUMP_MS)
          lora_dump_bu_dongu = true;
#endif

        dbg_append(
            "\n============================================================\n");
        dbg_append("[t=%lu ms] dongu#%lu  ~%.1f Hz  |  GOREV YUKU  |  "
                   "max_irtifa=%.1f m\n",
                   millis(), dbg.dongu_sayaci, dbg.dongu_hz, dbg.max_irtifa);

        // --- BME280 ---
        dbg_append("-- BME280 (ham -> Kalman) --\n");
        dbg_append("  basinc :%8.2f -> %8.2f hPa\n", dbg.ham_basinc,
                   incomingPacket.basinc);
        dbg_append("  temp   :%6.2f -> %6.2f C\n", dbg.ham_sicaklik,
                   incomingPacket.sicaklik);
        dbg_append("  nem    :%5.1f -> %5.1f %%\n", dbg.ham_nem,
                   incomingPacket.nem);
        dbg_append("  irtifa :%8.2f -> %8.2f m   (ref_basinc=%.2f hPa)\n",
                   dbg.ham_irtifa, incomingPacket.irtifa, referans_basinc);

        // --- GPS ---
        dbg_append("-- GPS (GY-NEO-7M) --\n");
        dbg_append("  valid:%s  uydu:%lu  hdop:%.2f  lat:%.6f  lng:%.6f  "
                   "alt:%.1f m  fix_yasi:%lu ms\n",
                   dbg.gps_valid ? "EVET" : "HAYIR", dbg.gps_sat, dbg.gps_hdop,
                   incomingPacket.gpsEnlem, incomingPacket.gpsBoylam,
                   dbg.gps_alt, dbg.gps_age);
        dbg_append("  saat(UTC):%02u:%02u:%02u  TR:%02u:%02u:%02u  "
                   "tarih:%02u/%02u/%04u\n",
                   dbg.gps_saat, dbg.gps_dakika, dbg.gps_saniye,
                   (dbg.gps_saat + 3) % 24, dbg.gps_dakika, dbg.gps_saniye,
                   dbg.gps_gun, dbg.gps_ay, dbg.gps_yil);

        // --- IMU (BNO055) ---
        dbg_append("-- BNO055 (ham, inis tespiti icin) --\n");
        if (bnoOk) {
          dbg_append("  ivme X:%7.2f Y:%7.2f Z:%7.2f m/s^2\n", dbg.ham_ivmeX,
                     dbg.ham_ivmeY, dbg.ham_ivmeZ);
          dbg_append("  gyro X:%7.2f Y:%7.2f Z:%7.2f rad/s\n", dbg.ham_gyroX,
                     dbg.ham_gyroY, dbg.ham_gyroZ);
          dbg_append("  kalibrasyon: Sys=%d/3 Gyro=%d/3 Accel=%d/3 Mag=%d/3\n",
                     dbg.cal_sys, dbg.cal_gyro, dbg.cal_accel, dbg.cal_mag);
        } else {
          dbg_append("  BNO055 YANIT VERMEDI (pasif)\n");
        }

        // --- Inis Durumu ---
        dbg_append("-- INIS TESPITI --\n");
        dbg_append("  dikey_hiz: %7.2f m/s | uctu: %s | indi: %s\n",
                   dbg.anlik_dikey_hiz, dbg.uctu ? "EVET" : "HAYIR",
                   dbg.indi ? "EVET" : "HAYIR");
        if (dbg.uctu && !dbg.indi) {
          if (dbg.rest_baslangic > 0) {
            dbg_append("  durgunluk sayaci: %lu/%d ms\n",
                       millis() - dbg.rest_baslangic, INIS_STABIL_SURE_MS);
          } else {
            dbg_append("  durgunluk bekleniyor (hareketli)\n");
          }
        }

        // --- TelemetryPacket ozeti ---
        dbg_append("-- PAKET --  sizeof(GorevYukuPaket)=%u B\n",
                   (unsigned)sizeof(GorevYukuPaket));

        // --- LoRa TX durumu ---
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
        dbg_append("-- LoRa TX (STRING/DOKUM) --  ");
        if (lora_dump_bu_dongu)
          dbg_append("[BU DONGU TAM DOKUM TX] (~her %d ms)\n", LORA_DUMP_MS);
        else
          dbg_append("[bekliyor %lu/%d ms]\n", millis() - son_lora_dump,
                     LORA_DUMP_MS);
#else
        dbg_append("-- LoRa CERCEVE (FRAMED) --  ");
        if (tx_bu_dongu)
          dbg_append("[BU DONGU TX EDILDI] ");
        else
          dbg_append("[bu dongu TX yok, sayac %lu/%d] ", lora_sayac,
                     LORA_GONDERIM_ORANI);
        if (son_frame_len > 0) {
          dbg_append("(%u B, CRC16=0x%04X)\n  ", (unsigned)son_frame_len,
                     son_crc);
          for (size_t i = 0; i < son_frame_len; i++) {
            dbg_append("%02X ", son_frame[i]);
            if ((i & 0x0F) == 0x0F)
              dbg_append("\n  ");
          }
          dbg_append("\n");
        } else {
          dbg_append("(henuz cerceve gonderilmedi)\n");
        }
#endif

        // --- Kuyruk durumu ---
        dbg_append("-- KUYRUK --  bekleyen:%u/%d  bos_yer:%u\n",
                   (unsigned)uxQueueMessagesWaiting(telemetryQueue),
                   TELEMETRY_QUEUE_LEN,
                   (unsigned)uxQueueSpacesAvailable(telemetryQueue));

        // --- SD durumu ---
        dbg_append("-- SD --  sdOk:%d  aktif_buffer:%c  buf_idx:%d/%d\n", sdOk,
                   (active_sd_buf == 0) ? 'A' : 'B', sd_buf_idx,
                   SD_DMA_BUF_SIZE);

        // --- Sistem ---
        dbg_append("-- SISTEM --  free_heap:%u B\n",
                   (unsigned)ESP.getFreeHeap());

        // === TEK SEFERDE BAS ===
        DBGPR(dbg_buf);
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
        if (lora_dump_bu_dongu) {
          uart_write_bytes(UART_NUM_1, dbg_buf, dbg_off);
          son_lora_dump = millis();
        }
#endif
      }
    }
  }
}

void setup() {
  // 0. USB SERI MONITOR
  Serial.begin(115200);
  delay(400);
  DBGLN("\n\n################################################################");
  DBGLN("#  TRAKYA ROKET 2026 - GOREV YUKU [DEBUG / TESHIS FORK]        #");
  DBGLN("################################################################");
  DBGF("  Derleme: %s %s\n", __DATE__, __TIME__);
  DBGF("  ESP-IDF: %s  |  CPU: %d MHz  |  free_heap: %u B\n",
       esp_get_idf_version(), getCpuFrequencyMhz(),
       (unsigned)ESP.getFreeHeap());
  DBGF("  Ayarlar: DBG_PRINT_MS=%d (~%.0f Hz)  SERIAL_FRAMED_OUTPUT=%d\n",
       DBG_PRINT_MS, 1000.0 / DBG_PRINT_MS, SERIAL_FRAMED_OUTPUT);

  pin_haritasi_bas();

  // 1. Pin Modları
  pinMode(PIN_SDKART_DET, INPUT_PULLUP);
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED, LOW);

  // 2. DMA Buffer Allocation
  sd_dma_buf_A = (char *)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
  sd_dma_buf_B = (char *)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
  DBGF("  DMA buffer tahsisi: A=%s B=%s\n", sd_dma_buf_A ? "OK" : "BASARISIZ",
       sd_dma_buf_B ? "OK" : "BASARISIZ");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

  // DEBUG: I2C bus taramasi
  i2c_bus_tara();

  // 3. GPS (UART2)
  Serial2.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  DBGLN("\n[GPS] Serial2 (UART2) baslatildi @9600 - fix icin acik gokyuzu "
        "gerekir.");

  // 4. LoRa (UART1) DMA
  uart_config_t uart_config = {.baud_rate = BAUD_LORA,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0);
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, PIN_LORA_TX, PIN_LORA_RX, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  DBGLN("\n[LoRa] UART1 IDF surucusu kuruldu. Konfigurasyon gonderiliyor...");
  lora_konfigurasyon();

  vTaskDelay(500 / portTICK_PERIOD_MS);
  const char *start_msg = "--- GOREV YUKU SISTEMI BASLATILIYOR (DEBUG) ---\n";
  uart_write_bytes(UART_NUM_1, start_msg, strlen(start_msg));

  // 5. SD Kart Baslatma
  DBGLN("\n[SD] Baslatiliyor...");
  DBGF("  SD_DET pini (%d) durumu: %s\n", PIN_SDKART_DET,
       digitalRead(PIN_SDKART_DET) ? "HIGH (kart yok olabilir)"
                                   : "LOW (kart var)");
  if (!SD.begin(PIN_SPI_CS)) {
    lora_log("UYARI: SD Kart baslatilamadi! Loglama yapilmayacak.");
    sdOk = false;
  } else {
    DBGF("  SD OK. Boyut: %llu MB, Tip: %d\n",
         SD.cardSize() / (1024ULL * 1024ULL), SD.cardType());
    lora_log("SD Kart baslatildi.");
    logFile = SD.open("/gorevyuku_log.csv", FILE_APPEND);
    if (logFile) {
      if (logFile.size() == 0) {
        logFile.println("basinc,sicaklik,nem,irtifa,lat,lng,ivmeX,ivmeY,ivmeZ,"
                        "gyroX,gyroY,gyroZ");
      }
      sdOk = true;
    } else {
      lora_log("HATA: Log dosyasi acilamadi!");
      sdOk = false;
    }
  }

  // 6. BME280
  DBGLN("\n[BME280] Baslatiliyor (adres 0x76 -> 0x77)...");
  bool bme76 = bme.begin(BME280_ADDR_PRIMARY);
  bool bme77 = false;
  if (!bme76)
    bme77 = bme.begin(BME280_ADDR_SECONDARY);
  if (!bme76 && !bme77) {
    lora_log("KRITIK: BME280 bulunamadi! Sistem durduruluyor.");
    DBGLN("  !! BME280 hicbir adreste bulunamadi. Sistem duruyor.");
    while (true) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  DBGF("  BME280 bulundu @ 0x%02X\n",
       bme76 ? BME280_ADDR_PRIMARY : BME280_ADDR_SECONDARY);
  lora_log("BME280 baslatildi.");
  bme.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X2,
                  Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);

  // Yer Kalibrasyonu
  vTaskDelay(200 / portTICK_PERIOD_MS);
  lora_log("Yer kalibrasyonu yapiliyor...");
  float basinc_toplam = 0.0;
  for (int i = 0; i < 20; i++) {
    basinc_toplam += bme.readPressure() / 100.0F;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  referans_basinc = basinc_toplam / 20.0;
  {
    char buf[40];
    snprintf(buf, sizeof(buf), "Referans Basinc (hPa): %.2f", referans_basinc);
    lora_log(buf);
  }

  // 6.2 BNO055 (IMU)
  DBGLN("\n[BNO055] Baslatiliyor (adres 0x28, mod IMUPLUS 0x08)...");
  if (bno.begin(OPERATION_MODE_IMUPLUS)) {
    bnoOk = true;
    lora_log("BNO055 baslatildi (IMU mode 0x08) - inis tespiti icin.");
  } else {
    bnoOk = false;
    lora_log(
        "UYARI: BNO055 bulunamadi! Inis tespiti yalniz baro ile yapilacak.");
  }

  // 7. FreeRTOS Kuyruk
  telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(GorevYukuPaket));
  if (telemetryQueue == NULL) {
    lora_log("KRITIK: Kuyruk olusturulamadi! Sistem durduruluyor.");
    while (true) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
  DBGF("\n[KUYRUK] Olusturuldu: %d eleman x %u B = %u B\n", TELEMETRY_QUEUE_LEN,
       (unsigned)sizeof(GorevYukuPaket),
       (unsigned)(TELEMETRY_QUEUE_LEN * sizeof(GorevYukuPaket)));

  // 8. RTOS Görevleri
  xTaskCreatePinnedToCore(Task1code, "GYSensor", TASK_STACK_SIZE, NULL,
                          TASK1_PRIORITY, &Task1, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(Task2code, "GYHaberlesme", TASK_STACK_SIZE, NULL,
                          TASK2_PRIORITY, &Task2, 1);

  lora_log("Setup Tamam. Gorevler Dagitildi.");
  DBGLN("\n>>> SETUP TAMAM. Periyodik durum dokumu basliyor...\n");
}

void loop() { vTaskDelay(1000 / portTICK_PERIOD_MS); }
