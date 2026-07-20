/*
================================================================================
  TRAKYA ROKET 2026 - UCUS YAZILIMI  ***DEBUG / TESHIS FORK'U***
================================================================================
  Bu dosya src/main.cpp'nin BIREBIR davranisini korur (sensor -> Kalman ->
  durum makinesi -> telemetryQueue -> SD + LoRa), fakat AYRICA HER SEYI USB
  seri monitore (115200) detaylica basar:
    - Baslangicta: pin haritasi, I2C bus taramasi, tum modul baglanti raporu
      (BNO055 / BME280 / SD / GPS / LoRa) adresleriyle, referans basinc, kuyruk.
    - ~10 Hz periyodik: ham vs Kalman sensor degerleri, GPS, durum makinesi,
      apogee kosullari, TelemetryPacket icerigi, LoRa cercevesinin ham hex'i +
      CRC, kuyruk doluluğu, SD ping-pong durumu, free heap.

  DERLEME:  pio run -e ucusdebug --target upload
  MONITOR:  pio device monitor -b 115200

  NOT: src/main.cpp'ye DOKUNULMAZ. Bu ayri bir PlatformIO ortami (ucusdebug).
       USB seri (UART0, GPIO1/3) ana yazilimda bos oldugu icin cakisma yok.
================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <FS.h>
#include "driver/uart.h"      // DMA destekli UART sürücüsü
#include "esp_heap_caps.h"    // DMA uyumlu bellek yönetimi
#include <stdarg.h>           // dbg_append(...) icin va_list

// ============================================================
//  >>> DEBUG AYARLARI <<<
// ============================================================
#define DBG_PRINT_MS        100   // Periyodik tam durum dokumu araligi (ms) ~10 Hz
#define FUNYE_GERCEK_ATES   0     // 0 = SIMULE (pin surulmez, guvenli) | 1 = GERCEK atesleme

// --- USB SERIAL CIKIS MODU ---
// 0 = NORMAL DEBUG: USB Serial'e insan-okunur tam durum dokumu + GPS ham + setup loglari.
// 1 = TEMIZ BINARY: USB Serial'e SADECE framed binary cerceve (AA 55 LEN payload CRC16,
//     ~10 Hz) basilir; TUM metin (dump, GPS ham, funye/durum mesajlari, i2c/pin/setup
//     loglari) susturulur. Yer istasyonu parser'ini USB kablosundan dogrudan besleyip
//     test etmek icin. LoRa (UART1) tarafi bu bayraktan BAGIMSIZ, kendi moduyla calisir.
#define SERIAL_FRAMED_OUTPUT 1

// Insan-okunur metin makrolari: SERIAL_FRAMED_OUTPUT=1 iken hicbir metin USB'ye yazilmaz.
// (Framed binary cerceve dogrudan Serial.write ile basilir, bu makrolardan bagimsiz.)
#if SERIAL_FRAMED_OUTPUT
  #define DBGF(...)   ((void)0)
  #define DBGLN(x)    ((void)0)
  #define DBGPR(x)    ((void)0)
  #define DBGWR(x)    ((void)0)
#else
  #define DBGF(...)   Serial.printf(__VA_ARGS__)
  #define DBGLN(x)    Serial.println(x)
  #define DBGPR(x)    Serial.print(x)
  #define DBGWR(x)    Serial.write(x)
#endif

// ============================================================
//  >>> YARISMA ALANI - LORA ADRES & KANAL AYARLARI <<<
// ============================================================
#define LORA_ADDH    0x00   // Adres yuksek byte
#define LORA_ADDL    0x02   // Adres dusuk byte
#define LORA_CHAN    0x16   // Kanal (Gorev Yuku'nden FARKLI sec!) frekans=410+CHAN MHz
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
#define PIN_TTL_RX 1
#define PIN_TTL_TX 3
#define PIN_LED_2 4
#define PIN_SPI_CS 5
#define PIN_BUZZER 12
#define PIN_LED 13
#define PIN_FUNYE_2 14
#define PIN_GPS_RX 17
#define PIN_GPS_TX 16
#define PIN_SPI_CLK 18
#define PIN_SPI_MISO 19
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_SPI_MOSI 23
#define PIN_LED_3 25
#define PIN_LED_1 26
#define PIN_FUNYE_1 27
#define PIN_LORA_TX 33   // ESP TX -> LoRa DIN (modulun RXD'si)
#define PIN_LORA_RX 32   // ESP RX <- LoRa DOUT (modulun TXD'si)
#define PIN_SDKART_DET 35
#define LORA_M0 15
#define LORA_M1 2

// Haberleşme Sabitleri
#define BAUD_GPS               9600  // UART2  — GY-NEO-7M GPS modülü
#define GPS_HAM_VERI           1     // 1: GPS'ten gelen ham NMEA cumlelerini Serial'e bas, 0: kapat
#define BAUD_LORA              9600  // UART1  — E32-433T30D LoRa modülü (SX1278 tabanlı)
#define UART_BUFFER_SIZE       1024

// --- ÇERÇEVE PROTOKOLü (Framed Binary) ---
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55

// --- LORA GÖNDERİM MODU SEÇİMİ ---
//   FRAMED : AA 55 LEN <payload> CRC16  -> ham binary, kompakt, CRC korumali
//   STRING : "$TELE,...*\r\n" CSV metin -> insan-okunur, basit alici (parse kolay)
// Alici tarafin AYNI modu beklemesi sart. Sadece bu satiri degistir:
#define LORA_MODE_FRAMED     0
#define LORA_MODE_STRING     1
#define LORA_GONDERIM_MODU   LORA_MODE_FRAMED

// STRING modunda LoRa'ya, serial monitordeki TAM debug dokumunun AYNISI basilir.
// Ama 9600 baud havada ~960 B/sn tasir; tam dokum ~1.5 KB oldugundan 10 Hz SIGMAZ.
// Bu yuzden LoRa'ya dokum su araligla (ms) gonderilir (serial yine tam hizda):
#define LORA_DUMP_MS         2000

// --- LORA GÖNDERİM HIZI ---
#define LORA_GONDERIM_ORANI    10

// Sensör Sabitleri
#define BNO055_DEF 55
#define BNO055_ADDR 0x28
#define BME280_ADDR_PRIMARY 0x76
#define BME280_ADDR_SECONDARY 0x77

// Uçuş Algoritması Sabitleri
#define APOGEE_IRTIFA_FARKI   15.0  // m
#define AYRILMA2_MESAFE      550.0  // m
#define MAX_EGLIM             10.0  // derece
#define MIN_DIKEY_HIZ          0.0  // m/s
#define KALKIS_IVME_ESIGI     20.0  // m/s²
#define INIS_HIZ_ESIGI         2.0  // m/s
#define INIS_IRTIFA_ESIGI     20.0  // m
#define BNO055_MIN_KALIBRASYON 1    // 0-3

// FreeRTOS Sabitleri
#define TASK_STACK_SIZE 10000
#define TASK1_PRIORITY 2
#define TASK2_PRIORITY 1
#define TELEMETRY_QUEUE_LEN 10

// Görev takipçisi (Task Handle) tanımları
TaskHandle_t Task1;
TaskHandle_t Task2;

// Uçuş Durum Makinesi (State Machine)
enum UcusDurumu {
    HAZIR      = 0,
    YUKSELIYOR = 1,
    INIS_1     = 2,
    INIS_2     = 3,
    INDI       = 4
};
UcusDurumu durum = HAZIR;

const char* durum_adi(uint8_t d) {
    switch (d) {
        case HAZIR:      return "HAZIR";
        case YUKSELIYOR: return "YUKSELIYOR";
        case INIS_1:     return "INIS_1(drogue)";
        case INIS_2:     return "INIS_2(ana)";
        case INDI:       return "INDI";
        default:         return "???";
    }
}

//Uçuş Algoritması İçin Gerekli Değişkenler
bool ayrilma1 = false;
bool ayrilma2 = false;
float max_irtifa_degeri = 0.0;
// --- FÜNYE ZAMANLAMA (Non-Blocking) ---
#define FUNYE_SURE_MS 400
unsigned long funye1_baslangic = 0;
unsigned long funye2_baslangic = 0;
bool funye1_aktif = false;
bool funye2_aktif = false;

// --- HIZ HESAPLAMA DEĞİŞKENLERİ ---
float onceki_irtifa = 0.0;
unsigned long onceki_zaman = 0;
float anlik_dikey_hiz = 0.0;
float eglim_acisi = 0.0;

// --- KALMAN FİLTRESİ SINIFI ---
class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) :
      err_measure(mea_e), err_estimate(est_e), q(q), last_estimate(0), kalman_gain(0), first_run(true) {}

    float updateEstimate(float mea) {
      if (first_run) {
        last_estimate = mea;
        first_run = false;
      }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      float current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
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
SimpleKalmanFilter kf_ivmeX(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_ivmeY(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_ivmeZ(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_gyroX(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_gyroY(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_gyroZ(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_roll(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_pitch(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_yaw(2.906, 9.982, 0.3884);
// DIKKAT: e_mea'yi buyutmek filtreyi OLDURUR (K -> 0, irtifa donar). Gecmiste
// 16.3 denendi -> irtifa -0.1'de dondu, apogee tetiklenmedi. Detay: src/main.cpp
SimpleKalmanFilter kf_irtifa(1.5, 1.5, 0.1);

// --- SENSÖR NESNELERİ ---
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_DEF, BNO055_ADDR);
Adafruit_BME280 bme;
TinyGPSPlus gps;

// --- SENSÖR VERİ DEĞİŞKENLERİ ---
float ivmeX = 0.0, ivmeY = 0.0, ivmeZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float irtifa = 0.0;
float gpsEnlem = 0.0, gpsBoylam = 0.0;

// --- TELEMETRİ YAPISI VE KUYRUK ---
#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float irtifa;
    float dikeyHiz;
    float eglimAcisi;
    float gpsEnlem, gpsBoylam;
    bool ayrilma1_durum;
    bool ayrilma2_durum;
    uint8_t ucus_durumu;
};
#pragma pack(pop)

TelemetryPacket* dma_packet;

// SD Kart Ping-Pong Buffer Tanımları
#define SD_DMA_BUF_SIZE 512
char* sd_dma_buf_A;
char* sd_dma_buf_B;
volatile int active_sd_buf = 0;
volatile int sd_buf_idx = 0;

QueueHandle_t telemetryQueue;
File logFile;
bool sdOk = false;

// ============================================================
//  >>> DEBUG PAYLASILAN DURUM (SNAPSHOT) <<<
//  Core 0 (Task1) doldurur, Core 1 (Task2) basar. Tek basici -> interleaving yok.
//  Debug amacli iyi huylu yaris (torn read) kabul edilir.
// ============================================================
struct DebugSnapshot {
    // Ham (filtresiz) sensor okumalari
    float ham_ivmeX, ham_ivmeY, ham_ivmeZ;
    float ham_gyroX, ham_gyroY, ham_gyroZ;
    float ham_roll, ham_pitch, ham_yaw;   // NOT: yaw=o.x, roll=o.y, pitch=o.z (main.cpp eslemesi)
    float ham_irtifa;
    // Filtreli degerler (global ivmeX.. zaten filtreli)
    // BNO055 kalibrasyon
    uint8_t cal_sys, cal_gyro, cal_accel, cal_mag;
    // GPS detay
    bool  gps_valid;
    uint32_t gps_sat;
    float gps_hdop;
    float gps_alt;
    uint32_t gps_age;
    // GPS zaman/tarih (UTC — TinyGPS++ ham degerleri)
    bool gps_time_valid;
    uint8_t gps_saat, gps_dakika, gps_saniye;   // UTC saat
    uint8_t gps_gun, gps_ay;
    uint16_t gps_yil;
    // Apogee kosul bilesenleri
    bool apo_A, apo_B, apo_D;
    // Zamanlama
    uint32_t dongu_sayaci;
    float dongu_hz;
    // BME ham cevresel (debug icin ekstra okunur)
    float bme_basinc_hpa, bme_sicaklik, bme_nem;
};
volatile DebugSnapshot dbg = {};

// LoRa cercevesinin son hex dokumu (Task2'de doldurulur, ayni yerde basilir)
// Not: STRING modda CSV metni framed binary'den uzun olabilir, bol tut.
static uint8_t son_frame[200];
static size_t  son_frame_len = 0;
static uint16_t son_crc = 0;

// --- DEBUG DOKUM BUFFER'I ---
// Serial monitore basilan tam durum dokumu once buraya kurulur; ayni buffer
// hem Serial'e hem (STRING modunda) LoRa'ya basilir -> ikisi BIREBIR ayni olur.
static char dbg_buf[2048];
static int  dbg_off = 0;

// printf gibi calisir ama dbg_buf'a ekler (tasma korumali).
static void dbg_append(const char* fmt, ...) {
    if (dbg_off >= (int)sizeof(dbg_buf) - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(dbg_buf + dbg_off, sizeof(dbg_buf) - dbg_off, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    dbg_off += n;
    if (dbg_off > (int)sizeof(dbg_buf) - 1) dbg_off = sizeof(dbg_buf) - 1;  // kirpildi
}

// ============================================================
//  Hazır Kullanılacak Metodlar/Fonksiyonlar
// ============================================================

// ============================================================================
//  *** FUNYE PIN GUVENLIGI — BU BLOK ASLA VE ASLA DEGISTIRILMEYECEK ***
// ============================================================================
// KURAL: Funye pinleri setup()'ta OUTPUT YAPILMAZ. Normalde high-Z (INPUT)
// olarak durur; gate'i harici pull-down GND'ye kilitler. Pin OUTPUT'a YALNIZCA
// funye_pin_ates() icinde, ateslemeden hemen once gecer; FUNYE_SURE_MS dolunca
// funye_pin_serbest() ile tekrar high-Z'ye birakilir.
//
// NEDEN (dokunmadan once oku):
//  1) Boot/reset sirasinda ESP32 pinleri high-Z'dir. setup()'ta OUTPUT yapmak
//     pini surulur hale getirir; o andan itibaren tek bir hatali digitalWrite
//     (bozuk bellek, stack tasmasi, kacak task, brown-out sonrasi yarim reset)
//     gercek funyeyi ateslemeye yeter.
//  2) High-Z'de MOSFET gate'ini pull-down direnci tutar — yazilim ne yaparsa
//     yapsin fiziksel olarak akim akmaz. Yazilim hatasi donanim guvenligini
//     asamaz.
//  3) Bu debug sketch'i masada, insan yanindayken calisiyor. FUNYE_GERCEK_ATES
//     yanlislikla 1 birakilirsa tek koruma bu high-Z katmanidir. "Temizlik
//     olsun" diye setup'a pinMode(PIN_FUNYE_x, OUTPUT) EKLEME.
//
// Sirayi da bozma: once digitalWrite(LOW), sonra pinMode. Ters sira,
// OUTPUT'a gecis aninda registerde kalmis eski HIGH'i pine basar (glitch).
// Bu davranis src/main.cpp ile birebir ayni tutulmalidir.
// ============================================================================
static inline void funye_pin_serbest(uint8_t pin) {
    digitalWrite(pin, LOW);      // cikis registerini once temizle
    pinMode(pin, INPUT);         // high-Z — surucu tamamen devre disi
}

static inline void funye_pin_ates(uint8_t pin) {
    digitalWrite(pin, LOW);      // OUTPUT'a gecerken glitch olmasin
    pinMode(pin, OUTPUT | PULLDOWN);
    digitalWrite(pin, HIGH);
}

// Non-Blocking fünye ateşleme
void Funye1Atesle(){
    if (!funye1_aktif) {
#if FUNYE_GERCEK_ATES
        funye_pin_ates(PIN_FUNYE_1);
#endif
        funye1_baslangic = millis();
        funye1_aktif = true;
        ayrilma1 = true;
        DBGF("\n!!!!! FUNYE1 ATESLENDI %s  (t=%lu ms) apogee: A=%d B=%d D=%d  max=%.1f irtifa=%.1f\n\n",
                      FUNYE_GERCEK_ATES ? "[GERCEK]" : "[SIMULE]",
                      funye1_baslangic, dbg.apo_A, dbg.apo_B, dbg.apo_D,
                      max_irtifa_degeri, irtifa);
    }
}

void Funye2Atesle(){
    if (!funye2_aktif) {
#if FUNYE_GERCEK_ATES
        funye_pin_ates(PIN_FUNYE_2);
#endif
        funye2_baslangic = millis();
        funye2_aktif = true;
        ayrilma2 = true;
        DBGF("\n!!!!! FUNYE2 ATESLENDI %s  (t=%lu ms) irtifa=%.1f\n\n",
                      FUNYE_GERCEK_ATES ? "[GERCEK]" : "[SIMULE]",
                      funye2_baslangic, irtifa);
    }
}

// Her döngüde çağrılmalı — süresi dolan fünyeyi kapatır
void funye_guncelle() {
    if (funye1_aktif && (millis() - funye1_baslangic >= FUNYE_SURE_MS)) {
#if FUNYE_GERCEK_ATES
        funye_pin_serbest(PIN_FUNYE_1);
#endif
        funye1_aktif = false;
    }
    if (funye2_aktif && (millis() - funye2_baslangic >= FUNYE_SURE_MS)) {
#if FUNYE_GERCEK_ATES
        funye_pin_serbest(PIN_FUNYE_2);
#endif
        funye2_aktif = false;
    }
}

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
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
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
void bufferla_ve_yaz_sd(File& file, const TelemetryPacket& pkt) {
    char temp_line[160];
    int line_len = snprintf(temp_line, sizeof(temp_line),
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%d,%d,%d\n",
        pkt.ivmeX, pkt.ivmeY, pkt.ivmeZ, pkt.gyroX, pkt.gyroY, pkt.gyroZ,
        pkt.roll, pkt.pitch, pkt.yaw, pkt.irtifa, pkt.dikeyHiz, pkt.eglimAcisi,
        pkt.gpsEnlem, pkt.gpsBoylam, pkt.ayrilma1_durum, pkt.ayrilma2_durum, pkt.ucus_durumu);

    char* current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;

    if (sd_buf_idx + line_len >= SD_DMA_BUF_SIZE) {
        if (file) {
            file.write((const uint8_t*)current_buf, sd_buf_idx);
        }
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

// --- ÇERÇEVE KUR (AA 55 LEN payload CRC16) — tek yer, hem UART hem Serial kullanir ---
// out[] en az sizeof(TelemetryPacket)+5 (=64) byte olmali. Kurulan cerceve uzunlugunu doner.
// Ayrica DEBUG global son_frame/son_frame_len/son_crc guncellenir (hex dokumu icin).
size_t build_framed(const TelemetryPacket& pkt, uint8_t* out) {
    const uint8_t* payload = (const uint8_t*)&pkt;
    const size_t   len     = sizeof(TelemetryPacket);
    uint16_t       crc     = crc16_ccitt(payload, len);

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

// --- ÇERÇEVELI PAKET GÖNDERME (DMA DESTEKLI UART / LoRa) ---
void gonder_paket_framed_dma(uart_port_t uart_num, const TelemetryPacket& pkt) {
    static uint8_t frame_buf[80];
    size_t idx = build_framed(pkt, frame_buf);
    uart_write_bytes(uart_num, (const char*)frame_buf, idx);
}

// --- ÇERÇEVELI PAKET GÖNDERME (USB SERIAL / UART0) ---
// SERIAL_FRAMED_OUTPUT modunda cagrilir: ayni binary cerceveyi USB Serial'e basar.
void gonder_paket_framed_serial(const TelemetryPacket& pkt) {
    static uint8_t frame_buf[80];
    size_t idx = build_framed(pkt, frame_buf);
    Serial.write(frame_buf, idx);
}

// --- CSV METIN PAKET GÖNDERME (STRING modu) ---
// Format: $TELE,ivmeX,ivmeY,ivmeZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,
//         irtifa,dikeyHiz,eglimAcisi,gpsEnlem,gpsBoylam,ayr1,ayr2,durum*\r\n
// DEBUG: gonderilen metin son_frame'e de yazilir ki Task2 hex/metin dokebilsin.
void gonder_paket_string_dma(uart_port_t uart_num, const TelemetryPacket& pkt) {
    static char str_buf[200];

    int n = snprintf(str_buf, sizeof(str_buf),
        "$TELE,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%d,%d,%u*\r\n",
        pkt.ivmeX, pkt.ivmeY, pkt.ivmeZ,
        pkt.gyroX, pkt.gyroY, pkt.gyroZ,
        pkt.roll,  pkt.pitch, pkt.yaw,
        pkt.irtifa, pkt.dikeyHiz, pkt.eglimAcisi,
        pkt.gpsEnlem, pkt.gpsBoylam,
        (int)pkt.ayrilma1_durum, (int)pkt.ayrilma2_durum,
        (unsigned)pkt.ucus_durumu);

    if (n < 0) return;                              // snprintf hatasi
    if (n >= (int)sizeof(str_buf)) n = sizeof(str_buf) - 1;  // taskin -> kirp

    uart_write_bytes(uart_num, str_buf, n);

    // DEBUG: son gonderileni sakla (hex/metin dokumu icin)
    size_t cpy = ((size_t)n < sizeof(son_frame)) ? (size_t)n : sizeof(son_frame);
    memcpy(son_frame, str_buf, cpy);
    son_frame_len = cpy;
    son_crc = 0;   // string modda CRC yok
}

// --- MOD SECIMLI LORA GONDERIM (dispatcher) ---
// LORA_GONDERIM_MODU makrosuna gore framed binary veya CSV metin basar.
void gonder_paket_lora(uart_port_t uart_num, const TelemetryPacket& pkt) {
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
    gonder_paket_string_dma(uart_num, pkt);
#else
    gonder_paket_framed_dma(uart_num, pkt);
#endif
}

// --- E32-433T30D LORA MODUL KONFIGURASYONU ---
void lora_konfigurasyon() {
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

    // DEBUG: gonderilen config paketini USB'ye de bas
    DBGF("  LoRa config paketi (6B): C0 %02X %02X %02X %02X %02X\n",
                  LORA_ADDH, LORA_ADDL, LORA_SPED, LORA_CHAN, LORA_OPTION);
    DBGF("  -> ADDH=0x%02X ADDL=0x%02X SPED=0x%02X CHAN=0x%02X (frekans=%d MHz) OPTION=0x%02X\n",
                  LORA_ADDH, LORA_ADDL, LORA_SPED, LORA_CHAN, 410 + LORA_CHAN, LORA_OPTION);
}

// --- SETUP TESHIS MESAJLARINI LORA UZERINDEN GONDER ---
// DEBUG: ayni mesaj hem LoRa hem USB seri monitore gider.
void lora_log(const char* msg) {
    uart_write_bytes(UART_NUM_1, msg, strlen(msg));
    uart_write_bytes(UART_NUM_1, "\r\n", 2);
    DBGPR("[LoRa-log] ");
    DBGLN(msg);
}

// ============================================================
//  >>> DEBUG YARDIMCILARI <<<
// ============================================================

// I2C bus taramasi (0x01..0x7F). Bulunan adresleri etiketleriyle basar.
void i2c_bus_tara() {
#if !SERIAL_FRAMED_OUTPUT
    Serial.println("\n--- I2C BUS TARAMASI (SDA=21, SCL=22) ---");
    int bulunan = 0;
    for (uint8_t adr = 0x01; adr < 0x7F; adr++) {
        Wire.beginTransmission(adr);
        uint8_t hata = Wire.endTransmission();
        if (hata == 0) {
            const char* etiket = "";
            if (adr == BNO055_ADDR)             etiket = "  <- BNO055 (IMU)";
            else if (adr == BME280_ADDR_PRIMARY)   etiket = "  <- BME280 (barometre, primary)";
            else if (adr == BME280_ADDR_SECONDARY) etiket = "  <- BME280 (barometre, secondary)";
            Serial.printf("  CIHAZ BULUNDU: 0x%02X%s\n", adr, etiket);
            bulunan++;
        }
    }
    if (bulunan == 0)
        Serial.println("  !! HICBIR I2C CIHAZI BULUNAMADI - kablolama/pull-up kontrol et!");
    else
        Serial.printf("  Toplam %d I2C cihazi bulundu.\n", bulunan);
    Serial.println("-----------------------------------------");
#endif
}

// Pin haritasini basar
void pin_haritasi_bas() {
#if !SERIAL_FRAMED_OUTPUT
    Serial.println("\n--- PIN HARITASI ---");
    Serial.printf("  I2C   : SDA=%d SCL=%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.printf("  SPI/SD: CLK=%d MISO=%d MOSI=%d CS=%d  SD_DET=%d\n",
                  PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS, PIN_SDKART_DET);
    Serial.printf("  GPS   : RX=%d TX=%d (UART2 @ %d)\n", PIN_GPS_RX, PIN_GPS_TX, BAUD_GPS);
    Serial.printf("  LoRa  : TX=%d RX=%d M0=%d M1=%d (UART1 @ %d)\n",
                  PIN_LORA_TX, PIN_LORA_RX, LORA_M0, LORA_M1, BAUD_LORA);
    Serial.printf("  FUNYE : F1=%d F2=%d  (atesleme modu: %s)\n",
                  PIN_FUNYE_1, PIN_FUNYE_2, FUNYE_GERCEK_ATES ? "GERCEK" : "SIMULE");
    Serial.printf("  LED   : LED=%d L1=%d L2=%d L3=%d  BUZZER=%d\n",
                  PIN_LED, PIN_LED_1, PIN_LED_2, PIN_LED_3, PIN_BUZZER);
    Serial.println("--------------------");
#endif
}

// Core 0'da çalışacak olan görevin fonsiyonu
void Task1code(void *pvParameters) {
  uint32_t sayac = 0;
  unsigned long son_hz_zaman = millis();
  uint32_t son_hz_sayac = 0;

  for (;;) {
    // 1. IMU (BNO055) Verilerini Okuma
    sensors_event_t a, g, o;
    bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
    dbg.ham_ivmeX = a.acceleration.x;
    dbg.ham_ivmeY = a.acceleration.y;
    dbg.ham_ivmeZ = a.acceleration.z;
    ivmeX = kf_ivmeX.updateEstimate(a.acceleration.x);
    ivmeY = kf_ivmeY.updateEstimate(a.acceleration.y);
    ivmeZ = kf_ivmeZ.updateEstimate(a.acceleration.z);

    bno.getEvent(&g, Adafruit_BNO055::VECTOR_GYROSCOPE);
    dbg.ham_gyroX = g.gyro.x;
    dbg.ham_gyroY = g.gyro.y;
    dbg.ham_gyroZ = g.gyro.z;
    gyroX = kf_gyroX.updateEstimate(g.gyro.x);
    gyroY = kf_gyroY.updateEstimate(g.gyro.y);
    gyroZ = kf_gyroZ.updateEstimate(g.gyro.z);

    bno.getEvent(&o, Adafruit_BNO055::VECTOR_EULER);
    dbg.ham_yaw   = o.orientation.x;
    dbg.ham_roll  = o.orientation.y;
    dbg.ham_pitch = o.orientation.z;
    yaw = kf_yaw.updateEstimate(o.orientation.x);
    roll = kf_roll.updateEstimate(o.orientation.y);
    pitch = kf_pitch.updateEstimate(o.orientation.z);

    // DEBUG: kalibrasyon her dongude okunur
    { uint8_t cs=0, cg=0, ca=0, cm=0;
      bno.getCalibration(&cs, &cg, &ca, &cm);
      dbg.cal_sys=cs; dbg.cal_gyro=cg; dbg.cal_accel=ca; dbg.cal_mag=cm; }

    // Eğim açısı
    float p_rad = pitch * DEG_TO_RAD;
    float r_rad = roll * DEG_TO_RAD;
    float cos_val = cos(p_rad) * cos(r_rad);
    cos_val = constrain(cos_val, -1.0f, 1.0f);
    eglim_acisi = acos(cos_val) * RAD_TO_DEG;

    // 2. Barometre (BME280) — DEBUG: cevresel degerler de okunur (gozlem icin)
    dbg.ham_irtifa   = bme.readAltitude(referans_basinc);
    dbg.bme_basinc_hpa = bme.readPressure() / 100.0F;
    dbg.bme_sicaklik   = bme.readTemperature();
    dbg.bme_nem        = bme.readHumidity();
    irtifa = kf_irtifa.updateEstimate(dbg.ham_irtifa);

    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);

    // 3. GPS Verilerini Okuma
    while (Serial2.available() > 0) {
        char c = Serial2.read();
#if GPS_HAM_VERI && !SERIAL_FRAMED_OUTPUT
        Serial.write(c);   // GPS'ten gelen ham NMEA verisini oldugu gibi Serial'e yansit
#endif
        gps.encode(c);
    }
    if (gps.location.isUpdated()) {
        gpsEnlem = gps.location.lat();
        gpsBoylam = gps.location.lng();
    }
    // DEBUG: GPS detaylari
    dbg.gps_valid = gps.location.isValid();
    dbg.gps_sat   = gps.satellites.value();
    dbg.gps_hdop  = gps.hdop.hdop();
    dbg.gps_alt   = gps.altitude.meters();
    dbg.gps_age   = gps.location.age();
    // GPS saat/tarih (UTC)
    dbg.gps_time_valid = gps.time.isValid();
    dbg.gps_saat   = gps.time.hour();
    dbg.gps_dakika = gps.time.minute();
    dbg.gps_saniye = gps.time.second();
    dbg.gps_gun    = gps.date.day();
    dbg.gps_ay     = gps.date.month();
    dbg.gps_yil    = gps.date.year();

#if GPS_HAM_VERI && !SERIAL_FRAMED_OUTPUT
    // Okuma aninda parse edilmis GPS ozeti (throttle yok)
    if (gps.location.isUpdated()) {
        uint8_t saat_tr = (dbg.gps_saat + 3) % 24;   // UTC -> Turkiye (UTC+3)
        Serial.printf("\n[GPS] lat:%.6f lng:%.6f uydu:%lu hdop:%.2f alt:%.1f valid:%s | "
                      "saat(UTC):%02u:%02u:%02u  TR:%02u:%02u:%02u  tarih:%02u/%02u/%04u  saat_gecerli:%s\n",
                      gpsEnlem, gpsBoylam, dbg.gps_sat, dbg.gps_hdop, dbg.gps_alt,
                      dbg.gps_valid ? "EVET" : "HAYIR",
                      dbg.gps_saat, dbg.gps_dakika, dbg.gps_saniye,
                      saat_tr, dbg.gps_dakika, dbg.gps_saniye,
                      dbg.gps_gun, dbg.gps_ay, dbg.gps_yil,
                      dbg.gps_time_valid ? "EVET" : "HAYIR");
    }
#endif

    // --- FÜNYE ZAMANLAMA KONTROLÜ ---
    funye_guncelle();

    // --- UÇUŞ ALGORİTMASI ---
    switch (durum) {
        case HAZIR:
            if (ivmeZ > KALKIS_IVME_ESIGI) {
                durum = YUKSELIYOR;
                DBGF("\n>>> DURUM DEGISTI: HAZIR -> YUKSELIYOR (ivmeZ=%.2f > %.1f)\n\n",
                              ivmeZ, KALKIS_IVME_ESIGI);
            }
            break;

        case YUKSELIYOR:
            if (irtifa > max_irtifa_degeri) {
                max_irtifa_degeri = irtifa;
            }
            // Apogee kosul bilesenleri (debug icin tek tek saklanir)
            dbg.apo_A = (max_irtifa_degeri - irtifa > APOGEE_IRTIFA_FARKI);
            dbg.apo_B = (anlik_dikey_hiz < MIN_DIKEY_HIZ);
            dbg.apo_D = (eglim_acisi < MAX_EGLIM);
            if (dbg.apo_A && dbg.apo_B && dbg.apo_D) {
                Funye1Atesle();
                durum = INIS_1;
            }
            break;

        case INIS_1:
            if ((irtifa < AYRILMA2_MESAFE) && (max_irtifa_degeri > AYRILMA2_MESAFE)) {
                Funye2Atesle();
                durum = INIS_2;
            }
            break;

        case INIS_2:
            if ((anlik_dikey_hiz > -INIS_HIZ_ESIGI) && (irtifa < INIS_IRTIFA_ESIGI)) {
                durum = INDI;
                DBGLN("\n>>> DURUM DEGISTI: INIS_2 -> INDI (yere inis)\n");
            }
            break;

        case INDI:
            break;
    }

    // --- STRUCT DOLDURMA VE CORE 1'E GÖNDERME ---
    TelemetryPacket packet;
    packet.ivmeX = ivmeX; packet.ivmeY = ivmeY; packet.ivmeZ = ivmeZ;
    packet.gyroX = gyroX; packet.gyroY = gyroY; packet.gyroZ = gyroZ;
    packet.roll = roll; packet.pitch = pitch; packet.yaw = yaw;
    packet.irtifa = irtifa;
    packet.dikeyHiz = anlik_dikey_hiz;
    packet.eglimAcisi = eglim_acisi;
    packet.gpsEnlem = gpsEnlem; packet.gpsBoylam = gpsBoylam;
    packet.ayrilma1_durum = ayrilma1; packet.ayrilma2_durum = ayrilma2;
    packet.ucus_durumu = (uint8_t)durum;

    xQueueSend(telemetryQueue, &packet, 0);

    // DEBUG: dongu sayaci + gercek frekans hesabi
    sayac++;
    dbg.dongu_sayaci = sayac;
    if (millis() - son_hz_zaman >= 1000) {
        dbg.dongu_hz = (sayac - son_hz_sayac) * 1000.0f / (millis() - son_hz_zaman);
        son_hz_zaman = millis();
        son_hz_sayac = sayac;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // ~100 Hz
  }
}

// Core 1'de çalışacak olan görevin fonsiyonu
void Task2code(void *pvParameters) {

  TelemetryPacket incomingPacket;
  uint32_t lora_sayac = 0;
  unsigned long son_basim = 0;

  for (;;) {
    if (xQueueReceive(telemetryQueue, &incomingPacket, portMAX_DELAY) == pdTRUE) {

        // 1. SD Kart — Ping-Pong Buffer ile Logla (~100 Hz)
        if (sdOk && logFile) {
            bufferla_ve_yaz_sd(logFile, incomingPacket);
            static int flush_sayac = 0;
            if (++flush_sayac >= 100) {
                sd_buffer_bosalt(logFile);
                flush_sayac = 0;
            }
            if (incomingPacket.ucus_durumu == INDI) {
                sd_buffer_bosalt(logFile);
            }
        }

        // 2. LoRa (E32-433T30D) — Asenkron DMA Gönderimi
        // FRAMED modunda: her LORA_GONDERIM_ORANI pakette bir binary cerceve (~10 Hz).
        // STRING modunda: burada gonderilmez; asagida TAM debug dokumu LoRa'ya basilir.
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
        // TEMIZ BINARY MOD: ayni framed cerceveyi USB Serial'e (~10 Hz) bas.
        // LoRa modundan BAGIMSIZ ayri sayac (LoRa STRING modunda da calisir).
        static uint32_t serial_frame_sayac = 0;
        if (++serial_frame_sayac >= LORA_GONDERIM_ORANI) {
            gonder_paket_framed_serial(incomingPacket);
            serial_frame_sayac = 0;
        }
#endif

        // 3. DEBUG: periyodik tam durum dokumu (~10 Hz)
        // Dokum once dbg_buf'a kurulur; AYNI buffer hem Serial'e (her zaman) hem
        // STRING modunda LoRa'ya (LORA_DUMP_MS'de bir) basilir -> ikisi BIREBIR ayni.
        static unsigned long son_lora_dump = 0;
        if (millis() - son_basim >= DBG_PRINT_MS) {
            son_basim = millis();
            dbg_off = 0;   // dokum buffer'ini sifirla

            // Bu dongu LoRa'ya tam dokum gonderilecek mi? (bandwidth throttle)
            bool lora_dump_bu_dongu = false;
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
            if (millis() - son_lora_dump >= LORA_DUMP_MS) lora_dump_bu_dongu = true;
#endif

            dbg_append("\n============================================================\n");
            dbg_append("[t=%lu ms] dongu#%lu  ~%.1f Hz  |  DURUM: %s  max_irtifa=%.1f m\n",
                          millis(), dbg.dongu_sayaci, dbg.dongu_hz,
                          durum_adi(incomingPacket.ucus_durumu), max_irtifa_degeri);

            // --- BNO055: ham vs Kalman ---
            dbg_append("-- BNO055 (ham -> Kalman) --\n");
            dbg_append("  ivme  X:%7.2f->%7.2f  Y:%7.2f->%7.2f  Z:%7.2f->%7.2f  m/s^2\n",
                          dbg.ham_ivmeX, incomingPacket.ivmeX, dbg.ham_ivmeY, incomingPacket.ivmeY,
                          dbg.ham_ivmeZ, incomingPacket.ivmeZ);
            dbg_append("  gyro  X:%7.2f->%7.2f  Y:%7.2f->%7.2f  Z:%7.2f->%7.2f  rad/s\n",
                          dbg.ham_gyroX, incomingPacket.gyroX, dbg.ham_gyroY, incomingPacket.gyroY,
                          dbg.ham_gyroZ, incomingPacket.gyroZ);
            dbg_append("  euler roll:%7.2f->%7.2f pitch:%7.2f->%7.2f yaw:%7.2f->%7.2f deg\n",
                          dbg.ham_roll, incomingPacket.roll, dbg.ham_pitch, incomingPacket.pitch,
                          dbg.ham_yaw, incomingPacket.yaw);
            dbg_append("  kalibrasyon: Sys=%d/3 Gyro=%d/3 Accel=%d/3 Mag=%d/3\n",
                          dbg.cal_sys, dbg.cal_gyro, dbg.cal_accel, dbg.cal_mag);

            // --- BME280 ---
            dbg_append("-- BME280 --\n");
            dbg_append("  irtifa ham:%8.2f -> Kalman:%8.2f m   (ref_basinc=%.2f hPa)\n",
                          dbg.ham_irtifa, incomingPacket.irtifa, referans_basinc);
            dbg_append("  basinc:%8.2f hPa  sicaklik:%6.2f C  nem:%5.1f %%\n",
                          dbg.bme_basinc_hpa, dbg.bme_sicaklik, dbg.bme_nem);

            // --- Turetilmis ---
            dbg_append("-- Turetilmis --  dikeyHiz:%7.2f m/s   eglim_acisi:%6.2f deg\n",
                          incomingPacket.dikeyHiz, incomingPacket.eglimAcisi);

            // --- GPS ---
            dbg_append("-- GPS (GY-NEO-7M) --\n");
            dbg_append("  valid:%s  uydu:%lu  hdop:%.2f  lat:%.6f  lng:%.6f  alt:%.1f m  fix_yasi:%lu ms\n",
                          dbg.gps_valid ? "EVET" : "HAYIR", dbg.gps_sat, dbg.gps_hdop,
                          incomingPacket.gpsEnlem, incomingPacket.gpsBoylam, dbg.gps_alt, dbg.gps_age);
            dbg_append("  saat(UTC):%02u:%02u:%02u  TR:%02u:%02u:%02u  tarih:%02u/%02u/%04u  saat_gecerli:%s\n",
                          dbg.gps_saat, dbg.gps_dakika, dbg.gps_saniye,
                          (dbg.gps_saat + 3) % 24, dbg.gps_dakika, dbg.gps_saniye,
                          dbg.gps_gun, dbg.gps_ay, dbg.gps_yil,
                          dbg.gps_time_valid ? "EVET" : "HAYIR");

            // --- Durum makinesi ---
            dbg_append("-- DURUM MAKINESI --\n");
            dbg_append("  durum:%s  apogee kosullari: A(dusus>15m)=%d B(hiz<0)=%d D(eglim<10)=%d\n",
                          durum_adi(incomingPacket.ucus_durumu), dbg.apo_A, dbg.apo_B, dbg.apo_D);
            dbg_append("  ayrilma1:%d ayrilma2:%d | funye1_aktif:%d(kalan %ld ms) funye2_aktif:%d(kalan %ld ms)\n",
                          incomingPacket.ayrilma1_durum, incomingPacket.ayrilma2_durum,
                          funye1_aktif, funye1_aktif ? (long)(FUNYE_SURE_MS - (millis()-funye1_baslangic)) : 0,
                          funye2_aktif, funye2_aktif ? (long)(FUNYE_SURE_MS - (millis()-funye2_baslangic)) : 0);

            // --- TelemetryPacket ozeti ---
            dbg_append("-- PAKET --  sizeof(TelemetryPacket)=%u B  ucus_durumu=%d\n",
                          (unsigned)sizeof(TelemetryPacket), incomingPacket.ucus_durumu);

            // --- LoRa TX durumu ---
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
            dbg_append("-- LoRa TX (STRING/DOKUM) --  ");
            if (lora_dump_bu_dongu) dbg_append("[BU DONGU TAM DOKUM TX] (~her %d ms)\n", LORA_DUMP_MS);
            else dbg_append("[bekliyor %lu/%d ms]\n", millis() - son_lora_dump, LORA_DUMP_MS);
#else
            dbg_append("-- LoRa CERCEVE (FRAMED) --  ");
            if (tx_bu_dongu) dbg_append("[BU DONGU TX EDILDI] ");
            else dbg_append("[bu dongu TX yok, sayac %lu/%d] ", lora_sayac, LORA_GONDERIM_ORANI);
            if (son_frame_len > 0) {
                dbg_append("(%u B, CRC16=0x%04X)\n  ", (unsigned)son_frame_len, son_crc);
                for (size_t i = 0; i < son_frame_len; i++) {
                    dbg_append("%02X ", son_frame[i]);
                    if ((i & 0x0F) == 0x0F) dbg_append("\n  ");
                }
                dbg_append("\n");
            } else {
                dbg_append("(henuz cerceve gonderilmedi)\n");
            }
#endif

            // --- Kuyruk durumu ---
            dbg_append("-- KUYRUK --  bekleyen:%u/%d  bos_yer:%u\n",
                          (unsigned)uxQueueMessagesWaiting(telemetryQueue), TELEMETRY_QUEUE_LEN,
                          (unsigned)uxQueueSpacesAvailable(telemetryQueue));

            // --- SD durumu ---
            dbg_append("-- SD --  sdOk:%d  aktif_buffer:%c  buf_idx:%d/%d\n",
                          sdOk, (active_sd_buf == 0) ? 'A' : 'B', sd_buf_idx, SD_DMA_BUF_SIZE);

            // --- Sistem ---
            dbg_append("-- SISTEM --  free_heap:%u B\n", (unsigned)ESP.getFreeHeap());

            // === TEK SEFERDE BAS ===
            DBGPR(dbg_buf);                               // Serial monitor: metin modda tam hizda (framed modda susar)
#if LORA_GONDERIM_MODU == LORA_MODE_STRING
            if (lora_dump_bu_dongu) {                     // LoRa: ayni dokum, throttle'li
                uart_write_bytes(UART_NUM_1, dbg_buf, dbg_off);
                son_lora_dump = millis();
            }
#endif
        }
    }
  }
}

void setup() {
    // 0. USB SERI MONITOR (DEBUG)
    Serial.begin(115200);
    delay(400);
    DBGLN("\n\n################################################################");
    DBGLN("#  TRAKYA ROKET 2026 - UCUS YAZILIMI  [DEBUG / TESHIS FORK]     #");
    DBGLN("################################################################");
    DBGF("  Derleme: %s %s\n", __DATE__, __TIME__);
    DBGF("  ESP-IDF: %s  |  CPU: %d MHz  |  free_heap: %u B\n",
                  esp_get_idf_version(), getCpuFrequencyMhz(), (unsigned)ESP.getFreeHeap());
    DBGF("  Ayarlar: DBG_PRINT_MS=%d (~%.0f Hz)  FUNYE_GERCEK_ATES=%d (%s)\n",
                  DBG_PRINT_MS, 1000.0/DBG_PRINT_MS, FUNYE_GERCEK_ATES,
                  FUNYE_GERCEK_ATES ? "GERCEK ATESLEME!" : "simule/guvenli");

    pin_haritasi_bas();

    // 2. Pin Modları ve Güvenlik
    // *** ASLA DEGISTIRME: funye pinleri burada OUTPUT YAPILMAZ. ***
    // Acikca high-Z'ye birakilir; gate'i dis pull-down GND'ye ceker. Pin
    // OUTPUT'a yalnizca Funye1Atesle/Funye2Atesle icinde, atesleme aninda
    // gecer ve funye_guncelle() sure dolunca tekrar high-Z'ye birakir.
    // Gerekce icin funye_pin_serbest/funye_pin_ates ustundeki blogu oku.
    funye_pin_serbest(PIN_FUNYE_1);
    funye_pin_serbest(PIN_FUNYE_2);

    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);
    pinMode(PIN_SDKART_DET, INPUT_PULLUP);

    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

    // 3. Protokoller ve DMA Bellek Tahsisi
    sd_dma_buf_A = (char*)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    sd_dma_buf_B = (char*)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    DBGF("  DMA buffer tahsisi: A=%s B=%s\n",
                  sd_dma_buf_A ? "OK" : "BASARISIZ", sd_dma_buf_B ? "OK" : "BASARISIZ");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    // DEBUG: I2C bus taramasi
    i2c_bus_tara();

    // 4. Modül Haberleşmeleri
    Serial2.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    DBGLN("\n[GPS] Serial2 (UART2) baslatildi @9600 - fix icin acik gokyuzu gerekir.");

    // LoRa (UART1) DMA Yapılandırması
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
    DBGLN("\n[LoRa] UART1 IDF surucusu kuruldu. Konfigurasyon gonderiliyor...");
    lora_konfigurasyon();

    delay(500);
    const char* start_msg = "--- ROKET SISTEMI BASLATILIYOR (DEBUG) ---\n";
    uart_write_bytes(UART_NUM_1, start_msg, strlen(start_msg));

    // SD Kart Başlatma
    DBGLN("\n[SD] Baslatiliyor...");
    DBGF("  SD_DET pini (%d) durumu: %s\n", PIN_SDKART_DET,
                  digitalRead(PIN_SDKART_DET) ? "HIGH (kart yok olabilir)" : "LOW (kart var)");
    if (!SD.begin(PIN_SPI_CS)) {
        lora_log("UYARI: SD Kart baslatilamadi! Loglama yapilmayacak.");
        sdOk = false;
    } else {
        DBGF("  SD OK. Boyut: %llu MB, Tip: %d\n", SD.cardSize() / (1024ULL*1024ULL), SD.cardType());
        lora_log("SD Kart baslatildi.");
        logFile = SD.open("/ucus_log.csv", FILE_APPEND);
        if (logFile) {
            if (logFile.size() == 0) {
                logFile.println("ivmeX,ivmeY,ivmeZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,irtifa,hiz,eglim,lat,lng,ayr1,ayr2,state");
            }
            sdOk = true;
        } else {
            lora_log("HATA: Log dosyasi acilamadi!");
            sdOk = false;
        }
    }

    // Sensör Başlatma — BNO055
    DBGLN("\n[BNO055] Baslatiliyor (adres 0x28, mod IMUPLUS 0x08)...");
    if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        lora_log("KRITIK: BNO055 bulunamadi! Sistem durduruluyor.");
        DBGLN("  !! BNO055 begin() BASARISIZ - I2C taramasinda 0x28 gorundu mu? Sistem duruyor.");
        while(true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    lora_log("BNO055 baslatildi (IMU mode 0x08).");

    lora_log("BNO055 kalibrasyonu bekleniyor (max ~15sn)...");
    {
        uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
        unsigned long kal_baslangic = millis();
        const unsigned long KAL_TIMEOUT_MS = 15000;
        while (cal_sys < BNO055_MIN_KALIBRASYON &&
               (millis() - kal_baslangic < KAL_TIMEOUT_MS)) {
            bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);
            char buf[80];
            snprintf(buf, sizeof(buf), "Kal: Sys=%d/3 Gyro=%d/3 Accel=%d/3 Mag=%d/3",
                     cal_sys, cal_gyro, cal_accel, cal_mag);
            lora_log(buf);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        if (cal_sys >= BNO055_MIN_KALIBRASYON)
            lora_log("BNO055 kalibrasyonu tamamlandi!");
        else
            lora_log("UYARI: Kalibrasyon timeout - devam ediliyor (sys<1).");
    }

    // BME280
    DBGLN("\n[BME280] Baslatiliyor (adres 0x76 -> 0x77)...");
    bool bme76 = bme.begin(BME280_ADDR_PRIMARY);
    bool bme77 = false;
    if (!bme76) bme77 = bme.begin(BME280_ADDR_SECONDARY);
    if (!bme76 && !bme77) {
        lora_log("KRITIK: BME280 bulunamadi! Sistem durduruluyor.");
        DBGLN("  !! BME280 hicbir adreste bulunamadi. Sistem duruyor.");
        while(true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    DBGF("  BME280 bulundu @ 0x%02X\n", bme76 ? BME280_ADDR_PRIMARY : BME280_ADDR_SECONDARY);
    lora_log("BME280 baslatildi.");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    // Ground Kalibrasyon
    delay(200);
    lora_log("Yer kalibrasyonu yapiliyor...");
    float basinc_toplam = 0.0;
    for (int i = 0; i < 20; i++) {
        basinc_toplam += bme.readPressure() / 100.0F;
        delay(50);
    }
    referans_basinc = basinc_toplam / 20.0;
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Referans Basinc (hPa): %.2f", referans_basinc);
        lora_log(buf);
    }

    // FreeRTOS Kuyruk
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(TelemetryPacket));
    if(telemetryQueue == NULL){
      lora_log("KRITIK: Kuyruk olusturulamadi! Sistem durduruluyor.");
      while(true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    DBGF("\n[KUYRUK] Olusturuldu: %d eleman x %u B = %u B\n",
                  TELEMETRY_QUEUE_LEN, (unsigned)sizeof(TelemetryPacket),
                  (unsigned)(TELEMETRY_QUEUE_LEN * sizeof(TelemetryPacket)));

    // RTOS Görevleri
    xTaskCreatePinnedToCore(Task1code, "UcusGörevi", TASK_STACK_SIZE, NULL, TASK1_PRIORITY, &Task1, 0);
    delay(100);
    xTaskCreatePinnedToCore(Task2code, "HaberlesmeGörevi", TASK_STACK_SIZE, NULL, TASK2_PRIORITY, &Task2, 1);

    lora_log("Setup Tamam. Gorevler Dagitildi.");
    DBGLN("\n>>> SETUP TAMAM. Periyodik durum dokumu basliyor...\n");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
