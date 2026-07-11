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
#include "led_durum.h"       // LED karar mantigi (enum UcusDurumu burada)

// ============================================================
//  >>> YARISMA ALANI - LORA ADRES & KANAL AYARLARI <<<
//  E32-433T30D parametreleri. Yarisma alaninda SADECE buradan ayarla.
//  ONEMLI: UKB (main) ile Gorev Yuku FARKLI kanalda olmali (RF cakismasi)!
//  Frekans (MHz) = 410 + LORA_CHAN   (CHAN: 0x00..0x1F ; 0x17 = 433 MHz)
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

/*
================================================================================
  NOT: KALMAN İRTİFA AYARI (kf_irtifa = 16.3, 264, 0.1112)
================================================================================
  e_mea=16.3 olculen BME irtifa gurultusune gore secildi (eski: 1.5).
  e_est=264 sadece baslangic, birkac ornekte kendini duzeltir (onemsiz).
  ORAN e_mea/q ≈ 148 (eski ~15) -> COK daha agir yumusatma = irtifada LAG.
  Etki: irtifa filtreli sinyali gercegin ~0.1-0.4 sn gerisinden takip eder.
  APOGEE'ye etkisi (gecikme yonunde!):
    - Tasarimsal 15m esigi zaten ~1.75 sn geciktiriyor (filtreyle ilgisiz).
    - Filtre lag'i bunun UZERINE ~0.1-0.4 sn / ~1-6 m ekler.
    - Toplam: drogue apogee'nin ~16-21 m altinda, tepeden ~1.8-2.2 sn sonra.
  YAPILACAK: SUT (3000m cik-dus senaryosu) ile apogee gecikmesini olc.
  Fazla gelirse q'yu 0.11->0.3 yap (lag ~yariya iner, gurultu biraz artar).
================================================================================
  YAPILACAKLAR LİSTESİ (TODO & MİMARİ PLAN)
================================================================================
  [ CORE 0 (Uçuş Kontrolü & Kritik İşlemler) ]
  - [x] Sensör Verilerinin Okunması (BNO055, BME280, GPS)
  - [x] Uçuş Algoritması (Apogee tespiti, serbest düşüş anlama vb.)
  - [x] Fünye (Ateşleme) Kontrollerinin durum makinesi (State Machine) ile yapılması
  - [x] Kalibrasyon ve Başlangıç İrtifası / Basıncı referans alımı
  - [ ] İletimdeki paket optimize edilecek. Bit kayması ve CRC kontrolü ile güvenlik sağlanacak. 
  - [ ] BNO055 Z-Ekseni Yön Kalibrasyonu (Şu an Z ekseni yukarı varsayılıyor, doğrulanmalı!)
  - [ ] Kalman fine-tuning: ivme, gyro, irtifa, sıcaklık, nem için ayrı ayrı parametreler

  [ CORE 1 (Haberleşme & Çevre Birimleri) ]
   - [x] LoRa üzerinden Non-Blocking Veri Aktarımı
   - [x] Yer İstasyonu Formatına Göre Verinin Metin (String/CSV) Olarak Parse Edilmesi
   - [x] SD Karta Loglama (Kara Kutu): Verilerin uçuş esnasında kaydedilmesi
   - [x] TTL devre dışı bırakıldı — pinler boş duruyor
 
  [ ORTAK / GENEL ]
  - [x] Sensör ve Haberleşme donanımları arası FreeRTOS Queue aktarımı
  - [x] Kalman Filtresi ekle
  - [x] Kurtarma Sistemi Durum (Ayrılma1/Ayrılma2) bayraklarının haberleşme paketine doğru işlenmesi
  - [ ] Buzzer ve LED uyarı sistemlerinin uçuş evrelerine (State) bağlanması

  [ PERFORMANS / OPTİMİZASYON ]
  - [x] DMA'nın daha iyi implemente edilmesi
================================================================================
*/

/*
================================================================================
  SENSÖR VERİ HARİTASI (Hangi veri nereden alınıyor ve değişkendeki adı ne?)
================================================================================
  1. BNO055 (IMU - İvme, Jiroskop, Yönelim)
     - Doğrusal İvme (Yerçekimsiz) -> ivmeX, ivmeY, ivmeZ
     - Jiroskop (Açısal Hız)       -> gyroX, gyroY, gyroZ
     - Euler Açıları (Yönelim)     -> roll, pitch, yaw
  
  2. BME280 (Barometre - Basınç, Sıcaklık, Nem, İrtifa)
     - Basınç (Pascal)             -> basinc
     - Sıcaklık (Santigrat)        -> bmeSicaklik
     - Nem (%)                     -> nem
     - Hesaplanan İrtifa (Metre)   -> irtifa

  3. GY-NEO-7M (GPS - Konum, Yükseklik)
     - Enlem                       -> gpsEnlem
     - Boylam                      -> gpsBoylam
     - GPS İrtifası (Metre)        -> gpsIrtifa
     - Bağlı Uydu Sayısı           -> gpsUydu
     - Konum Geçerliliği           -> gpsGecerli

  4. TELEMETRİ PAKETİ (TelemetryPacket Struct)
     - Yukarıdaki tüm veriler ve roketin kurtarma durumları (Ayrılma1/2), 
       'TelemetryPacket' isimli yapıya (struct) doldurulur.
     - Pragma pack(1) kullanıldığı için veriler bellekte boşluksuz dizilir, 
       bu sayede yer istasyonuna (LoRa) ham (binary) ve en hızlı şekilde iletilir.
================================================================================
*/


// Uyarlanabilir Sabitler
// referans_basinc: setup() içinde BME280'den otomatik ölçülür. Başlangıç değeri standart deniz seviyesidir.
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
#define BAUD_LORA              9600  // UART1  — E32-433T30D LoRa modülü (SX1278 tabanlı)
#define UART_BUFFER_SIZE       1024  // TX buffer (Non-Blocking gönderim için)

// --- ÇERÇEVE PROTOKOLü (Framed Binary) ---
// Format: [0xAA][0x55][LEN:1B][TelemetryPacket:59B][CRC16_HI:1B][CRC16_LO:1B]
// CRC algoritması: CRC16-CCITT (poly=0x1021, init=0xFFFF)
// Not: E32-433T30D kendi RF katmanında CRC/FEC yapıyor.
//      Uygulama katmanı CRC'si UART hattını ve buffer kaymalarını korur.
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55
// FRAME_SIZE: struct sonrasında hesaplanır → sizeof(TelemetryPacket) + 2+1+2 = 64 byte

// --- LORA GÖNDERİM HIZI ---
// E32-433T30D @ 9600 baud, 64 byte/çerçeve → ~15 çerçeve/sn max
// Core 0 queue → 100 Hz; LoRa → her LORA_GONDERIM_ORANI pakettte bir (≈10 Hz)
#define LORA_GONDERIM_ORANI    8

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

// Sensör Sabitleri
#define BNO055_DEF 55
#define BNO055_ADDR 0x28
#define BME280_ADDR_PRIMARY 0x76
#define BME280_ADDR_SECONDARY 0x77

// Uçuş Algoritması Sabitleri
#define APOGEE_IRTIFA_FARKI   15.0  // m     - Max irtifadan bu kadar düşünce apogee sayılır (BME280)
#define AYRILMA2_MESAFE      550.0  // m     - Bu irtifanın altında ana paraşüt açılır
#define MAX_EGLIM             10.0  // derece - Bu açıdan fazla eğimde apogee sayılmaz (güvenlik)
#define MIN_DIKEY_HIZ          0.0  // m/s   - Bu değerin altı (negatif) = düşüyor (BME280)
#define KALKIS_IVME_ESIGI     20.0  // m/s²  - Z ekseninde bu ivmenin üstü = kalkış (BNO055)
#define INIS_HIZ_ESIGI         2.0  // m/s   - Bu değerin altı = yerde sayılır
#define INIS_IRTIFA_ESIGI     20.0  // m     - Bu irtifanın altı = yerde sayılır
#define BNO055_MIN_KALIBRASYON 1    // 0-3 arası - Bu sistem kalibrasyon puanının altında beklenir

// FreeRTOS Sabitleri
#define TASK_STACK_SIZE 10000
#define TASK1_PRIORITY 2
#define TASK2_PRIORITY 1
#define TELEMETRY_QUEUE_LEN 10

// Görev takipçisi (Task Handle) tanımları
TaskHandle_t Task1;
TaskHandle_t Task2;

// Uçuş Durum Makinesi (enum UcusDurumu artik led_durum.h icinde)
UcusDurumu durum = HAZIR;

// setup() tamamlaninca true olur; false iken LED'ler config blink yapar
volatile bool sistem_hazir = false;

// SİT/SUT Mod Durum Makinesi (Task2 tarafindan yonetilir)
enum SitSutModu {
    MOD_BEKLEME   = 0, // Komut bekleniyor, test aktif degil
    MOD_SIT       = 1, // Sensör İzleme Testi aktif
    MOD_SUT       = 2  // Sentetik Uçuş Testi aktif
};
volatile SitSutModu sitSutMod = MOD_BEKLEME;

//Uçuş Algoritması İçin Gerekli Değişkenler
bool ayrilma1 = false;
bool ayrilma2 = false;
float max_irtifa_degeri = 0.0;
unsigned long kalkis_zaman = 0;          // Kalkis aninin millis() degeri (durum biti 1 icin)
volatile uint16_t durum_bitleri = 0;     // SUT durum bilgilendirme bitleri (Tablo 5, latch)

// --- FÜNYE ZAMANLAMA (Non-Blocking) ---
#define FUNYE_SURE_MS 400  // Fünyeye enerji verilecek süre (ms)
unsigned long funye1_baslangic = 0;  // 0 = aktif değil
unsigned long funye2_baslangic = 0;
bool funye1_aktif = false;
bool funye2_aktif = false;

// --- HIZ HESAPLAMA DEĞİŞKENLERİ ---
float onceki_irtifa = 0.0;
unsigned long onceki_zaman = 0;
float anlik_dikey_hiz = 0.0;
float eglim_acisi = 0.0; // Roketin dikeyden sapma açısı (Tilt)

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
// Parametreler: (Ölçüm Hatası, Tahmin Hatası, Süreç Gürültüsü)
SimpleKalmanFilter kf_ivmeX(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_ivmeY(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_ivmeZ(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_gyroX(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_gyroY(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_gyroZ(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_roll(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_pitch(2.906, 9.982, 0.3884);
SimpleKalmanFilter kf_yaw(2.906, 9.982, 0.3884);

// Sadece irtifa filtreleniyor (basinc/sicaklik/nem okunmuyor)
SimpleKalmanFilter kf_irtifa(16.3, 264, 0.1112); // olculen BME irtifa gurultusune gore (e_mea, e_est, q)

// --- SENSÖR NESNELERİ ---
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_DEF, BNO055_ADDR); 
Adafruit_BME280 bme; // I2C üzerinden iletişim
TinyGPSPlus gps;

// --- SENSÖR VERİ DEĞİŞKENLERİ ---
// IMU (BNO055) Verileri
float ivmeX = 0.0, ivmeY = 0.0, ivmeZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
// BNO055'in roketçilikte en büyük avantajı Euler açılarını donanımsal hesaplamasıdır:
float roll = 0.0, pitch = 0.0, yaw = 0.0; 

// Barometre (BME280) Verileri
float irtifa = 0.0; // basinc/bmeSicaklik/nem kullanilmadigi icin kaldirildi
float basinc = 0.0; // Yalniz SİT modunda okunur (Tablo 3 SİT paketi icin)

// GPS Verileri
float gpsEnlem = 0.0, gpsBoylam = 0.0;

// --- TELEMETRİ YAPISI VE KUYRUK ---
// "pragma pack(push, 1)" struct'ın bellekte boşluksuz (padding olmadan) paketlenmesini sağlar,
// bu sayede UART üzerinden ham byte olarak (DMA tarzında) basmak çok daha güvenli ve tutarlı olur.
#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float irtifa;
    float dikeyHiz; // Yeni eklenen dikey hız verisi
    float eglimAcisi; // Yeni eklenen eğim açısı
    float gpsEnlem, gpsBoylam;
    bool ayrilma1_durum;
    bool ayrilma2_durum;
    uint8_t ucus_durumu; // Uçuş evresi: 0=Hazır, 1=Yükseliyor, 2=İniş1, 3=İniş2, 4=İndi
};
#pragma pack(pop)

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

// DMA uyumlu telemetri paketi pointer'ı
TelemetryPacket* dma_packet; 

// SD Kart Ping-Pong Buffer Tanımları
#define SD_DMA_BUF_SIZE 512
char* sd_dma_buf_A;
char* sd_dma_buf_B;
volatile int active_sd_buf = 0; // 0: A, 1: B
volatile int sd_buf_idx = 0;

QueueHandle_t telemetryQueue;
File logFile;
bool sdOk = false;

// Hazır Kullanılacak Metodlar/Fonksiyonlar

// Non-Blocking fünye ateşleme: Pini HIGH yapar, zamanı kaydeder.
// 400ms sonra funye_guncelle() otomatik kapatır.
void Funye1Atesle(){
    if (!funye1_aktif) {
        // SUT tezgah testinde (bayrak 1) gercek fünyeyi ATESLEME — sadece LED.
        if (!(SUT_FUNYE_YERINE_LED && sitSutMod == MOD_SUT)) {
            digitalWrite(PIN_FUNYE_1, HIGH);
        }
        funye1_baslangic = millis();
        funye1_aktif = true;
        ayrilma1 = true;
    }
}

void Funye2Atesle(){
    if (!funye2_aktif) {
        if (!(SUT_FUNYE_YERINE_LED && sitSutMod == MOD_SUT)) {
            digitalWrite(PIN_FUNYE_2, HIGH);
        }
        funye2_baslangic = millis();
        funye2_aktif = true;
        ayrilma2 = true;
    }
}

// Her döngüde çağrılmalı — süresi dolan fünyeyi kapatır
void funye_guncelle() {
    if (funye1_aktif && (millis() - funye1_baslangic >= FUNYE_SURE_MS)) {
        digitalWrite(PIN_FUNYE_1, LOW);
        funye1_aktif = false;
    }
    if (funye2_aktif && (millis() - funye2_baslangic >= FUNYE_SURE_MS)) {
        digitalWrite(PIN_FUNYE_2, LOW);
        funye2_aktif = false;
    }
}

// --- LED GOSTERGE SURUCUSU (tek merkez) ---
// Karar mantigi led_durum.h'deki saf fonksiyonda; burada sadece pinlere yazilir.
void led_uygula() {
    bool normal_mod = (sitSutMod == MOD_BEKLEME);
    LedDurum d = hesapla_led_durumu(normal_mod, sistem_hazir, durum, millis());
    digitalWrite(PIN_LED_1, d.led1 ? HIGH : LOW);
    digitalWrite(PIN_LED_2, d.led2 ? HIGH : LOW);
    digitalWrite(PIN_LED_3, d.led3 ? HIGH : LOW);
}

// Görseldeki formüle göre Anlık Dikey Hız (Vz) Hesaplama Fonksiyonu
float hesapla_dikey_hiz(float guncel_irtifa) {
    unsigned long suanki_zaman = micros();
    
    // İlk ölçüm kontrolü
    if (onceki_zaman == 0) {
        onceki_zaman = suanki_zaman;
        onceki_irtifa = guncel_irtifa;
        return 0.0;
    }
    
    // Delta t hesaplaması (mikrosaniyeden saniyeye dönüşüm)
    float delta_t = (float)(suanki_zaman - onceki_zaman) / 1000000.0f;
    
    // Bölme hatası (divide-by-zero) koruması
    if (delta_t <= 0.0) {
        return anlik_dikey_hiz; // Geçerli zaman farkı yoksa eski hızı koru
    }
    
    // Delta Altitude (İrtifa farkı)
    float delta_irtifa = guncel_irtifa - onceki_irtifa;
    
    // V_z = Delta Altitude / Delta t
    float hiz_z = delta_irtifa / delta_t;
    
    // Gelecek hesaplama için değerleri güncelle
    onceki_zaman = suanki_zaman;
    onceki_irtifa = guncel_irtifa;
    
    return hiz_z;
}

// --- CRC16-CCITT (poly=0x1021, init=0xFFFF) ---
// E32-433T30D UART hattında bit flip / buffer kayması tespiti için
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

// --- BUFFERLI (PING-PONG) SD YAZMA ---
void bufferla_ve_yaz_sd(File& file, const TelemetryPacket& pkt) {
    char temp_line[160];
    // Paketi CSV satırına dönüştür (basinc, sicaklik, nem kaldirildi)
    int line_len = snprintf(temp_line, sizeof(temp_line), 
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%d,%d,%d\n",
        pkt.ivmeX, pkt.ivmeY, pkt.ivmeZ, pkt.gyroX, pkt.gyroY, pkt.gyroZ,
        pkt.roll, pkt.pitch, pkt.yaw, pkt.irtifa, pkt.dikeyHiz, pkt.eglimAcisi,
        pkt.gpsEnlem, pkt.gpsBoylam, pkt.ayrilma1_durum, pkt.ayrilma2_durum, pkt.ucus_durumu);

    char* current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;

    // Eğer yeni satır mevcut tampona sığmıyorsa, tamponu boşalt (Ping-Pong)
    if (sd_buf_idx + line_len >= SD_DMA_BUF_SIZE) {
        // Mevcut tamponu SD karta toplu (DMA dostu) olarak bas
        if (file) {
            file.write((const uint8_t*)current_buf, sd_buf_idx);
            // file.flush(); // Performans için her seferinde flush yapmıyoruz
        }
        
        // Tampon değiştir (Ping-Pong)
        active_sd_buf = (active_sd_buf == 0) ? 1 : 0;
        current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
        sd_buf_idx = 0;
    }

    // Veriyi aktif tampona kopyala
    memcpy(&current_buf[sd_buf_idx], temp_line, line_len);
    sd_buf_idx += line_len;
}

// --- PING-PONG TAMPONUNU DISKE BOSALT ---
// Tamponda bekleyen (henuz yazilmamis) kalan veriyi SD karta yazar ve flush eder.
// Periyodik (~1 sn) ve inis aninda son verinin guc kesintisinde kaybolmamasi icin cagrilir.
void sd_buffer_bosalt(File& file) {
    if (file && sd_buf_idx > 0) {
        char* current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
        file.write((const uint8_t*)current_buf, sd_buf_idx);
        sd_buf_idx = 0;
    }
    if (file) file.flush(); // Fiziksel olarak karta yazilmasini garanti et
}

// --- ÇERÇEVELI PAKET GÖNDERME (DMA DESTEKLI UART) ---
void gonder_paket_framed_dma(uart_port_t uart_num, const TelemetryPacket& pkt) {
    static uint8_t frame_buf[80]; // DMA uyumlu buffer (statik bellek genelde DMA erişimine uygundur)
    
    const uint8_t* payload = (const uint8_t*)&pkt;
    const size_t   len     = sizeof(TelemetryPacket);
    uint16_t       crc     = crc16_ccitt(payload, len);

    size_t idx = 0;
    frame_buf[idx++] = SYNC_BYTE_1;
    frame_buf[idx++] = SYNC_BYTE_2;
    frame_buf[idx++] = (uint8_t)len;
    memcpy(&frame_buf[idx], payload, len);
    idx += len;
    frame_buf[idx++] = (uint8_t)(crc >> 8);
    frame_buf[idx++] = (uint8_t)(crc & 0xFF);

    // uart_write_bytes: Veriyi dahili halka tampona (ring buffer) atar ve hemen döner.
    // Arka plandaki UART donanımı veriyi asenkron olarak (DMA-like) gönderir.
    uart_write_bytes(uart_num, (const char*)frame_buf, idx);
}

// --- E32-433T30D LORA MODUL KONFIGURASYONU ---
// Modul once config moduna alinir (M0=1, M1=1), parametre paketi UART1'e yazilir,
// ardindan normal (transparan) moda (M0=0, M1=0) geri donulur.
// Config paketi: {0xC0=kalici kaydet, ADDH, ADDL, SPED, CHAN, OPTION}
// Parametreler en bastaki YARISMA ALANI define blogundan gelir.
// NOT: Config modunda E32 her zaman 9600 8N1 haberlesir — UART1 zaten bu ayarda.
void lora_konfigurasyon() {
    static const uint8_t configPacket[6] = {0xC0, LORA_ADDH, LORA_ADDL, LORA_SPED, LORA_CHAN, LORA_OPTION};

    // 1. Config moduna gec: M0=1, M1=1
    digitalWrite(LORA_M0, HIGH);
    digitalWrite(LORA_M1, HIGH);
    vTaskDelay(50 / portTICK_PERIOD_MS); // Mod gecisi icin bekle (AUX pini bagli degil -> sabit gecikme)

    // 2. Hatta kalmis eski RX baytlarini temizle
    uart_flush(UART_NUM_1);

    // 3. Parametre paketini yaz ve gonderimin bitmesini bekle
    uart_write_bytes(UART_NUM_1, (const char*)configPacket, sizeof(configPacket));
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(200));

    // 4. Modulun parametreleri kaydetmesi ve 0xC1... echo yaniti icin bekle
    vTaskDelay(200 / portTICK_PERIOD_MS);
    uart_flush(UART_NUM_1); // Echo yanitini at, hat temiz kalsin

    // 5. Normal (transparan) moda don: M0=0, M1=0
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

// --- SETUP TESHIS MESAJLARINI LORA UZERINDEN GONDER ---
// UART1'i IDF surucusu sahiplendigi icin Serial1 kullanilamaz; dogrudan uart_write_bytes.
// Sadece setup icinde (task'lar baslamadan) cagrilir. ASCII metin 0xAA icermez, bu yuzden
// yer istasyonunun cerceve (0xAA 0x55) senkronunu bozmaz.
void lora_log(const char* msg) {
    uart_write_bytes(UART_NUM_1, msg, strlen(msg));
    uart_write_bytes(UART_NUM_1, "\r\n", 2);
}

// Core 0'da çalışacak olan görevin fonsiyonu
void Task1code(void *pvParameters) {

  static SitSutModu onceki_mod = MOD_BEKLEME;

  for (;;) {
    // 0. MOD GEÇİŞ KONTROLÜ — HER mod gecisinde temiz baslangic.
    //    Ozellikle SUT'tan cikista (DURDUR/SIT): sentetik ucus state'i
    //    gercek funyeyi tetiklemesin (GÜVENLİK).
    SitSutModu simdiki_mod = sitSutMod;
    if (simdiki_mod != onceki_mod) {
        durum             = HAZIR;
        ayrilma1          = false;
        ayrilma2          = false;
        max_irtifa_degeri = 0.0;
        durum_bitleri     = 0;
        kalkis_zaman      = 0;
        onceki_zaman      = 0;
        anlik_dikey_hiz   = 0.0;
        funye1_aktif = false;
        funye2_aktif = false;
        digitalWrite(PIN_FUNYE_1, LOW);
        digitalWrite(PIN_FUNYE_2, LOW);
        onceki_mod = simdiki_mod;
    }

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

    // Roketin yere göre eğim açısını (Tilt Angle) hesapla (0 = Tam dik)
    float p_rad = pitch * DEG_TO_RAD;
    float r_rad = roll * DEG_TO_RAD;
    // [FIX] Kalman gürültüsü cos(p)*cos(r)'yi 1.0'ı aşabilir → acos(NaN) → apogee asla tetiklenmez
    float cos_val = cos(p_rad) * cos(r_rad);
    cos_val = constrain(cos_val, -1.0f, 1.0f);
    eglim_acisi = acos(cos_val) * RAD_TO_DEG;

    // Anlık dikey hızı (Vz) hesapla
    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);

    // --- FÜNYE ZAMANLAMA KONTROLÜ (Non-Blocking) ---
    funye_guncelle();

    // --- LED GOSTERGELERINI GUNCELLE ---
    led_uygula();

    // --- UÇUŞ ALGORİTMASI ---
    switch (durum) {
        case HAZIR:
            // Kalkış tespiti: Z ekseninde yeterli ivme → YUKSELIYOR
            // [ TODO ] BNO055 Z-ekseni yönü doğrulanmalı!
            if (ivmeZ > KALKIS_IVME_ESIGI) {
                durum = YUKSELIYOR;
                kalkis_zaman = millis(); // Motor yanma onlem suresi (durum biti 1) icin referans
            }
            break;

        case YUKSELIYOR:
            // Max irtifa güncelle (sadece yükseliş fazında)
            if (irtifa > max_irtifa_degeri) {
                max_irtifa_degeri = irtifa;
            }

            // ================================================================
            // APOGEE TESPİTİ: 2 BAĞIMSIZ SENSÖR + GÜVENLİK KAPISI
            // ================================================================
            //
            // [SENSÖR 1 - BME280 Barometrik - 2 Koşul]
            //   Kriter A: İrtifa max değerden 15m düştü (yükselme durdu)
            //   Kriter B: Dikey hız negatif (irtifadan türev alınarak hesaplandı)
            //
            // [SENSÖR 2 - BNO055 IMU - 1 Koşul]
            //   Kriter C: Dikey doğrusal ivme 3 m/s² altına düştü.
            //   Neden?→ Motor durmuş + sürtünme bitti = roket serbest düşüşe geçti.
            //   Apogee'de Z-ivmesi sıfıra yaklaşır, sonra hafif negatiç olur.
            //   Bu BME280'den tamamen bağımsız, ikinci bir fiziksel onay.
            //
            // [GÜVENLİK KAPISI - BNO055 - 1 Koşul]
            //   Kriter D: Eğilm açısı < 10° (roket tümbling yapmıyor)
            //   Bu apogee algılaması değil, yanlış pozisyonda ateşlemeyi engeller.
            //
            // TÜM KOŞULLAR sağlanırsa Fünye1 ateşlenir.
            // ================================================================
            if ((max_irtifa_degeri - irtifa > APOGEE_IRTIFA_FARKI) &&  // [BME280] A
                (anlik_dikey_hiz < MIN_DIKEY_HIZ) &&                   // [BME280] B
                (eglim_acisi < MAX_EGLIM)) {                           // [BNO055] D - Güvenlik
                Funye1Atesle(); // Drogue paraşüt → 1. Ayrılma
                durum = INIS_1;
            }
            break;

        case INIS_1:
            // Alçak irtifaya inildiğinde ana paraşütü aç → 2. Ayrılma
            if ((irtifa < AYRILMA2_MESAFE) && (max_irtifa_degeri > AYRILMA2_MESAFE)) {
                Funye2Atesle(); // Ana paraşüt
                durum = INIS_2;
            }
            break;

        case INIS_2:
            // Yere iniş tespiti: hız sıfıra yakın + çok alçakta
            if ((anlik_dikey_hiz > -INIS_HIZ_ESIGI) && (irtifa < INIS_IRTIFA_ESIGI)) {
                durum = INDI;
            }
            break;

        case INDI:
            // Sistem pasif, hiçbir aksiyon alınmaz
            break;
    }

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

    // --- STRUCT DOLDURMA VE CORE 1'E GÖNDERME ---
    TelemetryPacket packet;
    packet.ivmeX = ivmeX; packet.ivmeY = ivmeY; packet.ivmeZ = ivmeZ;
    packet.gyroX = gyroX; packet.gyroY = gyroY; packet.gyroZ = gyroZ;
    packet.roll = roll; packet.pitch = pitch; packet.yaw = yaw;
    packet.irtifa = irtifa;
    packet.dikeyHiz = anlik_dikey_hiz; // Pakete dikey hızı ekle
    packet.eglimAcisi = eglim_acisi; // Pakete eğim açısını ekle
    packet.gpsEnlem = gpsEnlem; packet.gpsBoylam = gpsBoylam;
    packet.ayrilma1_durum = ayrilma1; packet.ayrilma2_durum = ayrilma2;
    packet.ucus_durumu = (uint8_t)durum;

    // Kuyruğa Gönder (Kuyruk doluysa beklemez (0), veriyi atlar. 
    // Sensör okuma hızının bloke olmasını engelleriz.)
    xQueueSend(telemetryQueue, &packet, 0);

    // KESİNLİKLE SİLİNMESİ YASAKTIR: Eğer burası boş döngüde kalırsa ESP32 Watchdog hatası verir ve çöker!
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yaklaşık 100 Hz çalışma frekansı
  }
}

// Core 1'de çalışacak olan görevin fonsiyonu
// ─────────────────────────────────────────────────────────────────────────────
// GÖREVİ: Core 0'dan gelen TelemetryPacket'i çerçeveleyip LoRa ve SD'ye gönderir.
//
// LoRa (UART1 / Serial1) → E32-433T30D modülü     @   9600 baud  → Her 10. paket (~10 Hz)
// SD Kart                 → Kara kutu loglama      @ CSV format   → HER paket (~100 Hz)
//
// ÇERÇEVE FORMATI (64 byte/paket):
//   [0xAA][0x55][LEN=59][...TelemetryPacket 59B...][CRC16_HI][CRC16_LO]
//
// CRC16-CCITT: UART hattında bit flip / buffer kayması tespiti.
// E32-433T30D zaten RF katmanında CRC/FEC yapıyor; bu uygulama katmanı CRC'si.
// ─────────────────────────────────────────────────────────────────────────────
void Task2code(void *pvParameters) {

  TelemetryPacket incomingPacket;
  uint32_t lora_sayac = 0; // LoRa hız sınırlayıcı sayacı

  // TTL komut/veri okuma buffer'i (UART0)
  uint8_t ttl_buf[SITSUT_DATA_BOYUT]; // Max 36 byte
  uint8_t ttl_idx = 0;
  uint8_t beklenen_boyut = SITSUT_PAKET_BOYUT;
  unsigned long son_ttl_byte_zamani = 0;

  // 1 sn gecikmeli aktivasyon
  SitSutModu    bekleyen_mod        = MOD_BEKLEME;
  bool          aktivasyon_bekliyor = false;
  unsigned long aktivasyon_zamani   = 0;

  for (;;) {
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
                          if (durum < YUKSELIYOR) {           // ucus basladiysa test komutunu yoksay (GÜVENLİK)
                              bekleyen_mod        = MOD_SIT;
                              aktivasyon_bekliyor = true;
                              aktivasyon_zamani   = millis() + TEST_AKTIVASYON_GECIKME_MS;
                          }
                          break;
                      case CMD_SUT_BASLAT:
                          if (durum < YUKSELIYOR) {
                              bekleyen_mod        = MOD_SUT;
                              aktivasyon_bekliyor = true;
                              aktivasyon_zamani   = millis() + TEST_AKTIVASYON_GECIKME_MS;
                          }
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
                      // SUT VERİSİ yalnizca SUT modunda globallere enjekte edilir (GÜVENLİK)
                      if (sitSutMod == MOD_SUT) {
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

    // Queue'dan paket al (CPU'yu yormadan veri gelene kadar bekler)
    if (xQueueReceive(telemetryQueue, &incomingPacket, pdMS_TO_TICKS(10)) == pdTRUE) {

        // 1. SD Kart — Ping-Pong Buffer ile Logla (~100 Hz)
        if (sdOk && logFile) {
            bufferla_ve_yaz_sd(logFile, incomingPacket);

            // Periyodik: her ~1 sn'de kalan tamponu fiziksel olarak diske bas
            // (guc kesintisinde en fazla ~1 sn'lik veri kaybi ile sinirlanir)
            static int flush_sayac = 0;
            if (++flush_sayac >= 100) {
                sd_buffer_bosalt(logFile);
                flush_sayac = 0;
            }

            // Inis tamamlandiginda: son veriyi hemen ve kesin olarak diske yaz
            if (incomingPacket.ucus_durumu == INDI) {
                sd_buffer_bosalt(logFile);
            }
        }

        // 2. LoRa (E32-433T30D) — Asenkron DMA Gönderimi (~10 Hz)
        lora_sayac++;
        if (lora_sayac >= LORA_GONDERIM_ORANI) {
            gonder_paket_framed_dma(UART_NUM_1, incomingPacket);
            lora_sayac = 0;
        }
    }
    // pdMS_TO_TICKS(10) ile en fazla 10ms bekleriz; bu sure hem TTL'i (Task2 basinda okunuyor)
    // sik sik yoklamamizi saglar hem de Watchdog'u tetiklemeyecek kadar kisadir.
  }
}

void setup() {
    // 1. TTL — SİT/SUT komut alma icin Serial (UART0) baslatiliyor
    Serial.begin(BAUD_TTL);

    // 2. Pin Modları ve Güvenlik (Tasklardan ÖNCE yapılmalı)
    pinMode(PIN_FUNYE_1, OUTPUT);
    pinMode(PIN_FUNYE_2, OUTPUT);
    digitalWrite(PIN_FUNYE_1, LOW);
    digitalWrite(PIN_FUNYE_2, LOW);
    
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);

    // Drogue/Ana gosterge LED'lerini basta sondur (PIN_LED_1/PIN_LED_2 zaten OUTPUT)
    digitalWrite(PIN_LED_DROGUE, LOW);
    digitalWrite(PIN_LED_ANA, LOW);

    pinMode(PIN_SDKART_DET, INPUT_PULLUP); // SD kart algılama genelde pull-up gerektirir

    // LoRa mod pinleri — baslangicta normal (transparan) mod: M0=0, M1=0
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

    // 3. Protokoller ve DMA Bellek Tahsisi
    sd_dma_buf_A = (char*)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    sd_dma_buf_B = (char*)heap_caps_malloc(SD_DMA_BUF_SIZE, MALLOC_CAP_DMA);
    
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    // 4. Modül Haberleşmeleri — LoRa'yı sensörlerden ÖNCE başlat
    Serial2.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    
    // LoRa (UART1) DMA Yapılandırması
    uart_config_t uart_config = {
        .baud_rate = BAUD_LORA,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // Sürücüyü kur: 2048 byte RX buffer, 2048 byte TX buffer (DMA asenkronluğu için)
    uart_driver_install(UART_NUM_1, 2048, 2048, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, PIN_LORA_TX, PIN_LORA_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // LoRa (E32) parametrelerini yukle — config modu → paket → normal mod
    lora_konfigurasyon();

    delay(500);
    
    const char* start_msg = "--- ROKET SISTEMI BASLATILIYOR (DMA ACTIVE) ---\n";
    uart_write_bytes(UART_NUM_1, start_msg, strlen(start_msg));

    // SD Kart Başlatma
    if (!SD.begin(PIN_SPI_CS)) {
        lora_log("UYARI: SD Kart baslatilamadi! Loglama yapilmayacak.");
        sdOk = false;
    } else {
        lora_log("SD Kart baslatildi.");
        logFile = SD.open("/ucus_log.csv", FILE_APPEND);
        if (logFile) {
            // Dosya yeni oluşturulduysa veya boşsa başlık yaz
            if (logFile.size() == 0) {
                logFile.println("ivmeX,ivmeY,ivmeZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,irtifa,hiz,eglim,lat,lng,ayr1,ayr2,state");
            }
            sdOk = true;
        } else {
            lora_log("HATA: Log dosyasi acilamadi!");
            sdOk = false;
        }
    }

    // Sensör Başlatma İşlemleri — IMU modu (0x08): accel+gyro fusion (mag yok).
    // Roket icin ideal: mag kalibrasyonu beklemez, motor/metal yaninda saglam;
    // ayrica klon modullerde NDOF'un 0000 vermesi sorununu da onler.
    if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        lora_log("KRITIK: BNO055 bulunamadi! Sistem durduruluyor.");
        while(true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    lora_log("BNO055 baslatildi (IMU mode 0x08).");
    // NOT: setExtCrystalUse KULLANILMAZ — klon modulde harici kristal yok
    // (true yapmak fusion cikisini 0000'a dusurur).

    // BNO055 Kalibrasyon Kalitesi Bekleme (TIMEOUT'lu)
    // begin() başarılı olsa bile kalibrasyon zaman alabilir; ancak rampada sabit
    // beklerken sonsuza kilitlenmemek icin ~15 sn timeout var. Sure dolarsa devam eder.
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
            led_uygula(); // config blink (sistem_hazir=false -> 1 Hz)
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        if (cal_sys >= BNO055_MIN_KALIBRASYON)
            lora_log("BNO055 kalibrasyonu tamamlandi!");
        else
            lora_log("UYARI: Kalibrasyon timeout - devam ediliyor (sys<1).");
    }

    // BME280 genelde 0x76 veya 0x77 I2C adresi kullanır
    if (!bme.begin(BME280_ADDR_PRIMARY) && !bme.begin(BME280_ADDR_SECONDARY)) {
        lora_log("KRITIK: BME280 bulunamadi! Sistem durduruluyor.");
        while(true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }
    lora_log("BME280 baslatildi.");
    // Gelişmiş okuma ayarları (BME280)
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // Sicaklik
                    Adafruit_BME280::SAMPLING_X16, // Basinc
                    Adafruit_BME280::SAMPLING_X1,  // Nem
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    // 3.1 Ground Kalibrasyon: Anlık yer seviyesi basıncını ölç (20 örnek ortalaması)
    delay(200);
    lora_log("Yer kalibrasyonu yapiliyor...");
    float basinc_toplam = 0.0;
    for (int i = 0; i < 20; i++) {
        basinc_toplam += bme.readPressure() / 100.0F; // Pascal → hPa
        delay(50);
        led_uygula(); // config blink (sistem_hazir henüz false)
    }
    referans_basinc = basinc_toplam / 20.0;
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Referans Basinc (hPa): %.2f", referans_basinc);
        lora_log(buf);
    }

    // Tum baslangic isleri bitti — LED'ler artik state machine'e gore (HAZIR = solid)
    sistem_hazir = true;

    // FreeRTOS Kuyruk Başlatma (Maksimum 10 paketlik yer ayıralım)
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(TelemetryPacket));
    if(telemetryQueue == NULL){
      lora_log("KRITIK: Kuyruk olusturulamadi! Sistem durduruluyor.");
      while(true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }

    // 5. RTOS Görevleri
    // Core 0: Genelde sensör okuma ve uçuş algoritması (Kritik işler)
    xTaskCreatePinnedToCore(
        Task1code, "UcusGörevi", TASK_STACK_SIZE, NULL, TASK1_PRIORITY, &Task1, 0); 
    delay(100); // Kısa bir nefes payı

    // Core 1: Genelde yer istasyonu haberleşmesi ve SD kart (Yavaş işler)
    xTaskCreatePinnedToCore(
        Task2code, "HaberlesmeGörevi", TASK_STACK_SIZE, NULL, TASK2_PRIORITY, &Task2, 1); 
    
    lora_log("Setup Tamam. Gorevler Dagitildi.");
}
void loop() {
  // FreeRTOS görevleri oluşturduğumuz için loop() içini genellikle boş veya
  // basit işler için kullanır mıyız Aslında loop() fonksiyonu varsayılan olarak
  // Core 1 üzerinde bir FreeRTOS görevi gibi çalışır. Bu nedenle ana işlemleri
  // yukarıdaki Task1code ve Task2code içine yazmalısınız.

  // Sonsuz döngüde arka plan işleri / Watchdog için küçük bir gecikme eklemek
  // iyi bir pratiktir:
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}