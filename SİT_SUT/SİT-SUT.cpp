#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <FS.h>


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
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define PIN_SPI_CLK 18
#define PIN_SPI_MISO 19
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_SPI_MOSI 23
#define PIN_LED_3 25
#define PIN_LED_1 26
#define PIN_FUNYE_1 27
#define PIN_LORA_TX 32
#define PIN_LORA_RX 33
#define PIN_SDKART_DET 35

// Haberleşme Sabitleri
#define BAUD_GPS               9600   // UART2  — GY-NEO-7M GPS modülü
#define BAUD_LORA              9600   // UART1  — E32-433T30D LoRa modülü (SX1278 tabanlı)
#define BAUD_TTL             115200   // UART0  — SİT/SUT komut alma (TTL) — Ek-7 Tablo 7 zorunlu deger
#define UART_BUFFER_SIZE       1024  // TX buffer (Non-Blocking gönderim için)

// --- ÇERÇEVE PROTOKOLü (Framed Binary) ---
// Format: [0xAA][0x55][LEN:1B][TelemetryPacket:71B][CRC16_HI:1B][CRC16_LO:1B]
// CRC algoritması: CRC16-CCITT (poly=0x1021, init=0xFFFF)
// Not: E32-433T30D kendi RF katmanında CRC/FEC yapıyor.
//      Uygulama katmanı CRC'si UART hattını ve buffer kaymalarını korur.
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55
// FRAME_SIZE: struct sonrasında hesaplanır → sizeof(TelemetryPacket) + 2+1+2 = 76 byte

// --- LORA GÖNDERİM HIZI ---
// E32-433T30D @ 9600 baud, 76 byte/çerçeve → ~12.6 çerçeve/sn max
// Core 0 queue → 100 Hz; LoRa → her LORA_GONDERIM_ORANI pakettte bir (≈10 Hz)
#define LORA_GONDERIM_ORANI    10

// --- SİT/SUT KOMUT PROTOKOLÜ ---
// Format: [HEADER:0xAA][COMMAND:1B][CHECKSUM:1B][FOOTER1:0x0D][FOOTER2:0x0A]
// Checksum = checksum'dan ONCEKI tum byte'larin toplami (mod 256) — EK-7 Bolum 3.
//   Komut icin: Header + Command.  Ornek: 0xAA+0x20=0xCA (SIT), 0xAA+0x24=0xCE (DURDUR)
//   NOT: EK-7 Tablo 1'deki 0x8C/0x8E/0x90 degerleri HATALI. Gercek cihaz (logic analyzer
//        ile dogrulandi) Header+Command gonderiyor. O yuzden o tabloya GUVENME.
#define SITSUT_HEADER        0xAA
#define SITSUT_DATA_HEADER   0xAB
#define SITSUT_FOOTER1       0x0D
#define SITSUT_FOOTER2       0x0A
#define SITSUT_PAKET_BOYUT   5
#define SITSUT_DATA_BOYUT    36

#define CMD_SIT_BASLAT       0x20  // Sensör İzleme Testi Başlat
#define CMD_SUT_BASLAT       0x22  // Sentetik Uçuş Testi Başlat
#define CMD_DURDUR           0x24  // Testi Durdur

// --- ORTAK ZAMANLAMA (Ek-7) ---
#define TEST_AKTIVASYON_GECIKME_MS  1000  // Komut onaylandiktan sonra modun aktif olma gecikmesi (Uygulama Plani c)
#define SITSUT_GONDERIM_PERIYOT_MS   100  // SİT telemetri + SUT durum paketi periyodu = 10 Hz (s.7)
#define SITSUT_FRAME_TIMEOUT_MS      100  // Yarim kalan cerceve icin parser sifirlama suresi (Tablo 7)

// --- SUT DURUM BİLGİLENDİRME PAKETİ (Tablo 5 & 6) ---
// Format: [HEADER=0xAA][Data1][Data2][Checksum][0x0D][0x0A] = 6 byte
// Data1 = bit 0-7, Data2 = bit 8-15. Checksum = (Data1 + Data2) & 0xFF
// Bit=1 ilgili asamanin aktif oldugunu gosterir (Tablo 5).
#define SITSUT_DURUM_BOYUT   6
#define ST_BIT_KALKIS        (1u << 0)  // Bit 0: Roket kalkisi algilandi
#define ST_BIT_MOTOR_YANMA   (1u << 1)  // Bit 1: Motor yanma onlem suresi doldu
#define ST_BIT_MIN_IRTIFA    (1u << 2)  // Bit 2: Minimum irtifa esigi asildi
#define ST_BIT_ACI_ESIK      (1u << 3)  // Bit 3: Govde acisi / yatay ivme esigi asildi
#define ST_BIT_ALCALMA       (1u << 4)  // Bit 4: Roket irtifasi alcalmaya basladi
#define ST_BIT_DROGUE_EMIR   (1u << 5)  // Bit 5: Suruklenme parasutu acma emri olusturuldu
#define ST_BIT_ANA_IRTIFA    (1u << 6)  // Bit 6: Roket belirlenen irtifanin altina indi
#define ST_BIT_ANA_EMIR      (1u << 7)  // Bit 7: Ana parasut acma emri olusturuldu

// Durum bitleri esik degerleri
#define DURUM_MIN_IRTIFA_ESIGI  100.0  // m       - Bit 2 esigi
#define DURUM_ACI_ESIGI          45.0  // derece  - Bit 3 esigi (govde egim acisi)
#define MOTOR_YANMA_SURE_MS      3000  // ms       - Bit 1: kalkistan sonra motor yanma onlem suresi

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

// Uçuş Durum Makinesi (State Machine)
enum UcusDurumu {
    HAZIR      = 0, // Rampa üzerinde, kalkış bekleniyor
    YUKSELIYOR = 1, // Kalkış algılandı, yükselme fazı
    INIS_1     = 2, // Apogee geçildi, drogue (küçük) paraşüt açıldı
    INIS_2     = 3, // Alçak irtifaya inildi, ana paraşüt açıldı
    INDI       = 4  // Yere iniş tamamlandı, sistem pasif
};
UcusDurumu durum = HAZIR;

// SİT/SUT Mod Durum Makinesi (Core 1 tarafından yönetilir)
enum SitSutModu {
    MOD_BEKLEME   = 0, // Komut bekleniyor, test aktif değil
    MOD_SIT       = 1, // Sensör İzleme Testi aktif
    MOD_SUT       = 2  // Sentetik Uçuş Testi aktif
};
volatile SitSutModu sitSutMod = MOD_BEKLEME;

//Uçuş Algoritması İçin Gerekli Değişkenler
bool ayrilma1 = false;
bool ayrilma2 = false;
float max_irtifa_degeri = 0.0;
unsigned long kalkis_zaman = 0;          // Kalkis aninin millis() degeri (durum biti 1 icin)

// SUT durum bilgilendirme bitleri (Tablo 5) — Core 0'da guncellenir, latch mantigi
volatile uint16_t durum_bitleri = 0;

// --- SUT TEZGAH TESTI: FÜNYE YERINE LED ---
// 1 = SUT modunda gercek fünye pinini SÜRME, sadece LED yak (tezgah testi — GÜVENLİ)
// 0 = SUT modunda gercek fünyeyi de atesle (Aksaray'daki GERÇEK test cihazi olcumu icin ŞART!)
//   NOT: SİT ve gercek ucusta bu bayraktan bagimsiz olarak fünye normal calisir.
#define SUT_FUNYE_YERINE_LED   1
#define PIN_LED_DROGUE   PIN_LED_1   // GPIO26 — 1. ayrilma (drogue/suruklenme) gostergesi
#define PIN_LED_ANA      PIN_LED_2   // GPIO4  — 2. ayrilma (ana parasut) gostergesi

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
SimpleKalmanFilter kf_ivmeX(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_ivmeY(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_ivmeZ(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_gyroX(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_gyroY(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_gyroZ(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_roll(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_pitch(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_yaw(0.1, 0.1, 0.01);

SimpleKalmanFilter kf_basinc(2.0, 2.0, 0.1);
SimpleKalmanFilter kf_bmeSicaklik(0.5, 0.5, 0.01);
SimpleKalmanFilter kf_irtifa(1.5, 1.5, 0.1);
SimpleKalmanFilter kf_nem(1.0, 1.0, 0.1);

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
float basinc = 0.0, bmeSicaklik = 0.0, irtifa = 0.0, nem = 0.0;

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
    float basinc, bmeSicaklik, irtifa, nem;
    float dikeyHiz; // Yeni eklenen dikey hız verisi
    float eglimAcisi; // Yeni eklenen eğim açısı
    float gpsEnlem, gpsBoylam;
    bool ayrilma1_durum;
    bool ayrilma2_durum;
    uint8_t ucus_durumu; // Uçuş evresi: 0=Hazır, 1=Yükseliyor, 2=İniş1, 3=İniş2, 4=İndi
};
#pragma pack(pop)

// --- SİT TELEMETRİ PAKETİ (Tablo 3 — 36 byte) ---
// Format: [0xAB][İRTİFA:4B][BASINÇ:4B][İVME_X:4B][İVME_Y:4B][İVME_Z:4B]
//         [AÇI_X:4B][AÇI_Y:4B][AÇI_Z:4B][CHK:1B][0x0D][0x0A]
#pragma pack(push, 1)
struct SitPaketi {
    uint8_t header;     // Byte 1  — 0xAB
    float   irtifa;     // Byte 2-5
    float   basinc;     // Byte 6-9
    float   ivmeX;      // Byte 10-13
    float   ivmeY;      // Byte 14-17
    float   ivmeZ;      // Byte 18-21
    float   aciX;       // Byte 22-25 (Roll)
    float   aciY;       // Byte 26-29 (Pitch)
    float   aciZ;       // Byte 30-33 (Yaw)
    uint8_t checksum;   // Byte 34
    uint8_t footer1;    // Byte 35 — 0x0D
    uint8_t footer2;    // Byte 36 — 0x0A
};
#pragma pack(pop)
// sizeof(SitPaketi) = 36 byte

QueueHandle_t telemetryQueue;
File logFile;
bool sdOk = false;
bool bnoOk = false;   // BNO055 bulundu/hazir mi? (yoksa Task1 IMU okumasini atlar)
bool bmeOk = false;   // BME280 bulundu/hazir mi? (yoksa Task1 barometre okumasini atlar)

// Hazır Kullanılacak Metodlar/Fonksiyonlar

// Non-Blocking fünye ateşleme: Pini HIGH yapar, zamanı kaydeder.
// 400ms sonra funye_guncelle() otomatik kapatır.
void Funye1Atesle(){
    if (!funye1_aktif) {
        // Gorsel gosterge: drogue LED'i her modda yakilir (latch — Durdur'a kadar yanik kalir)
        digitalWrite(PIN_LED_DROGUE, HIGH);
        // SUT tezgah testinde (bayrak 1 iken) gercek fünyeyi ATESLEME — sadece LED.
        // SİT, gercek ucus ve bayrak 0 iken gercek fünye pini surulur.
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
        // Gorsel gosterge: ana parasut LED'i her modda yakilir (latch)
        digitalWrite(PIN_LED_ANA, HIGH);
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

// --- CSV FORMATINDA (METİN) GÖNDERME ---
// SD karttan okunabilir format: veri1,veri2,veri3...
void gonder_paket_csv(Print& port, const TelemetryPacket& pkt) {
    port.print(pkt.ivmeX); port.print(",");
    port.print(pkt.ivmeY); port.print(",");
    port.print(pkt.ivmeZ); port.print(",");
    port.print(pkt.gyroX); port.print(",");
    port.print(pkt.gyroY); port.print(",");
    port.print(pkt.gyroZ); port.print(",");
    port.print(pkt.roll); port.print(",");
    port.print(pkt.pitch); port.print(",");
    port.print(pkt.yaw); port.print(",");
    port.print(pkt.basinc); port.print(",");
    port.print(pkt.bmeSicaklik); port.print(",");
    port.print(pkt.irtifa); port.print(",");
    port.print(pkt.nem); port.print(",");
    port.print(pkt.dikeyHiz); port.print(",");
    port.print(pkt.eglimAcisi); port.print(",");
    port.print(pkt.gpsEnlem, 6); port.print(","); 
    port.print(pkt.gpsBoylam, 6); port.print(",");
    port.print(pkt.ayrilma1_durum); port.print(",");
    port.print(pkt.ayrilma2_durum); port.print(",");
    port.println(pkt.ucus_durumu); // Son veri ve alt satıra geç
}

// --- ÇERÇEVELI PAKET GÖNDERME (BINARY) ---
// Format: [SYNC1=0xAA][SYNC2=0x55][LEN=71][...71 byte payload...][CRC16_HI][CRC16_LO]
void gonder_paket_framed(HardwareSerial& port, const TelemetryPacket& pkt) {
    const uint8_t* payload = (const uint8_t*)&pkt;
    const size_t   len     = sizeof(TelemetryPacket); // 71
    uint16_t       crc     = crc16_ccitt(payload, len);

    port.write(SYNC_BYTE_1);       // 0xAA
    port.write(SYNC_BYTE_2);       // 0x55
    port.write((uint8_t)len);      // 71
    port.write(payload, len);      // TelemetryPacket (ham binary)
    port.write((uint8_t)(crc >> 8));   // CRC16 high byte
    port.write((uint8_t)(crc & 0xFF)); // CRC16 low byte
    // Toplam: 76 byte/çerçeve
}

// FLOAT değerini virgülden sonra 2 basamağa yuvarlar (Ek-7 s.7:
// "Test cihazı yalnızca virgülden sonra 2 basamağa kadar olan veriyi kabul edecektir")
static inline float yuvarla2(float v) {
    return roundf(v * 100.0f) / 100.0f;
}

// --- SİT PAKETİ GÖNDERME (TTL / UART0 / Serial) ---
// Tablo 3'e göre 36 byte binary paket — Core 0'dan çağrılır
// Checksum: Payload byte'larının toplamı (header ve footer hariç), low byte
void gonder_sit_paketi() {
    SitPaketi pkt;
    pkt.header  = 0xAB;
    pkt.irtifa  = yuvarla2(irtifa);
    // Ek-7 Tablo 2: basinc birimi mBar (=hPa). bme.readPressure() Pascal doner,
    // bu yuzden /100 ile hPa'ya cevriliyor (Pa ~100727 -> hPa ~1007.27).
    pkt.basinc  = yuvarla2(basinc / 100.0f);
    pkt.ivmeX   = yuvarla2(ivmeX);
    pkt.ivmeY   = yuvarla2(ivmeY);
    pkt.ivmeZ   = yuvarla2(ivmeZ);
    pkt.aciX    = yuvarla2(roll);   // AÇI X = Roll
    pkt.aciY    = yuvarla2(pitch);  // AÇI Y = Pitch
    pkt.aciZ    = yuvarla2(yaw);    // AÇI Z = Yaw

    // Checksum = checksum'dan ONCEKI tum byte'larin toplami (mod 256):
    //   Header (0xAB) + 32 byte payload.  Komut checksum'i ile ayni kural (Header dahil).
    uint8_t chk = pkt.header;   // 0xAB
    const uint8_t* payload = (const uint8_t*)&pkt.irtifa;
    for (size_t i = 0; i < 32; i++) {
        chk += payload[i];
    }
    pkt.checksum = chk;
    pkt.footer1  = 0x0D;
    pkt.footer2  = 0x0A;

    Serial.write((const uint8_t*)&pkt, sizeof(SitPaketi));
}

// --- SUT DURUM BİLGİLENDİRME PAKETİ GÖNDERME (TTL / UART0 / Serial) ---
// Tablo 6'ya göre 6 byte: [0xAA][Data1][Data2][CHK][0x0D][0x0A]
// Data1 = durum_bitleri bit 0-7, Data2 = bit 8-15. CHK = (Data1+Data2)&0xFF.
// Yalnızca SUT modunda 10 Hz gönderilir (Core 0'dan çağrılır).
void gonder_durum_paketi() {
    uint8_t data1 = (uint8_t)(durum_bitleri & 0xFF);
    uint8_t data2 = (uint8_t)((durum_bitleri >> 8) & 0xFF);
    // Checksum = onceki byte'larin toplami: Header (0xAA) + Data1 + Data2 (mod 256)
    uint8_t chk   = (uint8_t)(SITSUT_HEADER + data1 + data2);
    uint8_t paket[SITSUT_DURUM_BOYUT] = {
        SITSUT_HEADER, data1, data2, chk, SITSUT_FOOTER1, SITSUT_FOOTER2
    };
    Serial.write(paket, SITSUT_DURUM_BOYUT);
}

// Core 0'da çalışacak olan görevin fonsiyonu
void Task1code(void *pvParameters) {
 

  static SitSutModu onceki_mod = MOD_BEKLEME;

  for (;;) {
    // ----------------------------------------------------
    // KENDI KODUNUZU BURAYA YAZIN (CORE 0 - SÜREKLİ DÖNGÜ)
    // ----------------------------------------------------

    // 0. MOD GEÇİŞ KONTROLÜ — Yeni bir SUT başladığında uçuş algoritmasını
    //    temiz bir başlangıç için sıfırla (Uygulama Planı: yeni test için hazır olmalı)
    SitSutModu simdiki_mod = sitSutMod;
    if (simdiki_mod != onceki_mod) {
        if (simdiki_mod == MOD_SUT) {
            durum             = HAZIR;
            ayrilma1          = false;
            ayrilma2          = false;
            max_irtifa_degeri = 0.0;
            durum_bitleri     = 0;
            kalkis_zaman      = 0;
            onceki_zaman      = 0;      // dikey hız hesaplayıcı yeniden başlasın
            anlik_dikey_hiz   = 0.0;
            // Fünye/LED durumunu temizle — tekrarli SUT icin temiz baslangic
            funye1_aktif = false;
            funye2_aktif = false;
            digitalWrite(PIN_FUNYE_1, LOW);
            digitalWrite(PIN_FUNYE_2, LOW);
            digitalWrite(PIN_LED_DROGUE, LOW);
            digitalWrite(PIN_LED_ANA, LOW);
        }
        onceki_mod = simdiki_mod;
    }

    // 1. Sensör Verilerini Okuma (SUT modunda değilsek donanımdan oku)
    if (sitSutMod != MOD_SUT) {
        // IMU (BNO055) — yalnizca sensor bulunduysa oku (yoksa deger 0 kalir)
        if (bnoOk) {
            sensors_event_t a, g, o;
            // İvme (Linear Acceleration - Yerçekimi hariç)
            bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
            ivmeX = kf_ivmeX.updateEstimate(a.acceleration.x);
            ivmeY = kf_ivmeY.updateEstimate(a.acceleration.y);
            ivmeZ = kf_ivmeZ.updateEstimate(a.acceleration.z);

            // Jiroskop
            bno.getEvent(&g, Adafruit_BNO055::VECTOR_GYROSCOPE);
            gyroX = kf_gyroX.updateEstimate(g.gyro.x);
            gyroY = kf_gyroY.updateEstimate(g.gyro.y);
            gyroZ = kf_gyroZ.updateEstimate(g.gyro.z);

            // Euler Açıları (Yönelim)
            bno.getEvent(&o, Adafruit_BNO055::VECTOR_EULER);
            yaw = kf_yaw.updateEstimate(o.orientation.x);
            roll = kf_roll.updateEstimate(o.orientation.y);
            pitch = kf_pitch.updateEstimate(o.orientation.z);
        }

        // 2. Barometre (BME280) Verilerini Okuma — yalnizca sensor bulunduysa
        if (bmeOk) {
            bmeSicaklik = kf_bmeSicaklik.updateEstimate(bme.readTemperature());
            basinc = kf_basinc.updateEstimate(bme.readPressure());
            nem = kf_nem.updateEstimate(bme.readHumidity());
            irtifa = kf_irtifa.updateEstimate(bme.readAltitude(referans_basinc));
        }

        // 3. GPS Verilerini Okuma
        while (Serial2.available() > 0) {
            gps.encode(Serial2.read());
        }
        if (gps.location.isUpdated()) {
            gpsEnlem = gps.location.lat();
            gpsBoylam = gps.location.lng();
        }
    } else {
        // MOD_SUT AKTİF: Donanım okuması atlandı. 
        // Değişkenler Core 1 tarafından TTL üzerinden güncelleniyor.
    }

    // Roketin yere göre eğim açısını (Tilt Angle) hesapla
    float p_rad = pitch * DEG_TO_RAD;
    float r_rad = roll * DEG_TO_RAD;
    float cos_val = cos(p_rad) * cos(r_rad);
    cos_val = constrain(cos_val, -1.0f, 1.0f);
    eglim_acisi = acos(cos_val) * RAD_TO_DEG;

    // Anlık dikey hızı (Vz) hesapla
    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);


    // --- FÜNYE ZAMANLAMA KONTROLÜ (Non-Blocking) ---
    funye_guncelle();

    // --- UÇUŞ ALGORİTMASI ---
    switch (durum) {
        case HAZIR:
            // Kalkış tespiti: Z ekseninde yeterli ivme → YUKSELIYOR
            // [ TODO ] BNO055 Z-ekseni yönü doğrulanmalı!
            if (ivmeZ > KALKIS_IVME_ESIGI) {
                durum = YUKSELIYOR;
                kalkis_zaman = millis(); // Motor yanma önlem süresi (durum biti 1) için referans
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

    // --- SUT DURUM BİTLERİNİ GÜNCELLE (Tablo 5, latch mantığı) ---
    // Uçuş algoritması hangi aşamada ise ilgili bit '1' yapılır (bir kez set edilince kalır).
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
    //   MOD_SIT → Tablo 3 telemetri paketi (36 byte)
    //   MOD_SUT → Tablo 6 durum bilgilendirme paketi (6 byte)
    static unsigned long son_ttl_gonderim = 0;
    static uint8_t lora_debug_sayac = 0;   // TTL gonderimlerini ~1 Hz LoRa'ya aynalamak icin
    if (millis() - son_ttl_gonderim >= SITSUT_GONDERIM_PERIYOT_MS) {
        son_ttl_gonderim = millis();
        // DEBUG AYNA: her 10. gonderimde (~1 Hz) LoRa'ya ozet bas — 9600 baud'u tikamamak icin
        bool lora_yaz = (++lora_debug_sayac >= 10);
        if (lora_yaz) lora_debug_sayac = 0;
        if (sitSutMod == MOD_SIT) {
            gonder_sit_paketi();     // 36 byte, Serial (UART0)
            if (lora_yaz) {
                Serial1.print("[TTL-TX SIT] irt="); Serial1.print(irtifa, 2);
                Serial1.print(" bas=");             Serial1.print(basinc / 100.0f, 2);
                Serial1.print(" ivZ=");             Serial1.print(ivmeZ, 2);
                Serial1.print(" aciZ=");            Serial1.println(yaw, 2);
            }
        } else if (sitSutMod == MOD_SUT) {
            gonder_durum_paketi();   // 6 byte durum paketi, Serial (UART0)
            if (lora_yaz) {
                Serial1.print("[TTL-TX DURUM] 0x");
                Serial1.print(durum_bitleri, HEX);
                Serial1.print(" irt=");  Serial1.print(irtifa, 2);
                Serial1.print(" durum="); Serial1.println((int)durum);
            }
        }
    }

    // --- STRUCT DOLDURMA VE CORE 1'E GÖNDERME ---
    TelemetryPacket packet;
    packet.ivmeX = ivmeX; packet.ivmeY = ivmeY; packet.ivmeZ = ivmeZ;
    packet.gyroX = gyroX; packet.gyroY = gyroY; packet.gyroZ = gyroZ;
    packet.roll = roll; packet.pitch = pitch; packet.yaw = yaw;
    packet.basinc = basinc; packet.bmeSicaklik = bmeSicaklik; 
    packet.irtifa = irtifa; packet.nem = nem;
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
// GÖREVİ:
//   1. TTL (UART0/Serial) üzerinden SİT/SUT komutlarını okur ve parse eder
//   2. Komuta göre mod durum makinesini günceller (MOD_BEKLEME / MOD_SIT / MOD_SUT)
//   3. Core 0'dan gelen TelemetryPacket'i çerçeveleyip LoRa ve SD'ye gönderir
//
// TTL (UART0 / Serial)  → SİT/SUT komut+veri (RX) / SİT+durum paketi (TX)  @ 115200 baud (Ek-7 Tablo 7)
// LoRa (UART1 / Serial1) → E32-433T30D modülü      @  9600 baud  → Her 10. paket (~10 Hz)
// SD Kart                 → Kara kutu loglama       @ CSV format  → HER paket (~100 Hz)
//
// SİT/SUT KOMUT FORMATI (5 byte):
//   [HEADER=0xAA][COMMAND][CHECKSUM][0x0D][0x0A]
// ─────────────────────────────────────────────────────────────────────────────
void Task2code(void *pvParameters) {

  TelemetryPacket incomingPacket;
  uint32_t lora_sayac = 0;

  // TTL komut/veri okuma buffer'ı
  uint8_t ttl_buf[SITSUT_DATA_BOYUT]; // Max 36 byte
  uint8_t ttl_idx = 0;
  uint8_t beklenen_boyut = SITSUT_PAKET_BOYUT;
  unsigned long son_ttl_byte_zamani = 0;   // Frame timeout için son alınan byte zamanı

  // 1 saniye gecikmeli aktivasyon (Uygulama Planı c) — komut onaylanır, mod 1 sn sonra aktif olur
  SitSutModu bekleyen_mod        = MOD_BEKLEME;
  bool          aktivasyon_bekliyor = false;
  unsigned long aktivasyon_zamani    = 0;

  for (;;) {

    // ─── 1. TTL OKUMA (Komutlar ve SUT Verisi) ────────────────────────
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
          // PAKET TİPİ: KOMUT (5 Byte)
          if (beklenen_boyut == SITSUT_PAKET_BOYUT) {
              uint8_t cmd = ttl_buf[1];
              uint8_t chk = ttl_buf[2];

              // --- DEBUG AYNA: gelen komutu LoRa'ya bas (TTL/RS232 hattini gormek icin) ---
              // RS232 sorunluyken firmware'in komutu gercekten alip almadigini LoRa'dan izlersin.
              Serial1.print("[TTL-RX komut] ");
              for (int i = 0; i < 5; i++) {
                  if (ttl_buf[i] < 0x10) Serial1.print('0');
                  Serial1.print(ttl_buf[i], HEX);
                  Serial1.print(' ');
              }
              bool footer_ok = (ttl_buf[3] == SITSUT_FOOTER1 && ttl_buf[4] == SITSUT_FOOTER2);
              // Checksum = Header + Command (onceki byte'larin toplami) — logic analyzer ile dogrulandi
              bool chk_ok    = (chk == (uint8_t)(SITSUT_HEADER + cmd));

              if (footer_ok && chk_ok) {
                  Serial1.println("-> GECERLI");
                  switch (cmd) {
                      // Başlat komutları: 1 sn sonra aktif ol (Uygulama Planı c)
                      case CMD_SIT_BASLAT:
                          bekleyen_mod        = MOD_SIT;
                          aktivasyon_bekliyor = true;
                          aktivasyon_zamani   = millis() + TEST_AKTIVASYON_GECIKME_MS;
                          Serial1.println("[SIT] onaylandi, 1sn sonra aktif");
                          break;
                      case CMD_SUT_BASLAT:
                          bekleyen_mod        = MOD_SUT;
                          aktivasyon_bekliyor = true;
                          aktivasyon_zamani   = millis() + TEST_AKTIVASYON_GECIKME_MS;
                          Serial1.println("[SUT] onaylandi, 1sn sonra aktif");
                          break;
                      // Durdur komutu: anında etkili, bekleyen aktivasyonu da iptal et
                      case CMD_DURDUR:
                          aktivasyon_bekliyor = false;
                          sitSutMod           = MOD_BEKLEME;
                          Serial1.println("[STOP] DURDURULDU");
                          break;
                      default:
                          Serial1.println("[?] Bilinmeyen komut");
                          break;
                  }
              } else {
                  // Komut geldi ama gecersiz — nedenini LoRa'ya yaz
                  Serial1.print("-> RED (");
                  if (!footer_ok) Serial1.print("footer ");
                  if (!chk_ok)    Serial1.print("checksum ");
                  Serial1.println(")");
              }
          }
          // PAKET TİPİ: SUT VERİSİ (36 Byte)
          else if (beklenen_boyut == SITSUT_DATA_BOYUT) {
              if (ttl_buf[34] == SITSUT_FOOTER1 && ttl_buf[35] == SITSUT_FOOTER2) {
                  // Checksum doğrula — TOLERANSLI:
                  // Ek-7 Bölüm 3 checksum'ın hangi byte'ları kapsadığını netleştirmiyor.
                  // Bu yüzden iki yaygın yorumu da kabul ediyoruz:
                  //   (a) chk_payload  = yalnızca 32 byte payload toplamı (header hariç)
                  //   (b) chk_header   = 0xAB header dahil toplam
                  // Resmi test cihazı hangisini kullanırsa kullansın paket geçerli sayılır.
                  uint8_t chk_payload = 0;
                  for (int i = 1; i <= 32; i++) chk_payload += ttl_buf[i];
                  uint8_t chk_header = chk_payload + ttl_buf[0]; // + 0xAB

                  if (ttl_buf[33] == chk_payload || ttl_buf[33] == chk_header) {
                      // Verileri global değişkenlere dağıt
                      SitPaketi* sut_data = (SitPaketi*)ttl_buf;
                      irtifa = sut_data->irtifa;
                      basinc = sut_data->basinc;
                      ivmeX  = sut_data->ivmeX;
                      ivmeY  = sut_data->ivmeY;
                      ivmeZ  = sut_data->ivmeZ;
                      roll   = sut_data->aciX;
                      pitch  = sut_data->aciY;
                      yaw    = sut_data->aciZ;
                  }
              }
          }
          ttl_idx = 0;
        }
      }
    }

    // ─── 1b. FRAME TIMEOUT (100 ms) — yarım kalan çerçeveyi sıfırla (Tablo 7) ──
    if (ttl_idx > 0 && (millis() - son_ttl_byte_zamani) > SITSUT_FRAME_TIMEOUT_MS) {
        ttl_idx = 0;
    }

    // ─── 1c. GECİKMELİ AKTİVASYON — komut onayından 1 sn sonra modu etkinleştir ──
    if (aktivasyon_bekliyor && (long)(millis() - aktivasyon_zamani) >= 0) {
        sitSutMod           = bekleyen_mod;
        aktivasyon_bekliyor = false;
    }

    // ─── 2. MOD İŞLEMLERİ ─────────────────────────────────────────────
    switch (sitSutMod) {
      case MOD_SIT:
        // SİT işlemleri Core 0'da (Task1) yürütülüyor.
        break;

      case MOD_SUT:
        // SUT işlemleri (veri dağıtımı) yukarıdaki parser içinde yapıldı.
        break;

      case MOD_BEKLEME:
      default:
        break;
    }

    // ─── 3. TELEMETRİ AKTARIMI (LoRa + SD) — Her modda aktif ──────────
    // Not: portMAX_DELAY yerine 10ms timeout → TTL sürekli okunabilsin
    if (xQueueReceive(telemetryQueue, &incomingPacket, pdMS_TO_TICKS(10)) == pdTRUE) {

        // 3a. SD Kart — Veriyi karta logla (~100 Hz)
        if (sdOk && logFile) {
            gonder_paket_csv(logFile, incomingPacket);

            static int flush_sayac = 0;
            if (++flush_sayac >= 50) {
                logFile.flush();
                flush_sayac = 0;
            }
        }

        // 3b. LoRa — Her LORA_GONDERIM_ORANI pakette bir gönder (~10 Hz)
        lora_sayac++;
        if (lora_sayac >= LORA_GONDERIM_ORANI) {
            gonder_paket_framed(Serial1, incomingPacket);
            lora_sayac = 0;
        }
    }
    // 10ms timeout sayesinde TTL komutları sürekli kontrol edilir,
    // aynı zamanda Queue boşken CPU Watchdog tetiklenmez.
  }
}

void setup() {
    // 1. TTL — SİT/SUT komut alma için Serial (UART0) başlatılıyor
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
    // Drogue/Ana gosterge LED'lerini basta sondur
    digitalWrite(PIN_LED_DROGUE, LOW);
    digitalWrite(PIN_LED_ANA, LOW);
    pinMode(PIN_SDKART_DET, INPUT_PULLUP); // SD kart algılama genelde pull-up gerektirir

    // 3. Protokoller
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    // 4. Modül Haberleşmeleri — LoRa'yı sensörlerden ÖNCE başlat (setup mesajları için)
    Serial2.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial1.setTxBufferSize(UART_BUFFER_SIZE);
    Serial1.begin(BAUD_LORA, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);
    delay(500); // LoRa modülünün hazır olması için kısa bekleme

    Serial1.println("--- ROKET SISTEMI BASLATILIYOR ---");

    // SD Kart Başlatma
    if (!SD.begin(PIN_SPI_CS)) {
        Serial1.println("UYARI: SD Kart baslatilamadi! Loglama yapilmayacak.");
        sdOk = false;
    } else {
        Serial1.println("SD Kart baslatildi.");
        logFile = SD.open("/ucus_log.csv", FILE_APPEND);
        if (logFile) {
            // Dosya yeni oluşturulduysa veya boşsa başlık yaz
            if (logFile.size() == 0) {
                logFile.println("ivmeX,ivmeY,ivmeZ,gyroX,gyroY,gyroZ,roll,pitch,yaw,basinc,sicaklik,irtifa,nem,hiz,eglim,lat,lng,ayr1,ayr2,state");
            }
            sdOk = true;
        } else {
            Serial1.println("HATA: Log dosyasi acilamadi!");
            sdOk = false;
        }
    }

    // Sensör Başlatma İşlemleri
    // ÖNEMLİ: Sensör bulunamazsa ARTIK sistemi durdurmuyoruz. Aksi halde SİT/SUT
    // komutlarını dinleyen Task2 hiç başlamaz ve test cihazı hiçbir yanıt alamaz.
    // Sensör yoksa sadece uyarı basılır; SUT zaten sensörleri yok sayar, SİT'te de
    // ilgili veriler 0 gönderilir. Bayraklar Task1'in okumayı atlamasını sağlar.
    if (bno.begin()) {
        bnoOk = true;
        // BNO055'i harici kristal kullanmaya ayarlamak okumaları daha stabil yapar
        bno.setExtCrystalUse(true);
        Serial1.println("BNO055 baslatildi.");

        // BNO055 Kalibrasyon Kalitesi Bekleme — SÜRE SINIRLI (max ~10 sn)
        // Test tezgahinda kalibrasyon sys>=1'e hic ulasmayabilir; sonsuz beklemek
        // yerine sinirli bekleyip devam ediyoruz (ucusta kalibrasyon icin ayri onlem alin).
        Serial1.println("BNO055 kalibrasyonu bekleniyor (max 10sn)...");
        uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
        unsigned long kal_baslangic = millis();
        while (cal_sys < BNO055_MIN_KALIBRASYON && (millis() - kal_baslangic) < 10000) {
            bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);
            char buf[80];
            snprintf(buf, sizeof(buf), "Kal: Sys=%d/3 Gyro=%d/3 Accel=%d/3 Mag=%d/3",
                     cal_sys, cal_gyro, cal_accel, cal_mag);
            Serial1.println(buf);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        Serial1.println(cal_sys >= BNO055_MIN_KALIBRASYON
                        ? "BNO055 kalibrasyonu tamamlandi!"
                        : "UYARI: BNO055 kalibrasyon zaman asimi, yine de devam.");
    } else {
        bnoOk = false;
        Serial1.println("UYARI: BNO055 bulunamadi! IMU verileri 0 gonderilecek (SUT etkilenmez).");
    }

    // BME280 genelde 0x76 veya 0x77 I2C adresi kullanır
    if (bme.begin(BME280_ADDR_PRIMARY) || bme.begin(BME280_ADDR_SECONDARY)) {
        bmeOk = true;
        Serial1.println("BME280 baslatildi.");
        // Gelişmiş okuma ayarları (BME280)
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,  // Sicaklik
                        Adafruit_BME280::SAMPLING_X16, // Basinc
                        Adafruit_BME280::SAMPLING_X1,  // Nem
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5);

        // 3.1 Ground Kalibrasyon: Anlık yer seviyesi basıncını ölç (20 örnek ortalaması)
        delay(200);
        Serial1.println("Yer kalibrasyonu yapiliyor...");
        float basinc_toplam = 0.0;
        for (int i = 0; i < 20; i++) {
            basinc_toplam += bme.readPressure() / 100.0F; // Pascal → hPa
            delay(50);
        }
        referans_basinc = basinc_toplam / 20.0;
        char buf[40];
        snprintf(buf, sizeof(buf), "Referans Basinc (hPa): %.2f", referans_basinc);
        Serial1.println(buf);
    } else {
        bmeOk = false;
        Serial1.println("UYARI: BME280 bulunamadi! Barometre verileri 0 gonderilecek (SUT etkilenmez).");
    }

    // FreeRTOS Kuyruk Başlatma (Maksimum 10 paketlik yer ayıralım)
    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(TelemetryPacket));
    if(telemetryQueue == NULL){
      Serial1.println("KRITIK: Kuyruk olusturulamadi! Sistem durduruluyor.");
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
    
    Serial1.println("Setup Tamam. Gorevler Dagitildi.");
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