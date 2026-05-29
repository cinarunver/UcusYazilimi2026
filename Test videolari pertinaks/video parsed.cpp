#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Uyarlanabilir Sabitler
float referans_basinc = 1013.25;

// Uyarlanabilir Pinler 
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_LORA_TX 33
#define PIN_LORA_RX 32

// Haberleşme Sabitleri
#define BAUD_LORA              9600
#define LORA_GONDERIM_ORANI    10

// Çerçeve Protokolü Sabitleri
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55

// Sensör Sabitleri
#define BNO055_DEF 55
#define BNO055_ADDR 0x28
#define BME280_ADDR_PRIMARY 0x76
#define BME280_ADDR_SECONDARY 0x77
#define BNO055_MIN_KALIBRASYON 0

// Uçuş Algoritması Sabitleri
#define APOGEE_IRTIFA_FARKI   15.0  // m
#define AYRILMA2_MESAFE      550.0  // m
#define MAX_EGLIM             10.0  // derece
#define MIN_DIKEY_HIZ          0.0  // m/s
#define KALKIS_IVME_ESIGI     20.0  // m/s²
#define INIS_HIZ_ESIGI         2.0  // m/s
#define INIS_IRTIFA_ESIGI     20.0  // m

// FreeRTOS Sabitleri
#define TASK_STACK_SIZE 10000
#define TASK1_PRIORITY 2
#define TASK2_PRIORITY 1
#define TELEMETRY_QUEUE_LEN 100

// Uçuş Durum Makinesi (State Machine)
enum UcusDurumu {
    HAZIR      = 0,
    YUKSELIYOR = 1,
    INIS_1     = 2,
    INIS_2     = 3,
    INDI       = 4
};
UcusDurumu durum = HAZIR;

bool ayrilma1 = false;
bool ayrilma2 = false;
float max_irtifa_degeri = 0.0;

TaskHandle_t Task1;
TaskHandle_t Task2;

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
Adafruit_BME280 bme; 

// --- SENSÖR VERİ DEĞİŞKENLERİ ---
float ivmeX = 0.0, ivmeY = 0.0, ivmeZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float roll = 0.0, pitch = 0.0, yaw = 0.0; 
float basinc = 0.0, bmeSicaklik = 0.0, irtifa = 0.0, nem = 0.0;

#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float irtifa;
    float dikeyHiz; 
    float eglimAcisi; 
    bool ayrilma1_durum; 
    bool ayrilma2_durum;
    uint8_t ucus_durumu; 
};
#pragma pack(pop)

QueueHandle_t telemetryQueue;

HardwareSerial loraSerial(1); // UART1

float hesapla_dikey_hiz(float guncel_irtifa) {
    unsigned long suanki_zaman = micros();
    if (onceki_zaman == 0) {
        onceki_zaman = suanki_zaman;
        onceki_irtifa = guncel_irtifa;
        return 0.0;
    }
    float delta_t = (float)(suanki_zaman - onceki_zaman) / 1000000.0f;
    if (delta_t <= 0.0) return anlik_dikey_hiz; 
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

// --- ÇERÇEVELI PAKET GÖNDERME ---
void gonder_paket_framed(const TelemetryPacket& pkt) {
    static uint8_t frame_buf[80]; 
    
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

    loraSerial.write(frame_buf, idx);
}

// Loglama fonksiyonu: Hem Serial'e (USB) hem LoRa'ya basar
void logMesaj(const char* msg) {
    Serial.print(msg);
    loraSerial.print(msg);
}

void Task1code(void *pvParameters) {
  for (;;) {
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
    yaw = kf_yaw.updateEstimate(o.orientation.x);
    roll = kf_roll.updateEstimate(o.orientation.y);
    pitch = kf_pitch.updateEstimate(o.orientation.z);

    float p_rad = pitch * DEG_TO_RAD;
    float r_rad = roll * DEG_TO_RAD;
    float cos_val = cos(p_rad) * cos(r_rad);
    cos_val = constrain(cos_val, -1.0f, 1.0f);
    eglim_acisi = acos(cos_val) * RAD_TO_DEG;

    bmeSicaklik = kf_bmeSicaklik.updateEstimate(bme.readTemperature());
    basinc = kf_basinc.updateEstimate(bme.readPressure());
    nem = kf_nem.updateEstimate(bme.readHumidity());
    irtifa = kf_irtifa.updateEstimate(bme.readAltitude(referans_basinc));

    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);

    // --- UÇUŞ ALGORİTMASI ---
    switch (durum) {
        case HAZIR:
            if (ivmeZ > KALKIS_IVME_ESIGI) {
                durum = YUKSELIYOR;
            }
            break;

        case YUKSELIYOR:
            if (irtifa > max_irtifa_degeri) {
                max_irtifa_degeri = irtifa;
            }

            if ((max_irtifa_degeri - irtifa > APOGEE_IRTIFA_FARKI) &&  
                (anlik_dikey_hiz < MIN_DIKEY_HIZ) &&                   
                (eglim_acisi < MAX_EGLIM)) {                           
                ayrilma1 = true;
                durum = INIS_1;
            }
            break;

        case INIS_1:
            if ((irtifa < AYRILMA2_MESAFE) && (max_irtifa_degeri > AYRILMA2_MESAFE)) {
                ayrilma2 = true;
                durum = INIS_2;
            }
            break;

        case INIS_2:
            if ((anlik_dikey_hiz > -INIS_HIZ_ESIGI) && (irtifa < INIS_IRTIFA_ESIGI)) {
                durum = INDI;
            }
            break;

        case INDI:
            break;
    }

    TelemetryPacket packet;
    packet.ivmeX = ivmeX; packet.ivmeY = ivmeY; packet.ivmeZ = ivmeZ;
    packet.gyroX = gyroX; packet.gyroY = gyroY; packet.gyroZ = gyroZ;
    packet.roll = roll; packet.pitch = pitch; packet.yaw = yaw;
    packet.irtifa = irtifa;
    packet.dikeyHiz = anlik_dikey_hiz; 
    packet.eglimAcisi = eglim_acisi; 
    
    packet.ayrilma1_durum = ayrilma1; 
    packet.ayrilma2_durum = ayrilma2;
    packet.ucus_durumu = (uint8_t)durum;

    xQueueSend(telemetryQueue, &packet, 0);

    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}
void Task2code(void *pvParameters) {
  TelemetryPacket incomingPacket;
  uint32_t lora_sayac = 0; 

  for (;;) {
    if (xQueueReceive(telemetryQueue, &incomingPacket, portMAX_DELAY) == pdTRUE) {
        lora_sayac++;
        if (lora_sayac >= LORA_GONDERIM_ORANI) {
            // LoRa üzerinden çerçevelenmiş binary veri gönderimi
            gonder_paket_framed(incomingPacket);
            
            // Saniyede 1 kez sensör verilerini Serial'den (USB) izleyebilmek için
            Serial.printf("Irtifa: %.2f | Vz: %.2f | Eglim: %.2f | Pitch: %.2f | Durum: %d\n", 
                          incomingPacket.irtifa, incomingPacket.dikeyHiz, incomingPacket.eglimAcisi, incomingPacket.pitch, incomingPacket.ucus_durumu);
                          
            lora_sayac = 0;
        }
    }
  }
}

void setup() {
    // USB Serial (PC'den okumak için)
    Serial.begin(115200);
    delay(1000);
    
    // LoRa Serial (UART1)
    loraSerial.begin(BAUD_LORA, SERIAL_8N1, 32, 33);
    delay(500);
    
    logMesaj("\n--- ROKET TEST SISTEMI BASLATILIYOR (Sadece Sensorler & LoRa) ---\n");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 

    if (!bno.begin()) {
        const char* err1 = "KRITIK: BNO055 bulunamadi! Sistem durduruluyor.\n";
        while(true) { 
            logMesaj(err1);
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
        }
    }
    logMesaj("BNO055 baslatildi.\n");

    logMesaj("BNO055 kalibrasyonu bekleniyor...\n");
    {
        uint8_t cal_sys = 0, cal_gyro = 0, cal_accel = 0, cal_mag = 0;
        while (cal_sys < BNO055_MIN_KALIBRASYON) {
            bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);
            char buf[80];
            snprintf(buf, sizeof(buf), "Kal: Sys=%d/3 Gyro=%d/3 Accel=%d/3 Mag=%d/3\n",
                     cal_sys, cal_gyro, cal_accel, cal_mag);
            logMesaj(buf);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
    logMesaj("BNO055 kalibrasyonu tamamlandi!\n");

    if (!bme.begin(BME280_ADDR_PRIMARY) && !bme.begin(BME280_ADDR_SECONDARY)) {
        const char* err2 = "KRITIK: BME280 bulunamadi! Sistem durduruluyor.\n";
        while(true) { 
            logMesaj(err2);
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
        }
    }
    logMesaj("BME280 baslatildi.\n");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2, 
                    Adafruit_BME280::SAMPLING_X16, 
                    Adafruit_BME280::SAMPLING_X1,  
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    delay(200);
    logMesaj("Yer kalibrasyonu yapiliyor...\n");
    float basinc_toplam = 0.0;
    for (int i = 0; i < 20; i++) {
        basinc_toplam += bme.readPressure() / 100.0F; 
        delay(50);
    }
    referans_basinc = basinc_toplam / 20.0;
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Referans Basinc (hPa): %.2f\n", referans_basinc);
        logMesaj(buf);
    }

    telemetryQueue = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(TelemetryPacket));
    if(telemetryQueue == NULL){
      const char* err3 = "KRITIK: Kuyruk olusturulamadi! Sistem durduruluyor.\n";
      while(true) { 
          logMesaj(err3);
          vTaskDelay(1000 / portTICK_PERIOD_MS); 
      }
    }

    BaseType_t res1 = xTaskCreatePinnedToCore(
        Task1code, "UcusGorevi", TASK_STACK_SIZE, NULL, TASK1_PRIORITY, &Task1, 0); 
    if (res1 != pdPASS) {
        const char* err_task1 = "KRITIK: UcusGorevi baslatilamadi!\n";
        while(true) {
            logMesaj(err_task1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    delay(100); 

    BaseType_t res2 = xTaskCreatePinnedToCore(
        Task2code, "HaberlesmeGorevi", TASK_STACK_SIZE, NULL, TASK2_PRIORITY, &Task2, 1); 
    if (res2 != pdPASS) {
        const char* err_task2 = "KRITIK: HaberlesmeGorevi baslatilamadi!\n";
        while(true) {
            logMesaj(err_task2);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    
    logMesaj("Setup Tamam. Sensorler Okunuyor ve LoRa uzerinden iletiliyor.\n");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
