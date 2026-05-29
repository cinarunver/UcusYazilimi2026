/*
 * ============================================================
 *  KALMAN FİLTRE TEST PROGRAMI
 *  BME280 + BNO055 | Ham vs Kalman Filtreli Karşılaştırma
 *  Her satır: [HAM] ve [KAL] değerleri yan yana basılır
 *  USB Serial (115200 baud) üzerinden izlenir
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// --- I2C Pinleri ---
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// --- Sensör Adresleri ---
#define BNO055_DEF          55
#define BNO055_ADDR         0x28
#define BME280_ADDR_PRIMARY 0x76
#define BME280_ADDR_SECONDARY 0x77

// --- Okuma periyodu (ms) ---
#define OKUMA_PERIYODU_MS 100   // 10 Hz

// --- Referans basınç (yer kalibrasyonuyla güncellenecek) ---
float referans_basinc = 1013.25f;

// ============================================================
//  BASİT KALMAN FİLTRESİ
//  mea_e  : ölçüm hatası (büyük → filtreye daha az güven)
//  est_e  : başlangıç tahmin hatası
//  q      : süreç gürültüsü (büyük → filtreyi hızlı adapte et)
// ============================================================
class SimpleKalmanFilter {
public:
    SimpleKalmanFilter(float mea_e, float est_e, float q)
        : err_measure(mea_e), err_estimate(est_e), q(q),
          last_estimate(0.0f), kalman_gain(0.0f), first_run(true) {}

    float updateEstimate(float mea) {
        if (first_run) {
            last_estimate = mea;
            first_run = false;
        }
        kalman_gain = err_estimate / (err_estimate + err_measure);
        float current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
        err_estimate = (1.0f - kalman_gain) * err_estimate
                       + fabsf(last_estimate - current_estimate) * q;
        last_estimate = current_estimate;
        return current_estimate;
    }

    void reset() { first_run = true; }

private:
    float err_measure;
    float err_estimate;
    float q;
    float last_estimate;
    float kalman_gain;
    bool  first_run;
};

// ============================================================
//  KALMAN FİLTRE NESNELERİ
//  BNO055 — ivme, gyro, euler
// ============================================================
SimpleKalmanFilter kf_ivmeX(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter kf_ivmeY(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter kf_ivmeZ(0.1f, 0.1f, 0.01f);

SimpleKalmanFilter kf_gyroX(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter kf_gyroY(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter kf_gyroZ(0.1f, 0.1f, 0.01f);

SimpleKalmanFilter kf_roll (0.1f, 0.1f, 0.01f);
SimpleKalmanFilter kf_pitch(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter kf_yaw  (0.1f, 0.1f, 0.01f);

// BME280 — basınç, sıcaklık, nem, irtifa
SimpleKalmanFilter kf_basinc    (2.0f, 2.0f, 0.1f);
SimpleKalmanFilter kf_sicaklik  (0.5f, 0.5f, 0.01f);
SimpleKalmanFilter kf_nem       (1.0f, 1.0f, 0.1f);
SimpleKalmanFilter kf_irtifa    (1.5f, 1.5f, 0.1f);

// ============================================================
//  SENSÖR NESNELERİ
// ============================================================
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_DEF, BNO055_ADDR);
Adafruit_BME280 bme;

// ============================================================
//  YARDIMCI: Başlık satırı bas
// ============================================================
static void baslikBas() {
    Serial.println(F("\n============================================================"));
    Serial.println(F("  KALMAN FİLTRE TEST | BME280 + BNO055"));
    Serial.println(F("  Format: [HAM]  vs  [KAL]"));
    Serial.println(F("============================================================\n"));

    // BNO başlıkları
    Serial.println(F("--- BNO055 ---"));
    Serial.println(F("  ivmeX_ham  ivmeX_kal | ivmeY_ham  ivmeY_kal | ivmeZ_ham  ivmeZ_kal"));
    Serial.println(F("  gyroX_ham  gyroX_kal | gyroY_ham  gyroY_kal | gyroZ_ham  gyroZ_kal"));
    Serial.println(F("  roll_ham   roll_kal  | pitch_ham  pitch_kal | yaw_ham    yaw_kal"));
    Serial.println(F("--- BME280 ---"));
    Serial.println(F("  bsn_ham    bsn_kal   | sck_ham    sck_kal   | nem_ham    nem_kal   | irt_ham    irt_kal"));
    Serial.println(F("------------------------------------------------------------\n"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1500);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // ---------- BNO055 BAŞLAT ----------
    Serial.println(F("[INIT] BNO055 baslatiliyor..."));
    if (!bno.begin()) {
        Serial.println(F("[HATA] BNO055 bulunamadi! I2C baglantiyi kontrol et."));
        while (true) { delay(1000); }
    }
    bno.setExtCrystalUse(true);
    Serial.println(F("[OK]   BNO055 baslatildi."));

    // ---------- BME280 BAŞLAT ----------
    Serial.println(F("[INIT] BME280 baslatiliyor..."));
    bool bme_ok = bme.begin(BME280_ADDR_PRIMARY);
    if (!bme_ok) {
        bme_ok = bme.begin(BME280_ADDR_SECONDARY);
    }
    if (!bme_ok) {
        Serial.println(F("[HATA] BME280 bulunamadi! I2C baglantiyi kontrol et."));
        while (true) { delay(1000); }
    }
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,   // sıcaklık
                    Adafruit_BME280::SAMPLING_X16,  // basınç
                    Adafruit_BME280::SAMPLING_X1,   // nem
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);
    Serial.println(F("[OK]   BME280 baslatildi."));

    // ---------- YER KALİBRASYONU ----------
    Serial.println(F("[INIT] Yer kalibrasyonu yapiliyor (20 okuma)..."));
    delay(200);
    float toplam = 0.0f;
    for (int i = 0; i < 20; i++) {
        toplam += bme.readPressure() / 100.0f;  // Pa → hPa
        delay(50);
    }
    referans_basinc = toplam / 20.0f;
    Serial.printf("[OK]   Referans Basinc: %.2f hPa\n", referans_basinc);

    baslikBas();

    Serial.println(F("[HAZIR] Okuma basliyor...\n"));
    delay(500);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    static unsigned long son_okuma = 0;
    unsigned long simdi = millis();

    if (simdi - son_okuma < OKUMA_PERIYODU_MS) return;
    son_okuma = simdi;

    // --------------------------------------------------------
    //  BNO055 — HAM OKUMA
    // --------------------------------------------------------
    sensors_event_t ev_ivme, ev_gyro, ev_euler;

    bno.getEvent(&ev_ivme,  Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&ev_gyro,  Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&ev_euler, Adafruit_BNO055::VECTOR_EULER);

    float raw_ivmeX = ev_ivme.acceleration.x;
    float raw_ivmeY = ev_ivme.acceleration.y;
    float raw_ivmeZ = ev_ivme.acceleration.z;

    float raw_gyroX = ev_gyro.gyro.x;
    float raw_gyroY = ev_gyro.gyro.y;
    float raw_gyroZ = ev_gyro.gyro.z;

    // Euler: x=yaw, y=roll, z=pitch (Adafruit konvansiyonu)
    float raw_yaw   = ev_euler.orientation.x;
    float raw_roll  = ev_euler.orientation.y;
    float raw_pitch = ev_euler.orientation.z;

    // --------------------------------------------------------
    //  BNO055 — KALMAN FİLTRELİ
    // --------------------------------------------------------
    float kal_ivmeX = kf_ivmeX.updateEstimate(raw_ivmeX);
    float kal_ivmeY = kf_ivmeY.updateEstimate(raw_ivmeY);
    float kal_ivmeZ = kf_ivmeZ.updateEstimate(raw_ivmeZ);

    float kal_gyroX = kf_gyroX.updateEstimate(raw_gyroX);
    float kal_gyroY = kf_gyroY.updateEstimate(raw_gyroY);
    float kal_gyroZ = kf_gyroZ.updateEstimate(raw_gyroZ);

    float kal_yaw   = kf_yaw  .updateEstimate(raw_yaw);
    float kal_roll  = kf_roll .updateEstimate(raw_roll);
    float kal_pitch = kf_pitch.updateEstimate(raw_pitch);

    // --------------------------------------------------------
    //  BME280 — HAM OKUMA
    // --------------------------------------------------------
    float raw_basinc   = bme.readPressure() / 100.0f;   // hPa
    float raw_sicaklik = bme.readTemperature();           // °C
    float raw_nem      = bme.readHumidity();              // %RH
    float raw_irtifa   = bme.readAltitude(referans_basinc); // m

    // --------------------------------------------------------
    //  BME280 — KALMAN FİLTRELİ
    // --------------------------------------------------------
    float kal_basinc   = kf_basinc  .updateEstimate(raw_basinc);
    float kal_sicaklik = kf_sicaklik.updateEstimate(raw_sicaklik);
    float kal_nem      = kf_nem     .updateEstimate(raw_nem);
    float kal_irtifa   = kf_irtifa  .updateEstimate(raw_irtifa);

    // ============================================================
    //  SERİAL ÇIKTI
    // ============================================================
    Serial.printf(
        "[t=%lu ms]\n"
        "  BNO | ivmeX: HAM=%7.3f  KAL=%7.3f  | ivmeY: HAM=%7.3f  KAL=%7.3f  | ivmeZ: HAM=%7.3f  KAL=%7.3f\n"
        "       gyroX: HAM=%7.3f  KAL=%7.3f  | gyroY: HAM=%7.3f  KAL=%7.3f  | gyroZ: HAM=%7.3f  KAL=%7.3f\n"
        "       roll:  HAM=%7.2f  KAL=%7.2f  | pitch: HAM=%7.2f  KAL=%7.2f  | yaw:   HAM=%7.2f  KAL=%7.2f\n"
        "  BME | bsn:  HAM=%8.2f KAL=%8.2f  | sck:   HAM=%6.2f  KAL=%6.2f  | nem:   HAM=%6.2f  KAL=%6.2f\n"
        "       irt:  HAM=%8.2f KAL=%8.2f\n"
        "------------------------------------------------------------\n",
        simdi,
        raw_ivmeX, kal_ivmeX, raw_ivmeY, kal_ivmeY, raw_ivmeZ, kal_ivmeZ,
        raw_gyroX, kal_gyroX, raw_gyroY, kal_gyroY, raw_gyroZ, kal_gyroZ,
        raw_roll,  kal_roll,  raw_pitch, kal_pitch,  raw_yaw,   kal_yaw,
        raw_basinc, kal_basinc, raw_sicaklik, kal_sicaklik, raw_nem, kal_nem,
        raw_irtifa, kal_irtifa
    );
}
