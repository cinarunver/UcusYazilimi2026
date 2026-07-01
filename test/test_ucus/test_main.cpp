/**
 * ============================================================
 * TRAKYA ROKET 2026 - UcusYazilimi Test Suite (main.cpp v2 - DMA)
 * ============================================================
 * IKI KATMAN:
 *   [A] YAZILIM (saf mantik)  - donanim gerektirmez, algoritma dogrulugu
 *         Kalman, dikey hiz, egim, CRC16, cerceveleme, state machine
 *   [B] DONANIM (kart uzerinde GERCEK test) - sensorler & SD kart takili olmali
 *         I2C tarama + chip-ID, sensor verisi akil-saglik araligi,
 *         GERCEK SD karta yaz/oku + ping-pong KAYIPSIZLIK
 *
 * >>> DONANIM testleri gercek pinler/adreslerle main.cpp ile birebir calisir.
 * >>> Sensor/SD takili degilse ILGILI test FAIL verir (amac karti test etmek).
 *
 * Calistirmak icin (kart USB'ye bagli):
 *   pio test -e esp32dev -f test_ucus
 * ============================================================
 */

#include <unity.h>
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>

// ============================================================
// DONANIM PINLERI / ADRESLERI (main.cpp ile BIREBIR)
// ============================================================
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_SPI_CLK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_MOSI 23
#define PIN_SPI_CS 5

#define BNO055_DEF            55
#define BNO055_ADDR           0x28
#define BME280_ADDR_PRIMARY   0x76
#define BME280_ADDR_SECONDARY 0x77

#define BNO055_CHIP_ID_REG    0x00
#define BNO055_CHIP_ID_VAL    0xA0
#define BME280_ID_REG         0xD0
#define BME280_ID_VAL         0x60

// --- UCUS ALGORITMASI SABITLERI (main.cpp ile senkron) ---
#define APOGEE_IRTIFA_FARKI   15.0f
#define AYRILMA2_MESAFE      550.0f
#define MAX_EGLIM             10.0f
#define MIN_DIKEY_HIZ          0.0f
#define KALKIS_IVME_ESIGI     20.0f
#define INIS_HIZ_ESIGI         2.0f
#define INIS_IRTIFA_ESIGI     20.0f

// --- CERCEVE PROTOKOLU (main.cpp ile senkron) ---
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55
#define PACKET_SIZE          59
#define FRAME_SIZE           (PACKET_SIZE + 5)  // 64

#define SD_DMA_BUF_SIZE      512

// --- Sensor nesneleri (main.cpp ile ayni) ---
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_DEF, BNO055_ADDR);
Adafruit_BME280 bme;
bool bnoOk = false;
bool bmeOk = false;
bool sdOk  = false;

// ============================================================
// [A] YAZILIM: main.cpp donanim-bagimsiz kod kopyalari
// ============================================================

// --- KALMAN FILTRESI (main.cpp ile birebir) ---
class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) :
      err_measure(mea_e), err_estimate(est_e), q(q),
      last_estimate(0), kalman_gain(0), first_run(true) {}
    float updateEstimate(float mea) {
      if (first_run) { last_estimate = mea; first_run = false; }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      float ce = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabsf(last_estimate - ce) * q;
      last_estimate = ce;
      return ce;
    }
  private:
    float err_measure, err_estimate, q, last_estimate, kalman_gain;
    bool  first_run;
};

// --- UCUS DURUM MAKINESI ---
enum UcusDurumu { HAZIR = 0, YUKSELIYOR = 1, INIS_1 = 2, INIS_2 = 3, INDI = 4 };

// --- TELEMETRI PAKETI (GUNCEL main.cpp - 59 byte) ---
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
#pragma pack(pop)

// --- DIKEY HIZ (hardware-free versiyon) ---
float hesapla_dikey_hiz_test(float onceki_irtifa, float guncel_irtifa,
                             unsigned long onceki_us, unsigned long guncel_us) {
    if (onceki_us == 0) return 0.0f;
    float dt = (float)(guncel_us - onceki_us) / 1000000.0f;
    if (dt <= 0.0f) return 0.0f;
    return (guncel_irtifa - onceki_irtifa) / dt;
}

// --- EGIM (TILT) ACISI (main.cpp ile birebir + NaN korumasi) ---
static const float T_DEG_TO_RAD = M_PI / 180.0f;
static const float T_RAD_TO_DEG = 180.0f / M_PI;
float hesapla_eglim_acisi(float pitch_deg, float roll_deg) {
    float cv = cosf(pitch_deg * T_DEG_TO_RAD) * cosf(roll_deg * T_DEG_TO_RAD);
    if (cv >  1.0f) cv =  1.0f;
    if (cv < -1.0f) cv = -1.0f;
    return acosf(cv) * T_RAD_TO_DEG;
}

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

// --- CERCEVELEME (gonder_paket_framed_dma mantigi) ---
size_t cercevele(uint8_t* out, const TelemetryPacket& pkt) {
    const uint8_t* p = (const uint8_t*)&pkt;
    const size_t len = sizeof(TelemetryPacket);
    uint16_t crc = crc16_ccitt(p, len);
    size_t idx = 0;
    out[idx++] = SYNC_BYTE_1; out[idx++] = SYNC_BYTE_2; out[idx++] = (uint8_t)len;
    memcpy(&out[idx], p, len); idx += len;
    out[idx++] = (uint8_t)(crc >> 8); out[idx++] = (uint8_t)(crc & 0xFF);
    return idx;
}

// --- CSV SATIR FORMATI (main.cpp bufferla_ve_yaz_sd ile birebir) ---
int format_csv_line(char* out, size_t cap, const TelemetryPacket& p) {
    return snprintf(out, cap,
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%d,%d,%d\n",
        p.ivmeX, p.ivmeY, p.ivmeZ, p.gyroX, p.gyroY, p.gyroZ,
        p.roll, p.pitch, p.yaw, p.irtifa, p.dikeyHiz, p.eglimAcisi,
        p.gpsEnlem, p.gpsBoylam, p.ayrilma1_durum, p.ayrilma2_durum, p.ucus_durumu);
}

// --- SD PING-PONG (main.cpp ile birebir; GERCEK File'a yazar) ---
static char sd_dma_buf_A[SD_DMA_BUF_SIZE];
static char sd_dma_buf_B[SD_DMA_BUF_SIZE];
static int  active_sd_buf = 0;
static int  sd_buf_idx    = 0;

void sd_state_reset() { active_sd_buf = 0; sd_buf_idx = 0; }

void bufferla_ve_yaz_sd(File& file, const TelemetryPacket& pkt) {
    char temp_line[160];
    int line_len = format_csv_line(temp_line, sizeof(temp_line), pkt);
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

void sd_buffer_bosalt(File& file) {
    if (file && sd_buf_idx > 0) {
        char* current_buf = (active_sd_buf == 0) ? sd_dma_buf_A : sd_dma_buf_B;
        file.write((const uint8_t*)current_buf, sd_buf_idx);
        sd_buf_idx = 0;
    }
    if (file) file.flush();
}

// Tek register okuma (I2C chip-ID dogrulama icin)
bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t* val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, 1) != 1) return false;
    *val = Wire.read();
    return true;
}

static TelemetryPacket ornek_paket(int i) {
    TelemetryPacket p; memset(&p, 0, sizeof(p));
    p.ivmeX = i * 1.5f; p.ivmeZ = 9.8f + i;
    p.irtifa = 100.0f + i * 3.0f; p.dikeyHiz = -i * 0.25f;
    p.eglimAcisi = (float)(i % 90);
    p.gpsEnlem = 41.012345f; p.gpsBoylam = 28.987654f;
    p.ayrilma1_durum = (i % 2); p.ucus_durumu = (uint8_t)(i % 5);
    return p;
}

// ============================================================
// [A] YAZILIM TESTLERI
// ============================================================

void test_kalman_ilk_cagri(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.0f, kf.updateEstimate(42.0f));
}
void test_kalman_yakinsar(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    float r = 0; for (int i = 0; i < 50; i++) r = kf.updateEstimate(100.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, r);
}
void test_kalman_gurultu_azaltir(void) {
    SimpleKalmanFilter kf(2.0f, 2.0f, 0.1f);
    float in[] = {108,92,105,95,103,97,101,99,102,98};
    float r = 0; for (int i = 0; i < 10; i++) r = kf.updateEstimate(in[i]);
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 100.0f, r);
}
void test_kalman_nan_uretmiyor(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    TEST_ASSERT_FALSE(isnan(kf.updateEstimate(0.0f)));
    TEST_ASSERT_FALSE(isnan(kf.updateEstimate(1000.0f)));
}

void test_dikey_hiz_yukselis(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, hesapla_dikey_hiz_test(0, 100, 1000UL, 1001000UL));
}
void test_dikey_hiz_inis(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, hesapla_dikey_hiz_test(500, 450, 1000UL, 2001000UL));
}
void test_dikey_hiz_ilk_cagri_sifir(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, hesapla_dikey_hiz_test(0, 100, 0, 0));
}

void test_eglim_dik_sifir(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, hesapla_eglim_acisi(0, 0));
}
void test_eglim_guvenlik_gecer(void) {
    TEST_ASSERT_TRUE(hesapla_eglim_acisi(2, 2) < MAX_EGLIM);
}
void test_eglim_tumbling_engeller(void) {
    TEST_ASSERT_TRUE(hesapla_eglim_acisi(45, 0) >= MAX_EGLIM);
}
void test_eglim_nan_uretmiyor(void) {
    TEST_ASSERT_FALSE(isnan(hesapla_eglim_acisi(180, 180)));
}

void test_packet_boyutu_59(void) {
    TEST_ASSERT_EQUAL_UINT(59, sizeof(TelemetryPacket));
}
void test_packet_padding_yok(void) {
    TelemetryPacket p;
    size_t off = (size_t)((uint8_t*)&p.ucus_durumu - (uint8_t*)&p);
    TEST_ASSERT_EQUAL_UINT(58, off);
}

void test_crc16_bilinen_vektor(void) {
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt((const uint8_t*)"123456789", 9));
}
void test_crc16_tek_bit_farkli(void) {
    uint8_t a[4] = {0x10,0x20,0x30,0x40}, b[4] = {0x10,0x20,0x30,0x41};
    TEST_ASSERT_NOT_EQUAL(crc16_ccitt(a,4), crc16_ccitt(b,4));
}

void test_cerceve_baslik_ve_boyut(void) {
    TelemetryPacket p; memset(&p, 0, sizeof(p));
    uint8_t f[FRAME_SIZE];
    size_t n = cercevele(f, p);
    TEST_ASSERT_EQUAL_UINT(64, n);
    TEST_ASSERT_EQUAL_HEX8(0xAA, f[0]);
    TEST_ASSERT_EQUAL_HEX8(0x55, f[1]);
    TEST_ASSERT_EQUAL_UINT8(59, f[2]);
}
void test_cerceve_payload_bozulmadan(void) {
    TelemetryPacket p; memset(&p, 0, sizeof(p));
    p.irtifa = 1234.5f; p.ucus_durumu = 3;
    uint8_t f[FRAME_SIZE]; cercevele(f, p);
    TEST_ASSERT_EQUAL_MEMORY(&p, &f[3], sizeof(TelemetryPacket));
}
void test_cerceve_bozuk_payload_crc_yakalar(void) {
    TelemetryPacket p; memset(&p, 0, sizeof(p)); p.irtifa = 500.0f;
    uint8_t f[FRAME_SIZE]; cercevele(f, p);
    f[10] ^= 0xFF; // bit flip
    uint16_t alinan = ((uint16_t)f[FRAME_SIZE-2] << 8) | f[FRAME_SIZE-1];
    uint16_t hesap  = crc16_ccitt(&f[3], sizeof(TelemetryPacket));
    TEST_ASSERT_NOT_EQUAL(alinan, hesap);
}

void test_sm_kalkis(void) {
    UcusDurumu d = HAZIR;
    if (25.0f > KALKIS_IVME_ESIGI) d = YUKSELIYOR;
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, d);
}
void test_sm_apogee_tam_kosul(void) {
    UcusDurumu d = YUKSELIYOR; bool ayr = false;
    float maxi = 600, irt = 580, hiz = -5, eg = 5;
    if ((maxi-irt > APOGEE_IRTIFA_FARKI) && (hiz < MIN_DIKEY_HIZ) && (eg < MAX_EGLIM)) { ayr = true; d = INIS_1; }
    TEST_ASSERT_EQUAL_INT(INIS_1, d); TEST_ASSERT_TRUE(ayr);
}
void test_sm_apogee_egim_engeller(void) {
    UcusDurumu d = YUKSELIYOR; bool ayr = false;
    float maxi = 600, irt = 580, hiz = -5, eg = 45;
    if ((maxi-irt > APOGEE_IRTIFA_FARKI) && (hiz < MIN_DIKEY_HIZ) && (eg < MAX_EGLIM)) { ayr = true; d = INIS_1; }
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, d); TEST_ASSERT_FALSE(ayr);
}
void test_sm_inis1_ana_parasut(void) {
    UcusDurumu d = INIS_1; bool ayr = false;
    float irt = 400, maxi = 700;
    if ((irt < AYRILMA2_MESAFE) && (maxi > AYRILMA2_MESAFE)) { ayr = true; d = INIS_2; }
    TEST_ASSERT_EQUAL_INT(INIS_2, d); TEST_ASSERT_TRUE(ayr);
}
void test_sm_inis2_yere_inis(void) {
    UcusDurumu d = INIS_2;
    float hiz = -1.5f, irt = 10.0f;
    if ((hiz > -INIS_HIZ_ESIGI) && (irt < INIS_IRTIFA_ESIGI)) d = INDI;
    TEST_ASSERT_EQUAL_INT(INDI, d);
}

// ============================================================
// [B] DONANIM TESTLERI (GERCEK KART)
// ============================================================

// --- I2C: BNO055 bagli mi + chip-ID ---
void test_hw_i2c_bno055_cevap(void) {
    Wire.beginTransmission(BNO055_ADDR);
    TEST_ASSERT_EQUAL_MESSAGE(0, Wire.endTransmission(),
        "BNO055 (0x28) I2C hattinda cevap vermiyor - kablo/besleme kontrol");
}
void test_hw_bno055_chip_id(void) {
    uint8_t id = 0;
    TEST_ASSERT_TRUE_MESSAGE(i2c_read_reg(BNO055_ADDR, BNO055_CHIP_ID_REG, &id),
        "BNO055 CHIP_ID register okunamadi");
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(BNO055_CHIP_ID_VAL, id,
        "BNO055 CHIP_ID beklenen 0xA0 degil");
}

// --- I2C: BME280 bagli mi + chip-ID ---
void test_hw_i2c_bme280_cevap(void) {
    Wire.beginTransmission(BME280_ADDR_PRIMARY);
    bool p = (Wire.endTransmission() == 0);
    Wire.beginTransmission(BME280_ADDR_SECONDARY);
    bool s = (Wire.endTransmission() == 0);
    TEST_ASSERT_TRUE_MESSAGE(p || s,
        "BME280 (0x76/0x77) I2C hattinda cevap vermiyor");
}
void test_hw_bme280_chip_id(void) {
    uint8_t id = 0;
    bool ok = i2c_read_reg(BME280_ADDR_PRIMARY, BME280_ID_REG, &id) ||
              i2c_read_reg(BME280_ADDR_SECONDARY, BME280_ID_REG, &id);
    TEST_ASSERT_TRUE_MESSAGE(ok, "BME280 ID register okunamadi");
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(BME280_ID_VAL, id,
        "BME280 ID beklenen 0x60 degil (BMP280 olabilir)");
}

// --- BNO055 baslar + verisi akil-saglik araliginda ---
void test_hw_bno055_baslar(void) {
    bnoOk = bno.begin();
    TEST_ASSERT_TRUE_MESSAGE(bnoOk, "bno.begin() basarisiz");
}
void test_hw_bno055_veri_makul(void) {
    TEST_ASSERT_TRUE_MESSAGE(bnoOk, "BNO055 baslamadigi icin veri okunamiyor");
    sensors_event_t a, o;
    bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&o, Adafruit_BNO055::VECTOR_EULER);
    // Veri NaN olmamali
    TEST_ASSERT_FALSE(isnan(a.acceleration.x));
    TEST_ASSERT_FALSE(isnan(o.orientation.x));
    // Sicaklik makul aralikta (-40..85 C)
    int8_t t = bno.getTemp();
    TEST_ASSERT_TRUE_MESSAGE(t > -40 && t < 85, "BNO055 sicakligi anlamsiz");
    // Euler heading 0..360
    TEST_ASSERT_TRUE(o.orientation.x >= -0.1f && o.orientation.x <= 360.1f);
}

// --- BME280 baslar + verisi akil-saglik araliginda ---
void test_hw_bme280_baslar(void) {
    bmeOk = bme.begin(BME280_ADDR_PRIMARY) || bme.begin(BME280_ADDR_SECONDARY);
    TEST_ASSERT_TRUE_MESSAGE(bmeOk, "bme.begin() basarisiz");
}
void test_hw_bme280_veri_makul(void) {
    TEST_ASSERT_TRUE_MESSAGE(bmeOk, "BME280 baslamadigi icin veri okunamiyor");
    float t = bme.readTemperature();
    float p = bme.readPressure() / 100.0f; // hPa
    float h = bme.readHumidity();
    TEST_ASSERT_FALSE(isnan(t)); TEST_ASSERT_FALSE(isnan(p));
    TEST_ASSERT_TRUE_MESSAGE(t > -40.0f && t < 85.0f,  "BME280 sicaklik araligi disi");
    TEST_ASSERT_TRUE_MESSAGE(p > 300.0f && p < 1100.0f, "BME280 basinc araligi disi (hPa)");
    TEST_ASSERT_TRUE_MESSAGE(h >= 0.0f && h <= 100.0f,  "BME280 nem %0-100 disi");
}

// --- SD kart baslar ---
void test_hw_sd_baslar(void) {
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);
    sdOk = SD.begin(PIN_SPI_CS);
    TEST_ASSERT_TRUE_MESSAGE(sdOk, "SD.begin() basarisiz - kart takili mi?");
}

// --- SD gercek yaz/oku roundtrip ---
void test_hw_sd_yaz_oku(void) {
    TEST_ASSERT_TRUE_MESSAGE(sdOk, "SD baslamadigi icin yaz/oku atlandi");
    const char* msg = "TRAKYA-ROKET-2026-SDTEST";
    SD.remove("/selftest.txt");
    File w = SD.open("/selftest.txt", FILE_WRITE);
    TEST_ASSERT_TRUE_MESSAGE(w, "SD dosya yazma icin acilamadi");
    w.print(msg); w.close();

    File r = SD.open("/selftest.txt", FILE_READ);
    TEST_ASSERT_TRUE_MESSAGE(r, "SD dosya okuma icin acilamadi");
    char buf[64]; int n = r.readBytes(buf, sizeof(buf) - 1); buf[n] = '\0'; r.close();
    TEST_ASSERT_EQUAL_STRING_MESSAGE(msg, buf, "SD geri okunan veri yazilanla ayni degil");
}

// --- SD GERCEK ping-pong + KAYIPSIZLIK (dosya boyutu == uretilen) ---
void test_hw_sd_pingpong_kayipsiz(void) {
    TEST_ASSERT_TRUE_MESSAGE(sdOk, "SD baslamadigi icin ping-pong testi atlandi");
    sd_state_reset();
    SD.remove("/pptest.csv");
    File f = SD.open("/pptest.csv", FILE_WRITE);
    TEST_ASSERT_TRUE_MESSAGE(f, "ping-pong test dosyasi acilamadi");

    long beklenen = 0;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        TelemetryPacket p = ornek_paket(i);
        char tmp[160];
        beklenen += format_csv_line(tmp, sizeof(tmp), p);
        bufferla_ve_yaz_sd(f, p);
    }
    sd_buffer_bosalt(f); // inis/kapanis: kalan tamponu bas
    f.close();

    File r = SD.open("/pptest.csv", FILE_READ);
    TEST_ASSERT_TRUE(r);
    long boyut = r.size();
    r.close();

    // Gercek karta yazilan dosya boyutu, uretilen toplam byte'a esit olmali (SIFIR KAYIP)
    TEST_ASSERT_EQUAL_INT32_MESSAGE(beklenen, boyut,
        "SD dosya boyutu uretilen veriyle esit degil - veri kaybi var!");
    TEST_ASSERT_EQUAL_INT(0, sd_buf_idx);
}

// ============================================================
// UNITY
// ============================================================
void setUp(void)    {}
void tearDown(void) {}

void setup() {
    delay(2000); // seri port + sensor guc-acilis (BNO055 ~650ms) icin bekle
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    UNITY_BEGIN();

    // --- [A] YAZILIM ---
    RUN_TEST(test_kalman_ilk_cagri);
    RUN_TEST(test_kalman_yakinsar);
    RUN_TEST(test_kalman_gurultu_azaltir);
    RUN_TEST(test_kalman_nan_uretmiyor);
    RUN_TEST(test_dikey_hiz_yukselis);
    RUN_TEST(test_dikey_hiz_inis);
    RUN_TEST(test_dikey_hiz_ilk_cagri_sifir);
    RUN_TEST(test_eglim_dik_sifir);
    RUN_TEST(test_eglim_guvenlik_gecer);
    RUN_TEST(test_eglim_tumbling_engeller);
    RUN_TEST(test_eglim_nan_uretmiyor);
    RUN_TEST(test_packet_boyutu_59);
    RUN_TEST(test_packet_padding_yok);
    RUN_TEST(test_crc16_bilinen_vektor);
    RUN_TEST(test_crc16_tek_bit_farkli);
    RUN_TEST(test_cerceve_baslik_ve_boyut);
    RUN_TEST(test_cerceve_payload_bozulmadan);
    RUN_TEST(test_cerceve_bozuk_payload_crc_yakalar);
    RUN_TEST(test_sm_kalkis);
    RUN_TEST(test_sm_apogee_tam_kosul);
    RUN_TEST(test_sm_apogee_egim_engeller);
    RUN_TEST(test_sm_inis1_ana_parasut);
    RUN_TEST(test_sm_inis2_yere_inis);

    // --- [B] DONANIM (GERCEK KART) ---
    RUN_TEST(test_hw_i2c_bno055_cevap);
    RUN_TEST(test_hw_bno055_chip_id);
    RUN_TEST(test_hw_i2c_bme280_cevap);
    RUN_TEST(test_hw_bme280_chip_id);
    RUN_TEST(test_hw_bno055_baslar);
    RUN_TEST(test_hw_bno055_veri_makul);
    RUN_TEST(test_hw_bme280_baslar);
    RUN_TEST(test_hw_bme280_veri_makul);
    RUN_TEST(test_hw_sd_baslar);
    RUN_TEST(test_hw_sd_yaz_oku);
    RUN_TEST(test_hw_sd_pingpong_kayipsiz);

    UNITY_END();
}

void loop() {}
