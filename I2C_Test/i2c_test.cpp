/**
 * ============================================================
 * TRAKYA ROKET 2026 - BNO055 HAM REGISTER (IMU MODE 0x08)
 * ============================================================
 * BNO055'i Adafruit kutuphanesi olmadan, dogrudan I2C register
 * yazarak IMU moduna (0x08) alir ve verisini basar.
 *   I2C: SDA=21, SCL=22 @ 100 kHz
 *   BNO055 adresi: 0x28
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>

#define BNO_ADDR    0x28
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// --- BNO055 register adresleri ---
#define REG_CHIP_ID     0x00
#define REG_ACC_X_LSB   0x08   // accel veri (6 byte: X,Y,Z LSB/MSB)
#define REG_EUL_H_LSB   0x1A   // euler veri (6 byte: H,R,P LSB/MSB)
#define REG_TEMP        0x34
#define REG_ST_RESULT   0x36
#define REG_SYS_STATUS  0x39
#define REG_SYS_ERR     0x3A
#define REG_OPR_MODE    0x3D
#define REG_SYS_TRIGGER 0x3F

// ---- HAM REGISTER ERISIM YARDIMCILARI ----
void write_register(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BNO_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t read_register(uint8_t reg) {
    Wire.beginTransmission(BNO_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);      // repeated-start
    Wire.requestFrom((int)BNO_ADDR, 1);
    if (Wire.available()) return Wire.read();
    return 0;
}

// Ardisik iki register'i signed 16-bit olarak oku (LSB, MSB)
int16_t read_int16(uint8_t reg_lsb) {
    uint8_t lsb = read_register(reg_lsb);
    uint8_t msb = read_register(reg_lsb + 1);
    return (int16_t)((msb << 8) | lsb);
}

// BNO055'i IMU moduna (0x08) alan sekans (senin sequence'in - aynen)
void bno_imu_moduna_al() {
    // 1. Ensure we are explicitly in Config Mode first
    write_register(0x3D, 0x00);
    delay(20);

    // 2. Force internal oscillator (Clear bit 7 of SYS_TRIGGER)
    write_register(0x3F, 0x00);
    delay(20);

    // 3. Attempt the switch to IMU mode (0x08)
    write_register(0x3D, 0x08);
    delay(50); // Give it extra time to settle

    // 4. Read it back to check
    uint8_t current_mode = read_register(0x3D);
    if (current_mode == 0x00) {
        // If it's STILL 0x00, read the error registers now:
        uint8_t sys_err = read_register(0x3A);
        uint8_t st_result = read_register(0x36);
        // Print these values to your serial monitor to diagnose!
        Serial.printf("[HATA] Mod HALA 0x00! sys_err=0x%02X  st_result=0x%02X\n",
                      sys_err, st_result);
    }
    Serial.printf("Ayarlanan mod: 0x%02X (IMU=0x08 olmali)\n", current_mode);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n### BNO055 HAM REGISTER - IMU MODE (0x08) ###");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(50);
    delay(700); // BNO055 power-on boot (~650ms)

    uint8_t chip = read_register(REG_CHIP_ID);
    Serial.printf("CHIP_ID: 0x%02X (beklenen 0xA0)\n", chip);

    bno_imu_moduna_al();
    delay(50);
}

void loop() {
    // Moddan HIC cikmasin: 0x08 degilse zorla geri al
    uint8_t mode = read_register(REG_OPR_MODE);
    if (mode != 0x08) {
        write_register(REG_OPR_MODE, 0x08);
        delay(30);
        mode = read_register(REG_OPR_MODE);
        Serial.println("[UYARI] mod 0x08 disina cikmisti -> geri alindi");
    }

    // Ham accel (1 m/s^2 = 100 LSB) ve euler (1 derece = 16 LSB)
    float ax = read_int16(REG_ACC_X_LSB)     / 100.0f;
    float ay = read_int16(REG_ACC_X_LSB + 2) / 100.0f;
    float az = read_int16(REG_ACC_X_LSB + 4) / 100.0f;
    float h  = read_int16(REG_EUL_H_LSB)     / 16.0f;
    float r  = read_int16(REG_EUL_H_LSB + 2) / 16.0f;
    float p  = read_int16(REG_EUL_H_LSB + 4) / 16.0f;
    int8_t  temp     = (int8_t)read_register(REG_TEMP);
    uint8_t sys_stat = read_register(REG_SYS_STATUS);
    uint8_t sys_err  = read_register(REG_SYS_ERR);

    Serial.printf("mod:0x%02X | ivme X:%6.2f Y:%6.2f Z:%6.2f m/s^2 | euler H:%6.1f R:%6.1f P:%6.1f "
                  "| T:%dC | sys_stat:%d sys_err:%d\n",
                  mode, ax, ay, az, h, r, p, temp, sys_stat, sys_err);

    delay(200);
}
