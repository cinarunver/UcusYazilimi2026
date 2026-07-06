#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  RS232 RX TESTI — HEARTBEAT + ECHO
//  Amac: TX ve RX'i AYNI ANDA gozlemlemek.
//    - Her 500 ms artan sayac basar:  "HB: 0", "HB: 1", ...  -> TX + firmware CANLI
//    - Gelen her byte'i geri yansitir: ">> RX: 0xAA" ...      -> RX CALISIYOR
//
//  Hat: UART0 (Serial), TX=GPIO1, RX=GPIO3, Baud=115200 (SIT/SUT ile ayni)
//
//  YORUM:
//    * Ekranda "HB:" akiyor AMA komut basinca ">> RX:" HIC cikmiyorsa -> RX KOPUK
//      (MAX3232 R-kanali / modul RXD -> GPIO3 teli / GND / PC adaptor TXD)
//    * Komut basinca ">> RX: 0xAA / 0x20 / 0xCA / 0x0D / 0x0A" cikiyorsa -> RX SAGLAM
//      (sorun donanimda degil, sitsut firmware'inin parse tarafinda)
// ─────────────────────────────────────────────────────────────

#define BAUD_RS232   115200
#define HB_PERIYOT_MS   500   // Kalp atisi araligi

void setup() {
    Serial.begin(BAUD_RS232);
    delay(200);
    Serial.println("RX-ECHO + HEARTBEAT HAZIR");
}

void loop() {
    // 1) HEARTBEAT — TX'in ve firmware'in canli oldugunu gosteren artan sayac
    static unsigned long son_hb = 0;
    static uint32_t sayac = 0;
  /*  if (millis() - son_hb >= HB_PERIYOT_MS) {
        son_hb += HB_PERIYOT_MS;
        Serial.print("HB: ");
        Serial.println(sayac++);
    }*/

    // 2) ECHO — gelen her byte'i hemen hex olarak geri yansit (RX kanit)
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();
        Serial1.print(">> RX: 0x");
        if (b < 0x10) Serial.print("0");
        Serial.println(b, HEX);
    }

        // 2) ECHO — gelen her byte'i hemen hex olarak geri yansit (RX kanit)
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();
        Serial.print(">> RX: 0x");
        if (b < 0x10) Serial.print("0");
        Serial.println(b, HEX);
    }
}
