#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  RS232 TEST — Saniyede 10 kere 'a' basar
//  Hat: UART0 (Serial) → MAX3232 → RS232
//  TX = GPIO1, RX = GPIO3, Baud = 115200 (SİT/SUT TTL ile ayni)
// ─────────────────────────────────────────────────────────────

#define BAUD_RS232 115200
#define GONDERIM_PERIYOT_MS 100   // 100 ms = 10 Hz

void setup() {
    Serial.begin(BAUD_RS232);
}

void loop() {
    static unsigned long son_gonderim = 0;

    if (millis() - son_gonderim >= GONDERIM_PERIYOT_MS) {
        son_gonderim += GONDERIM_PERIYOT_MS;   // periyot kaymasi olmadan 10 Hz
        Serial.write('a');
    }
}
