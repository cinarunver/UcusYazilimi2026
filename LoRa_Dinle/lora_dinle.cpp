/**
 * ============================================================
 * TRAKYA ROKET 2026 - LORA DINLEYICI
 * ============================================================
 * main.cpp'den forklanmistir. Sadece LoRa (E32-433T30D) UART'ini
 * dinler ve gelen HER BYTE'i USB Serial monitore basar.
 *
 *   LoRa  : UART1  RX=32, TX=33 @ 9600 baud (main.cpp ile ayni)
 *   Cikti : USB Serial @ 115200  ->  pio device monitor -e loradinle
 *
 * Gonderici STRING modunda TAM debug dokumunu (cok satirli, serial monitorun
 * AYNISI) ~LORA_DUMP_MS'de bir gonderir. Gelen ASCII'yi oldugu gibi basiyoruz
 * -> terminalde okunur cok satirli blok cikar.
 *
 * DERLEME/YUKLEME:
 *   pio run -e loradinle --target upload
 *   pio device monitor -e loradinle
 * ============================================================
 */

#include <Arduino.h>

// --- LoRa pinleri (main.cpp ile birebir) ---
#define PIN_LORA_TX 33
#define PIN_LORA_RX 32
#define LORA_M0     15
#define LORA_M1     2
#define BAUD_LORA   9600

void setup() {
    Serial.begin(115200);

    // E32'yi normal (transparan) moda al ki havadan geleni RX'e bassin
    pinMode(LORA_M0, OUTPUT);
    pinMode(LORA_M1, OUTPUT);
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);

    // LoRa UART1: RX=32, TX=33
    Serial1.begin(BAUD_LORA, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

    delay(300);
    Serial.println("\n### LORA DINLEYICI - gelen veri (STRING / TAM DOKUM) ###");
}

void loop() {
    // STRING modu: gonderici ASCII CSV + \r\n gonderiyor.
    // Gelen byte'lari oldugu gibi USB monitore yaz -> satirlar okunabilir cikar.
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        Serial.write(c);
    }
}
