/*
================================================================================
  TRAKYA ROKET 2026 - LoRa SADECE AAAA GONDERIM TESTI
================================================================================
  Amac: Donanimin calisip calismadigini test etmek icin her saniye "aaaa" basar.
  Hem USB Serial'e hem de LoRa'ya (Serial1) cikti verir.

  DERLEME:  pio run -e loratest --target upload
  MONITOR:  pio device monitor -b 115200
================================================================================
*/

#include <Arduino.h>

// --- LoRa (Serial1) pinleri ---
#define PIN_LORA_TX 32 // ESP TX -> LoRa DIN (modulun RXD'si)
#define PIN_LORA_RX 33 // ESP RX <- LoRa DOUT (modulun TXD'si)
#define BAUD_LORA 9600
#define LORA_M0 15
#define LORA_M1 2

// --- E32 ADRES / KANAL ---
#define LORA_ADDH 0x00 // Adres yuksek byte
#define LORA_ADDL 0x02 // Adres dusuk byte -> ADRES = 2
#define LORA_CHAN 0x16 // Kanal 22 (0x16)  -> 432 MHz
#define LORA_SPED 0x1C // UART 9600 8N1 + hava hizi

// --- OPTION byte bilesenleri (E32-433T30D datasheet §7.5) ---
// OPTION = TX_MODE | IO_DRIVE | WOR_TIME | FEC | TX_POWER  (asagidan otomatik
// birlesir)
#define LORA_TX_MODE                                                           \
  0x00 // bit7  0x00=SEFFAF (transparent) | 0x80=SABIT (fixed) - Test icin
       // varsayilan SEFFAF (0x00)
#define LORA_IO_DRIVE                                                          \
  0x40 // bit6  0x40=push-pull/pull-up (varsayilan) | 0x00=open-collector
#define LORA_WOR_TIME                                                          \
  0x00 // bit5-3 uyanma(WOR) suresi: 0x00=250 0x08=500 0x10=750 0x18=1000
       //                            0x20=1250 0x28=1500 0x30=1750 0x38=2000 ms
#define LORA_FEC 0x04 // bit2  0x04=FEC ACIK (varsayilan) | 0x00=FEC kapali
#define LORA_TX_POWER                                                          \
  0x00 // bit1-0 TX gucu: 0x00=30dBm(1W) 0x01=27 0x02=24 0x03=21 dBm

#define LORA_OPTION                                                            \
  (LORA_TX_MODE | LORA_IO_DRIVE | LORA_WOR_TIME | LORA_FEC | LORA_TX_POWER)

unsigned long son_gonderim = 0;
unsigned long sayac = 0;

void lora_konfigurasyon() {
  static const uint8_t cfg[6] = {0xC0,      LORA_ADDH, LORA_ADDL,
                                 LORA_SPED, LORA_CHAN, LORA_OPTION};

  Serial.println("[LoRa] Konfigurasyon moduna geciliyor...");
  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH); // Config modu
  delay(100);

  // Eski verileri temizle
  while (Serial1.available())
    Serial1.read();

  Serial.print("[LoRa] Config paketi gonderiliyor: ");
  for (int i = 0; i < 6; i++)
    Serial.printf("%02X ", cfg[i]);
  Serial.println();

  Serial1.write(cfg, sizeof(cfg));
  Serial1.flush();
  delay(200);

  uint8_t resp[8];
  int n = 0;
  while (Serial1.available() && n < (int)sizeof(resp)) {
    resp[n++] = Serial1.read();
  }

  Serial.print("[LoRa] Config yaniti: ");
  for (int i = 0; i < n; i++)
    Serial.printf("%02X ", resp[i]);
  Serial.println(n ? "" : "(Yanit yok! Baglantilari kontrol et)");

  Serial.println("[LoRa] Normal moda geciliyor...");
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW); // Normal/Seffaf mod
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);

  // LoRa UART
  Serial1.begin(BAUD_LORA, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

  lora_konfigurasyon();

  Serial.println("\n--- LoRa AAAA Testi Basladi ---");
  Serial.printf(
      "Mod: %s (Option: 0x%02X)\n",
      ((LORA_TX_MODE == 0x80) ? "SABIT (Fixed)" : "SEFFAF (Transparent)"),
      LORA_OPTION);
  Serial.printf("Hedef Kanal: %d (frekans: %d MHz)\n", LORA_CHAN,
                410 + LORA_CHAN);
}

void loop() {
  if (millis() - son_gonderim >= 1000) {
    son_gonderim = millis();
    sayac++;

    Serial.printf("[TX #%lu] aaaa gonderiliyor...\n", sayac);

    if (LORA_TX_MODE == 0x80) {
      // Sabit modda gonderim yaparken basina hedef ADDH, ADDL ve KANAL
      // eklenmelidir. Alici olarak ADDH=0x00, ADDL=0x02, KANAL=0x16 (22) kabul
      // ediyoruz.
      Serial1.write(0x00); // Hedef ADDH
      Serial1.write(0x02); // Hedef ADDL
      Serial1.write(0x16); // Hedef KANAL
    }

    Serial1.print("aaaa\r\n"); // LoRa uzerinden gonder
  }
}
