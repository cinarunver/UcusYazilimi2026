/*
================================================================================
  TRAKYA ROKET 2026 - LoRa IZOLE TEST (SADECE GONDERIM)
================================================================================
  Amac: GPS'i denklemden CIKAR. Bu sketch sadece LoRa(Serial1) uzerinden her
  saniye artan bir sayac + mesaj yayinlar. Alicinda (kanal 22 / adres 2) bu
  mesajlar geliyorsa LoRa TX hatti + adres/kanal ayari DOGRU demektir.

  - Serial1 (LoRa) : "TRAKYA LORA TEST #n ..." mesaji  -> alicida gor
  - Serial  (USB)  : ayni mesaj + config yaniti          -> pio device monitor

  DERLEME:  pio run -e loratest --target upload
  MONITOR:  pio device monitor -b 115200
================================================================================
*/

#include <Arduino.h>

// --- LoRa (Serial1) pinleri ---
#define PIN_LORA_TX  32   // ESP TX -> LoRa DIN (modulun RXD'si)
#define PIN_LORA_RX  33   // ESP RX <- LoRa DOUT (modulun TXD'si)
#define BAUD_LORA    9600
#define LORA_M0      15
#define LORA_M1      2

// --- E32 ADRES / KANAL (ALICIYLA AYNI) ---
#define LORA_ADDH    0x00   // Adres yuksek byte
#define LORA_ADDL    0x02   // Adres dusuk byte -> ADRES = 2
#define LORA_CHAN    0x16   // Kanal 22 (0x16)  -> 432 MHz
#define LORA_SPED    0x1C   // UART 9600 8N1 + hava hizi
#define LORA_OPTION  0xC4   // TX gucu / opsiyon byte

unsigned long sayac = 0;
unsigned long son_gonderim = 0;

// --- E32 LoRa KONFIGURASYONU (adres/kanal aliciyla eslesir) ---
void lora_konfigurasyon() {
  static const uint8_t cfg[6] = {0xC0, LORA_ADDH, LORA_ADDL, LORA_SPED, LORA_CHAN, LORA_OPTION};

  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);       // config modu
  delay(50);
  while (Serial1.available()) Serial1.read();

  Serial1.write(cfg, sizeof(cfg));
  Serial1.flush();
  delay(200);

  uint8_t resp[8]; int n = 0;
  while (Serial1.available() && n < (int)sizeof(resp)) resp[n++] = Serial1.read();
  Serial.print("LoRa config yaniti: ");
  for (int i = 0; i < n; i++) Serial.printf("%02X ", resp[i]);
  Serial.println(n ? "" : "(yanit yok - DOUT/RX pini yanlis olabilir, TX yine de calisir)");

  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);        // normal/seffaf mod
  delay(50);
}

// Hem USB Serial'e hem LoRa(Serial1) hattina yazar
void cift_bas(const char *s) {
  Serial.print(s);
  Serial1.print(s);
}

// --- E32'NIN TUM AYARLARINI MODULDEN OKU VE DECODE EDIP BAS ---
// C1 C1 C1 -> 6B parametre ; C3 C3 C3 -> 4B versiyon. Her bit alani cozulur.
void lora_tum_ayarlari_bas() {
  char b[96];

  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);       // config modu
  delay(60);
  while (Serial1.available()) Serial1.read();

  // 1) Calisma parametreleri (C1 C1 C1 -> 6 byte: HEAD ADDH ADDL SPED CHAN OPTION)
  const uint8_t oku_param[3] = {0xC1, 0xC1, 0xC1};
  Serial1.write(oku_param, 3);
  Serial1.flush();
  delay(150);
  uint8_t p[6]; int n = 0;
  while (Serial1.available() && n < 6) p[n++] = Serial1.read();

  cift_bas("\r\n========== E32 LoRa TUM AYARLAR ==========\r\n");
  if (n < 6) {
    snprintf(b, sizeof(b),
             "PARAM OKUNAMADI (%d byte geldi). DOUT->ESP RX (GPIO%d) baglantisini kontrol et.\r\n",
             n, PIN_LORA_RX);
    cift_bas(b);
  } else {
    uint8_t head = p[0], addh = p[1], addl = p[2], sped = p[3], chan = p[4], opt = p[5];
    snprintf(b, sizeof(b), "Ham param (6B): %02X %02X %02X %02X %02X %02X\r\n",
             p[0], p[1], p[2], p[3], p[4], p[5]); cift_bas(b);

    snprintf(b, sizeof(b), "  Kayit turu   : 0x%02X (%s)\r\n", head,
             head == 0xC0 ? "guc kesilince KAYITLI" : head == 0xC2 ? "gecici" : "?"); cift_bas(b);
    snprintf(b, sizeof(b), "  Adres        : %u  (ADDH=0x%02X ADDL=0x%02X)\r\n",
             (addh << 8) | addl, addh, addl); cift_bas(b);

    // SPED byte: [7-6]parite [5-3]uart baud [2-0]hava hizi
    const char *parite[4]  = {"8N1", "8O1", "8E1", "8N1"};
    const uint32_t ubaud[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    const char *airr[8]    = {"0.3k", "1.2k", "2.4k", "4.8k", "9.6k", "19.2k", "19.2k", "19.2k"};
    snprintf(b, sizeof(b), "  UART parite  : %s\r\n", parite[(sped >> 6) & 0x03]); cift_bas(b);
    snprintf(b, sizeof(b), "  UART baud    : %lu bps\r\n", (unsigned long)ubaud[(sped >> 3) & 0x07]); cift_bas(b);
    snprintf(b, sizeof(b), "  Hava hizi    : %s bps\r\n", airr[sped & 0x07]); cift_bas(b);

    snprintf(b, sizeof(b), "  Kanal        : %u  -> %u MHz\r\n", chan, 410 + chan); cift_bas(b);

    // OPTION byte: [7]iletim modu [6]IO [5-3]uyanma [2]FEC [1-0]guc
    snprintf(b, sizeof(b), "  Iletim modu  : %s\r\n",
             (opt & 0x80) ? "SABIT (fixed; adres+kanal on-eki gerekir!)" : "SEFFAF (transparent)"); cift_bas(b);
    snprintf(b, sizeof(b), "  IO surme     : %s\r\n",
             (opt & 0x40) ? "push-pull / pull-up" : "acik-drain (pull-up gerekir)"); cift_bas(b);
    const uint16_t wu[8] = {250, 500, 750, 1000, 1250, 1500, 1750, 2000};
    snprintf(b, sizeof(b), "  Uyanma suresi: %u ms\r\n", wu[(opt >> 3) & 0x07]); cift_bas(b);
    snprintf(b, sizeof(b), "  FEC          : %s\r\n", (opt & 0x04) ? "ACIK" : "KAPALI"); cift_bas(b);
    const char *guc[4] = {"30 dBm (1W)", "27 dBm", "24 dBm", "21 dBm"};
    snprintf(b, sizeof(b), "  TX gucu      : %s\r\n", guc[opt & 0x03]); cift_bas(b);
  }

  // 2) Versiyon bilgisi (C3 C3 C3 -> 4 byte)
  while (Serial1.available()) Serial1.read();
  const uint8_t oku_ver[3] = {0xC3, 0xC3, 0xC3};
  Serial1.write(oku_ver, 3);
  Serial1.flush();
  delay(150);
  uint8_t v[4]; int m = 0;
  while (Serial1.available() && m < 4) v[m++] = Serial1.read();
  if (m >= 4) {
    snprintf(b, sizeof(b), "  Versiyon(C3) : %02X %02X %02X %02X  (frekans/model, surum, ozellik)\r\n",
             v[0], v[1], v[2], v[3]);
  } else {
    snprintf(b, sizeof(b), "  Versiyon(C3) : okunamadi (%d byte)\r\n", m);
  }
  cift_bas(b);
  cift_bas("==========================================\r\n");

  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);        // normal moda don
  delay(50);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);

  Serial1.begin(BAUD_LORA, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);  // (rx, tx)
  lora_konfigurasyon();       // adres=2, kanal=22 ayarla + normal moda gec
  lora_tum_ayarlari_bas();    // modulun TUM ayarlarini modulden okuyup dok (FEC, mod, guc...)

  Serial.println("\r\n--- LoRa IZOLE TEST BASLADI (sadece gonderim) ---");
  Serial.printf("LoRa: TX=%d RX=%d @%d  |  ADRES=%d  KANAL=%d (432 MHz)\r\n",
                PIN_LORA_TX, PIN_LORA_RX, BAUD_LORA,
                (LORA_ADDH << 8) | LORA_ADDL, LORA_CHAN);
}

void loop() {
  // Her saniye LoRa'dan bir mesaj bas
  if (millis() - son_gonderim >= 1000) {
    son_gonderim = millis();
    sayac++;

    char buf[96];
    snprintf(buf, sizeof(buf), "TRAKYA LORA TEST #%lu  t=%lu ms\r\n", sayac, millis());

    Serial1.print(buf);   // LoRa hattindan yayinla
    Serial.print(buf);    // USB'den de gor (calisiyor mu diye)
  }
}
