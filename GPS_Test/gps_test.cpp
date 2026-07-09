/*
================================================================================
  TRAKYA ROKET 2026 - GPS IZOLE TEST
================================================================================
  Amac: GPS ve Serial1(LoRa) pinlerinden cevap alinamiyor. Bu sketch GPS'i
  TEK BASINA test eder ve ayni veriyi HEM Serial1(LoRa) HEM de USB Serial'e basar.

  TESHIS MANTIGI (onemli):
    - USB Serial (115200) : "GPS calisiyor mu?" -> pio device monitor ile oku
    - Serial1   (LoRa)    : "Serial1/LoRa TX calisiyor mu?" -> LoRa alicisinda gor
    Boylece iki sorun BIRBIRINDEN AYRILIR.

    - ham_byte SAYACI en kritik ipucu:
        ham_byte = 0 kaliyorsa  -> GPS'ten ESP'ye HIC veri gelmiyor
                                   (GPS_RX pini yanlis / kablo ters / GPS beslenmiyor)
        ham_byte artiyor ama fix=YOK -> GPS CALISIYOR, sadece uydu kilidi yok
                                        (acik gokyuzu gerekir; pin sorunu DEGIL)

  DERLEME:  pio run -e gpstest --target upload
  MONITOR:  pio device monitor -b 115200
================================================================================
*/

#include <Arduino.h>
#include <TinyGPS++.h>

// ---------------------------------------------------------------------------
//  PINLER — cevap alamazsan ILK burayi degistir
//  main_debug.cpp'deki degerler. GPS'ten byte GELMIYORSA RX/TX'i TERS dene:
//     PIN_GPS_RX 16 / PIN_GPS_TX 17
// ---------------------------------------------------------------------------
#define PIN_GPS_RX   17   // ESP RX  <- GPS'in TX'ine baglanir  (GPS icin KRITIK olan pin)
#define PIN_GPS_TX   16   // ESP TX  -> GPS'in RX'ine baglanir  (cogu GPS icin onemsiz)
#define BAUD_GPS     9600 // GY-NEO-7M varsayilan

// LoRa (Serial1) — ayni veriyi buradan da yayinliyoruz
#define PIN_LORA_TX  33   // ESP TX -> LoRa DIN (modulun RXD'si)
#define PIN_LORA_RX  32   // ESP RX <- LoRa DOUT (modulun TXD'si)
#define BAUD_LORA    9600
#define LORA_M0      15
#define LORA_M1      2

// --- E32 ADRES / KANAL (ALICIYLA AYNI OLMALI) ---
// Elde: alici LoRa = KANAL 22, ADRES 2. Verici de ayni olmali ki haberlessin.
#define LORA_ADDH    0x00   // Adres yuksek byte
#define LORA_ADDL    0x02   // Adres dusuk byte  -> ADRES = 2
#define LORA_CHAN    0x16   // Kanal 22 (0x16)   -> frekans = 410 + 22 = 432 MHz
#define LORA_SPED    0x1C   // UART 9600 8N1 + hava hizi (degistirme)

// --- OPTION byte bilesenleri (E32-433T30D datasheet §7.5) ---
// OPTION = TX_MODE | IO_DRIVE | WOR_TIME | FEC | TX_POWER  (asagidan otomatik birlesir)
#define LORA_TX_MODE   0x00   // bit7  0x00=SEFFAF (transparent) | 0x80=SABIT (fixed)
#define LORA_IO_DRIVE  0x40   // bit6  0x40=push-pull/pull-up (varsayilan) | 0x00=open-collector
#define LORA_WOR_TIME  0x00   // bit5-3 uyanma(WOR) suresi: 0x00=250 0x08=500 0x10=750 0x18=1000
                              //                            0x20=1250 0x28=1500 0x30=1750 0x38=2000 ms
#define LORA_FEC       0x04   // bit2  0x04=FEC ACIK (varsayilan) | 0x00=FEC kapali
#define LORA_TX_POWER  0x00   // bit1-0 TX gucu: 0x00=30dBm(1W) 0x01=27 0x02=24 0x03=21 dBm

#define LORA_OPTION  (LORA_TX_MODE | LORA_IO_DRIVE | LORA_WOR_TIME | LORA_FEC | LORA_TX_POWER)  // = 0x44 (seffaf, FEC acik, 250ms, 30dBm)

TinyGPSPlus gps;

unsigned long ham_byte = 0;    // Serial2'den gelen toplam ham byte (en onemli sayac)
unsigned long son_ozet = 0;    // son ozet basim zamani

// --- E32 LoRa KONFIGURASYONU (adres/kanal aliciyla eslesir) ---
// M0=M1=HIGH -> config modu; 6 byte {0xC0,ADDH,ADDL,SPED,CHAN,OPTION} yazilir;
// sonra M0=M1=LOW -> normal/seffaf mod. Modul yaniti USB'ye basilir (dogrulama).
void lora_konfigurasyon() {
  static const uint8_t cfg[6] = {0xC0, LORA_ADDH, LORA_ADDL, LORA_SPED, LORA_CHAN, LORA_OPTION};

  digitalWrite(LORA_M0, HIGH);
  digitalWrite(LORA_M1, HIGH);       // config modu
  delay(50);
  while (Serial1.available()) Serial1.read();   // eski baytlari temizle

  Serial1.write(cfg, sizeof(cfg));
  Serial1.flush();                   // gonderim bitene kadar bekle
  delay(200);                        // modul isleme suresi

  // Modul yaniti (genelde ayni 6 byte geri doner) -> DOUT->ESP RX baglantisini de dogrular
  uint8_t resp[8]; int n = 0;
  while (Serial1.available() && n < (int)sizeof(resp)) resp[n++] = Serial1.read();
  Serial.print("LoRa config yaniti: ");
  for (int i = 0; i < n; i++) Serial.printf("%02X ", resp[i]);
  Serial.println(n ? "" : "(yanit yok - DOUT/RX pini yanlis olabilir, TX yine de calisir)");

  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);        // normal/seffaf mod
  delay(50);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);

  Serial1.begin(BAUD_LORA, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);  // (rx, tx)
  lora_konfigurasyon();   // adres=2, kanal=22 (aliciyla ayni) ayarla + normal moda gec

  Serial2.begin(BAUD_GPS,  SERIAL_8N1, PIN_GPS_RX,  PIN_GPS_TX);   // (rx, tx)
  delay(200);

  Serial.println("\r\n--- GPS IZOLE TEST BASLADI ---");
  Serial.printf("GPS : RX=%d TX=%d @%d   |   LoRa(Serial1): TX=%d RX=%d @%d\r\n",
                PIN_GPS_RX, PIN_GPS_TX, BAUD_GPS, PIN_LORA_TX, PIN_LORA_RX, BAUD_LORA);
  Serial.printf("LoRa ayar: ADRES=%d  KANAL=%d (432 MHz)  -> alici ile ayni olmali\r\n",
                (LORA_ADDH << 8) | LORA_ADDL, LORA_CHAN);
  Serial.println("Ipucu: ham_byte 0 kaliyorsa GPS pini/kablosu yanlis. Artiyorsa GPS OK.");
  Serial1.print("--- GPS TEST (LoRa hatti) ---\r\n");
}

void loop() {
  // 1) GPS ham baytlarini oku, say, parse et ve ham NMEA'yi USB'ye yansit
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    ham_byte++;
    gps.encode(c);
    Serial.write(c);   // ham NMEA: GPS konusuyorsa burada $GPGGA/$GPRMC gorursun
  }

  // 2) Her 1 saniyede ozet: HEM USB Serial HEM Serial1(LoRa)
  if (millis() - son_ozet >= 1000) {
    son_ozet = millis();

    char buf[180];
    snprintf(buf, sizeof(buf),
      "\r\n[OZET] ham_byte:%lu  fix:%s  uydu:%lu  lat:%.6f  lng:%.6f  "
      "alt:%.1f m  hdop:%.2f  UTC:%02d:%02d:%02d\r\n",
      ham_byte,
      gps.location.isValid() ? "VAR" : "YOK",
      gps.satellites.value(),
      gps.location.lat(), gps.location.lng(),
      gps.altitude.meters(), gps.hdop.hdop(),
      gps.time.hour(), gps.time.minute(), gps.time.second());

    Serial.print(buf);    // USB'den oku
    Serial1.print(buf);   // LoRa hattindan yayinla
  }
}
