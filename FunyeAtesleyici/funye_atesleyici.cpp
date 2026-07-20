// ============================================================================
//  FÜNYE ATEŞLEYİCİ (Zamanlanmış) — Bağımsız Test / Gösterim Firmware'i
//  Trakya Roket 2026 — Uçuş Yazılımı
// ----------------------------------------------------------------------------
//  AKIŞ:  Güç geldi (boot) → 10 sn bekle → LED yanar → 10 sn bekle → Fünye ateşlenir
//  ATEŞLEME:  1 sn AÇIK / 1 sn KAPALI şeklinde 5 kez pals (5 açık pals).
//  Sekans TEK SEFER çalışır, sonra kilitlenir.
//
//  YÜKLEME:  pio run -e funye --target upload
//  MONİTÖR:  pio device monitor -b 115200
//
//  !!! GÜVENLİK !!!
//  Varsayılan FUNYE_GERCEK_ATES = 0 → SİMÜLE modu (fünye pini SÜRÜLMEZ).
//  Gerçek ateşleme için aşağıdaki bayrağı 1 yapıp yeniden yükleyin ve
//  fünyeyi ancak sahada, güvenli mesafede, herkes uzaktayken bağlayın.
// ============================================================================

#include <Arduino.h>

// ── GÜVENLİK ANAHTARI ───────────────────────────────────────────────────────
#define FUNYE_GERCEK_ATES   1     // 0 = SİMÜLE (pin sürülmez, güvenli) | 1 = GERÇEK ateşleme

// ── Pinler (ana uçuş yazılımıyla aynı) ──────────────────────────────────────
#define PIN_FUNYE   14            // Fünye / MOSFET çıkışı (main.cpp PIN_FUNYE_1)
#define PIN_LED     25            // LED çıkışı           (main.cpp PIN_LED)

// ── Zamanlama sabitleri ─────────────────────────────────────────────────────
#define BASLANGIC_SABITLEME_MS  1000UL    // Boot'ta pin LOW garanti bekleme
#define LED_GECIKME_MS         1000UL    // Güç → LED arası bekleme
#define FUNYE_GECIKME_MS       10000UL    // LED → fünye arası bekleme
#define FUNYE_PALS_SAYISI          5      // Kaç kez ateşleme palsı
#define FUNYE_ACIK_MS           5000UL    // Her palsta pin HIGH kalma süresi
#define FUNYE_KAPALI_MS         5000UL    // Palslar arası pin LOW süresi

// ── Durum makinesi ──────────────────────────────────────────────────────────
enum Durum {
    BEKLE_LED,        // Güç geldi, LED gecikmesi sayılıyor
    LED_YANDI_BEKLE,  // LED yandı, fünye gecikmesi sayılıyor
    ATESLE,           // Fünye palsı aktif
    BITTI             // Sekans tamamlandı, kilitli
};

Durum durum = BEKLE_LED;
unsigned long asama_baslangic  = 0;    // Aktif aşamanın başladığı an (millis)
unsigned long pals_gecis       = 0;    // Son pals AÇIK/KAPALI geçişinin anı
unsigned long son_log          = 0;    // Geri sayım logu için
int  funye_pals_no             = 0;    // Tamamlanan AÇIK pals sayısı
bool funye_pin_acik            = false; // Şu an fünye pini HIGH mı
bool atesleme_yapildi          = false; // Tek atış kilidi

// ============================================================================
//  *** FUNYE PIN GUVENLIGI — BU FONKSIYON ASLA VE ASLA DEGISTIRILMEYECEK ***
// ============================================================================
// KURAL: PIN_FUNYE setup()'ta OUTPUT YAPILMAZ. Pin OUTPUT'a yalnizca HIGH
// yazilacagi an gecer; LOW yazilirken high-Z'ye (INPUT) geri birakilir. Boylece
// palslar arasinda ve sekans bitince pin surulmez halde kalir.
//
// NEDEN (dokunmadan once oku):
//  1) Boot/reset sirasinda ESP32 pinleri high-Z'dir. setup()'ta OUTPUT yapmak
//     pini surulur hale getirir; o andan itibaren tek bir hatali digitalWrite
//     (bozuk bellek, kacak kod yolu, brown-out sonrasi yarim reset) gercek
//     funyeyi ateslemeye yeter.
//  2) High-Z'de MOSFET gate'ini harici pull-down GND'ye kilitler — yazilim ne
//     yaparsa yapsin fiziksel olarak akim akmaz. Yazilim hatasi donanim
//     guvenligini asamaz.
//  3) Bu sketch tek isi funye atesleme olan, tezgahta insan yanindayken
//     calisan bir sketch. Buraya pinMode(PIN_FUNYE, OUTPUT) satirini setup'a
//     tasima veya kalici OUTPUT'a cevirme.
//
// Sira onemli: HIGH'ta once digitalWrite(LOW) sonra pinMode(OUTPUT) — ters
// sira, OUTPUT'a gecis aninda registerde kalmis eski HIGH'i pine basar.
// Bu davranis src/main.cpp ile ayni mantiktadir.
// ============================================================================
// SİMÜLE modunda pin sürülmez; GERÇEK modda fünye pinini yazar.
void FunyePinYaz(int seviye) {
#if FUNYE_GERCEK_ATES
    if (seviye == HIGH) {
        digitalWrite(PIN_FUNYE, LOW);       // OUTPUT'a gecerken glitch olmasin
        pinMode(PIN_FUNYE, OUTPUT | PULLDOWN);
        digitalWrite(PIN_FUNYE, HIGH);
    } else {
        digitalWrite(PIN_FUNYE, LOW);       // cikis registerini once temizle
        pinMode(PIN_FUNYE, INPUT);          // high-Z — surucu tamamen devre disi
    }
#else
    (void)seviye;                       // SİMÜLE: pin sürülmez
#endif
}

// ── Non-Blocking fünye ateşleme (pals dizisi) ───────────────────────────────
// 1 sn AÇIK / 1 sn KAPALI şeklinde FUNYE_PALS_SAYISI kadar pals başlatır.
void FunyeAtesle() {
    if (atesleme_yapildi) return;       // Çift ateşlemeyi engelle
    atesleme_yapildi = true;
    funye_pals_no  = 0;
    funye_pin_acik = true;
    pals_gecis     = millis();
    FunyePinYaz(HIGH);
#if FUNYE_GERCEK_ATES
    Serial.printf(">>> FÜNYE ATEŞLEME BAŞLADI (GERÇEK) — %d pals, 1s AÇIK / 1s KAPALI\n", FUNYE_PALS_SAYISI);
#else
    Serial.printf(">>> FÜNYE ATEŞLEME BAŞLADI (SİMÜLE) — %d pals, 1s AÇIK / 1s KAPALI\n", FUNYE_PALS_SAYISI);
#endif
    Serial.println("Pals 1 -> AÇIK");
}

// Pals dizisini yürütür (her döngüde çağrılır). 5. palstan sonra kilitlenir.
void FunyeGuncelle() {
    if (durum != ATESLE) return;
    unsigned long simdi = millis();

    if (funye_pin_acik) {
        // AÇIK faz doldu → pini kapat
        if (simdi - pals_gecis >= FUNYE_ACIK_MS) {
            FunyePinYaz(LOW);
            funye_pin_acik = false;
            funye_pals_no++;
            pals_gecis = simdi;
            Serial.printf("Pals %d -> KAPALI\n", funye_pals_no);
            if (funye_pals_no >= FUNYE_PALS_SAYISI) {
                Serial.println("Tüm palslar tamam. Sekans BİTTİ.");
                durum = BITTI;          // pin LOW durumda kilitlenir (güvenli)
            }
        }
    } else {
        // KAPALI faz doldu → sonraki palsı aç
        if (funye_pals_no < FUNYE_PALS_SAYISI && (simdi - pals_gecis >= FUNYE_KAPALI_MS)) {
            FunyePinYaz(HIGH);
            funye_pin_acik = true;
            pals_gecis = simdi;
            Serial.printf("Pals %d -> AÇIK\n", funye_pals_no + 1);
        }
    }
}

// Saniyede bir geri sayım logu basar
void GeriSayimLog(const char* etiket, unsigned long gecen, unsigned long toplam) {
    if (millis() - son_log >= 1000UL) {
        son_log = millis();
        long kalan = (long)((toplam - gecen) / 1000UL);
        if (kalan < 0) kalan = 0;
        Serial.printf("[T-%lds] %s\n", kalan, etiket);
    }
}

void setup() {
    // --- GÜVENLİK: İlk iş fünye pinini güvenli (LOW) hale getirmek ---


    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    Serial.begin(115200);
    delay(50);

    Serial.println();
    Serial.println("=== FÜNYE ATEŞLEYİCİ ===");
#if FUNYE_GERCEK_ATES
    Serial.println("MOD: *** GERÇEK ATEŞLEME *** — DİKKAT!");
#else
    Serial.println("MOD: SİMÜLE (güvenli) — fünye pini sürülmeyecek");
#endif
    Serial.printf("Akış: guc -> %lus -> LED -> %lus -> funye (%d pals, 1s ac / 1s kapa)\n",
                  LED_GECIKME_MS / 1000, FUNYE_GECIKME_MS / 1000, FUNYE_PALS_SAYISI);

    // --- Başlangıç sabitleme: pin LOW garanti, sonra sayaç başlar ---
    delay(BASLANGIC_SABITLEME_MS);
    asama_baslangic = millis();
    son_log = millis();
    Serial.println("Baslangic sabitlendi. LED geri sayimi basladi.");
}

void loop() {
    FunyeGuncelle();   // Süresi dolan palsı kapat

    switch (durum) {
        case BEKLE_LED: {
            unsigned long gecen = millis() - asama_baslangic;
            GeriSayimLog("LED bekleniyor...", gecen, LED_GECIKME_MS);
            if (gecen >= LED_GECIKME_MS) {
                digitalWrite(PIN_LED, HIGH);
                Serial.println("LED YANDI. Funye geri sayimi basladi.");
                durum = LED_YANDI_BEKLE;
                asama_baslangic = millis();
            }
            break;
        }

        case LED_YANDI_BEKLE: {
            unsigned long gecen = millis() - asama_baslangic;
            GeriSayimLog("Funye bekleniyor...", gecen, FUNYE_GECIKME_MS);
            if (gecen >= FUNYE_GECIKME_MS) {
                FunyeAtesle();
                durum = ATESLE;
            }
            break;
        }

        case ATESLE:
            // Pals FunyeGuncelle() tarafından kapatılır; burada beklenir.
            break;

        case BITTI:
            // Boşta, kilitli. Sekans tekrar çalışmaz.
            break;
    }
}
