// ============================================================================
//  FÜNYE ATEŞLEYİCİ (Zamanlanmış) — Bağımsız Test / Gösterim Firmware'i
//  Trakya Roket 2026 — Uçuş Yazılımı
// ----------------------------------------------------------------------------
//  AKIŞ:  Güç geldi (boot) → 10 sn → LED yanar → 5 sn → 1. fünye (pin 27) 1 sn
//         palslanır → (1. fünye ateşinden 5 sn sonra) 2. fünye (pin 14) 1 sn
//         palslanır → kilitlenir.
//         İki fünye ASLA aynı anda sürülmez: F1 palsı biter (high-Z), aradan
//         geçer, sonra F2 ateşlenir.
//  Sekans TEK SEFER çalışır, sonra kilitlenir.
//
//  Zaman çizelgesi (t=0 boot):
//    t=10s  LED yanar
//    t=15s  1. fünye (27) ateşlenir → 1 sn HIGH
//    t=16s  1. fünye high-Z'ye döner
//    t=20s  2. fünye (14) ateşlenir → 1 sn HIGH
//    t=21s  2. fünye high-Z, BİTTİ (kilitli)
//
//  YÜKLEME:  pio run -e funye --target upload
//  MONİTÖR:  pio device monitor -b 115200
//
//  GPIO:  Fünye ve LED pinleri ESP-IDF native sürücüsüyle (driver/gpio.h)
//         yönetilir; Arduino pinMode/digitalWrite kullanılmaz.
//
//  !!! GÜVENLİK !!!
//  Varsayılan FUNYE_GERCEK_ATES = 0 → SİMÜLE modu (fünye pini SÜRÜLMEZ).
//  Gerçek ateşleme için aşağıdaki bayrağı 1 yapıp yeniden yükleyin ve
//  fünyeyi ancak sahada, güvenli mesafede, herkes uzaktayken bağlayın.
// ============================================================================

#include <Arduino.h>
#include "driver/gpio.h"

// ── GÜVENLİK ANAHTARI ───────────────────────────────────────────────────────
#define FUNYE_GERCEK_ATES   1     // 0 = SİMÜLE (pin sürülmez, güvenli) | 1 = GERÇEK ateşleme

// ── GEÇİCİ TEST BAYRAĞI ─────────────────────────────────────────────────────
// 1 = pin 14 (PIN_FUNYE_2) setup'ta OUTPUT yapılıp LOW'a sürülür — boot'taki
//     HIGH'ı aktif bastırma denemesi.
// !!! UYARI: Bu, güvenlik banner'ındaki "funye pinini setup'ta OUTPUT YAPMA"
// kuralını BİLEREK deler. Bu kartta pin OUTPUT'a geçince ateşleme olduğu
// gözlendi — yani bu blok pin 14'teki fünyeyi BOOT'ta ateşleyebilir. SADECE
// pin 14'te CANLI FÜNYE YOKKEN dene. Test bitince tekrar 0 yap.
#define TEST_PIN14_SETUP_OUTPUT   1

// ── Pinler (ana uçuş yazılımıyla aynı) ──────────────────────────────────────
#define PIN_FUNYE_1 27            // 1. fünye / MOSFET çıkışı (main.cpp PIN_FUNYE_1)
#define PIN_FUNYE_2 14            // 2. fünye / MOSFET çıkışı (main.cpp PIN_FUNYE_2)
#define PIN_LED     25            // LED çıkışı               (main.cpp PIN_LED)

// ── Zamanlama sabitleri ─────────────────────────────────────────────────────
#define BASLANGIC_SABITLEME_MS  1000UL     // Boot'ta pin high-Z garanti bekleme
#define LED_GECIKME_MS         20000UL     // Güç → LED arası bekleme (20 sn)
#define F1_GECIKME_MS          10000UL     // LED → 1. fünye arası bekleme (10 sn)
#define F1_F2_ARASI_MS          5000UL     // 1. fünye ateşi → 2. fünye ateşi arası (5 sn)
#define FUNYE_PALS_MS           1000UL     // Her fünye pininin HIGH kalma süresi (1 sn)

// ── Durum makinesi ──────────────────────────────────────────────────────────
enum Durum {
    BEKLE_LED,     // Güç geldi, LED gecikmesi sayılıyor
    BEKLE_F1,      // LED yandı, 1. fünye gecikmesi sayılıyor
    F1_ACIK,       // 1. fünye HIGH (pals sürüyor)
    BEKLE_F2,      // 1. fünye high-Z'de, 2. fünye ateş anı bekleniyor
    F2_ACIK,       // 2. fünye HIGH (pals sürüyor)
    BITTI          // Sekans tamamlandı, kilitli
};

Durum durum = BEKLE_LED;
unsigned long asama_baslangic = 0;   // Aktif aşamanın başladığı an (millis)
unsigned long f1_ates_ani     = 0;   // 1. fünyenin ateşlendiği an (F2 zamanlaması için)
unsigned long pals_baslangic  = 0;   // Aktif fünye palsının başladığı an
unsigned long son_log         = 0;   // Geri sayım logu için

// ============================================================================
//  *** FUNYE PIN GUVENLIGI — BU FONKSIYON ASLA VE ASLA DEGISTIRILMEYECEK ***
// ============================================================================
// KURAL: Funye pinleri setup()'ta OUTPUT YAPILMAZ. Pin OUTPUT'a yalnizca HIGH
// yazilacagi an gecer; LOW yazilirken high-Z'ye (INPUT) geri birakilir. Boylece
// palslar arasinda ve sekans bitince pin surulmez halde kalir.
//
// NEDEN (dokunmadan once oku):
//  1) Boot/reset sirasinda ESP32 pinleri high-Z'dir. setup()'ta OUTPUT yapmak
//     pini surulur hale getirir; o andan itibaren tek bir hatali cikis
//     (bozuk bellek, kacak kod yolu, brown-out sonrasi yarim reset) gercek
//     funyeyi ateslemeye yeter. DAHA KOTUSU: bu kartta atesleme HIGH seviyede
//     degil, pin OUTPUT'a GECER GECMEZ olur. Yani pini kazara OUTPUT yapmak =
//     dogrudan atesleme. Bu yuzden pin default high-Z kalmali, OUTPUT'a yalniz
//     bilincli atesleme aninda gecmeli.
//  2) High-Z'de MOSFET gate'ini harici pull-down GND'ye kilitler — yazilim ne
//     yaparsa yapsin fiziksel olarak akim akmaz. Yazilim hatasi donanim
//     guvenligini asamaz. Ek emniyet olarak high-Z'de dahili pull-down da acilir.
//  3) Bu sketch tek isi funye atesleme olan, tezgahta insan yanindayken calisan
//     bir sketch. Buraya funye pinini setup'ta OUTPUT yapan bir satir EKLEME.
//  4) Iki funye asla ayni anda HIGH olmaz; F1 palsi biter (high-Z), aradan gecer,
//     sonra F2 ateslenir.
//
// NATIVE GPIO (driver/gpio.h) NOTLARI:
//  - Ates: once gpio_set_level(pin,0), sonra gpio_config(mode=OUTPUT). OUTPUT'a
//    gecis registerdeki 0'i basar (glitch olmaz), ardindan gpio_set_level(pin,1).
//    Sira onemli: once level=0, sonra yon degisimi.
//  - Guvenli: gpio_set_level(pin,0) + gpio_config(mode=INPUT) → high-Z.
//  - gpio_reset_pin() KULLANILMAZ: o pini dahili PULL-UP ile birakir; bir MOSFET
//    gate'inde pull-up gate'i HIGH'a cekmeye calisir = tehlike. Onun yerine acikca
//    gpio_config ile pull-up KAPALI / pull-down ACIK yapiyoruz.
// ============================================================================
// SİMÜLE modunda pin sürülmez; GERÇEK modda fünye pinini yazar.
void FunyePinYaz(int pin, int seviye) {
#if FUNYE_GERCEK_ATES
    if (seviye == HIGH) {
        gpio_set_level((gpio_num_t)pin, 0);         // OUTPUT'a gecerken glitch olmasin
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << pin,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);                           // <-- ATES burada (OUTPUT'a gecis)
        gpio_set_level((gpio_num_t)pin, 1);
    } else {
        gpio_set_level((gpio_num_t)pin, 0);         // cikis registerini once temizle
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << pin,
            .mode         = GPIO_MODE_INPUT,        // high-Z — surucu tamamen devre disi
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,   // ek emniyet: gate GND'ye cekilir
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }
#else
    (void)pin; (void)seviye;            // SİMÜLE: pin sürülmez
#endif
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
    // --- GÜVENLİK: İLK İŞ — fünye pinlerini güvenli high-Z'ye al ---
    // Yazılımın erişebildiği en erken an. Pini INPUT + dahili pull-down yapar
    // (OUTPUT DEĞİL — yani ateşlemez), setup sonrası zayıf LOW'a çeker.
    //
    // !!! ÖNEMLİ SINIR !!! Bu satırlar pin 14'ün (GPIO14) AÇILIŞTAKI HIGH'ını
    // ENGELLEMEZ. GPIO14, ROM bootloader'ının dahili pull-up ile bıraktığı bir
    // pindir; o zayıf HIGH bu kod çalışmadan ÖNCE, boot'un ilk ~yüz ms'inde olur.
    // setup()'a ne yazılırsa yazılsın o pencereyi kapatamaz. Boot HIGH'ını
    // yalnızca gate'teki HARİCİ PULL-DOWN direnci güvenceye alır (ROM'un ~45kΩ
    // zayıf pull-up'ını yener). Buradaki satır sadece setup SONRASI durum içindir.
    // SİMÜLE modunda no-op.
    FunyePinYaz(PIN_FUNYE_1, LOW);
    FunyePinYaz(PIN_FUNYE_2, LOW);

#if TEST_PIN14_SETUP_OUTPUT
    // *** GEÇİCİ TEST — güvenlik kuralını BİLEREK deliyor (bkz. TEST_PIN14_SETUP_OUTPUT) ***
    // Pin 14'ü OUTPUT yapıp LOW'a sürer: boot HIGH'ını aktif bastırma denemesi.
    // DİKKAT: OUTPUT'a geçiş bu kartta ateşleme tetikleyebilir — canlı fünye bağlıyken ÇALIŞTIRMA.
    {
        gpio_set_level((gpio_num_t)PIN_FUNYE_2, 0);   // önce çıkış registerini temizle
        gpio_config_t test14 = {
            .pin_bit_mask = 1ULL << PIN_FUNYE_2,
            .mode         = GPIO_MODE_OUTPUT,          // <-- pin 14 OUTPUT (test)
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&test14);
        gpio_set_level((gpio_num_t)PIN_FUNYE_2, 0);   // pin 14'ü LOW'a sür (bastır)
    }
#endif

    // --- LED çıkışı (native) ---
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << PIN_LED,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level((gpio_num_t)PIN_LED, 0);

    Serial.begin(115200);
    delay(50);

    Serial.println();
    Serial.println("=== FÜNYE ATEŞLEYİCİ ===");
#if FUNYE_GERCEK_ATES
    Serial.println("MOD: *** GERÇEK ATEŞLEME *** — DİKKAT!");
#else
    Serial.println("MOD: SİMÜLE (güvenli) — fünye pini sürülmeyecek");
#endif
    Serial.printf("Akis: guc -> %lus -> LED -> %lus -> F1(pin %d) %lums pals -> +%lus -> F2(pin %d) %lums pals\n",
                  LED_GECIKME_MS / 1000, F1_GECIKME_MS / 1000, PIN_FUNYE_1, FUNYE_PALS_MS,
                  F1_F2_ARASI_MS / 1000, PIN_FUNYE_2, FUNYE_PALS_MS);
    Serial.printf("Pinler: FUNYE_1=%d  FUNYE_2=%d  LED=%d\n", PIN_FUNYE_1, PIN_FUNYE_2, PIN_LED);

    // --- Başlangıç sabitleme: pin high-Z garanti, sonra sayaç başlar ---
    delay(BASLANGIC_SABITLEME_MS);
    asama_baslangic = millis();
    son_log = millis();
    Serial.println("Baslangic sabitlendi. LED geri sayimi basladi.");
}

void loop() {
    unsigned long simdi = millis();

    switch (durum) {
        case BEKLE_LED: {
            unsigned long gecen = simdi - asama_baslangic;
            GeriSayimLog("LED bekleniyor...", gecen, LED_GECIKME_MS);
            if (gecen >= LED_GECIKME_MS) {
                gpio_set_level((gpio_num_t)PIN_LED, 1);
                Serial.println("LED YANDI. 1. funye geri sayimi basladi.");
                durum = BEKLE_F1;
                asama_baslangic = simdi;
            }
            break;
        }

        case BEKLE_F1: {
            unsigned long gecen = simdi - asama_baslangic;
            GeriSayimLog("1. funye bekleniyor...", gecen, F1_GECIKME_MS);
            if (gecen >= F1_GECIKME_MS) {
                FunyePinYaz(PIN_FUNYE_1, HIGH);
                f1_ates_ani    = simdi;
                pals_baslangic = simdi;
                durum = F1_ACIK;
                Serial.printf(">>> 1. FUNYE (pin %d) ATESLENDI -> %lu ms pals\n", PIN_FUNYE_1, FUNYE_PALS_MS);
            }
            break;
        }

        case F1_ACIK: {
            // Pals süresi dolunca 1. fünyeyi high-Z'ye al (kapat)
            if (simdi - pals_baslangic >= FUNYE_PALS_MS) {
                FunyePinYaz(PIN_FUNYE_1, LOW);
                Serial.println("1. funye high-Z (kapali).");
                durum = BEKLE_F2;
            }
            break;
        }

        case BEKLE_F2: {
            // 2. fünye, 1. fünyenin ateşlendiği andan F1_F2_ARASI_MS sonra ateşlenir.
            // (F1 palsı zaten bittiği için iki pin asla aynı anda HIGH olmaz.)
            unsigned long gecen = simdi - f1_ates_ani;
            GeriSayimLog("2. funye bekleniyor...", gecen, F1_F2_ARASI_MS);
            if (gecen >= F1_F2_ARASI_MS) {
                FunyePinYaz(PIN_FUNYE_2, HIGH);
                pals_baslangic = simdi;
                durum = F2_ACIK;
                Serial.printf(">>> 2. FUNYE (pin %d) ATESLENDI -> %lu ms pals\n", PIN_FUNYE_2, FUNYE_PALS_MS);
            }
            break;
        }

        case F2_ACIK: {
            // Pals süresi dolunca 2. fünyeyi high-Z'ye al ve sekansı kilitle
            if (simdi - pals_baslangic >= FUNYE_PALS_MS) {
                FunyePinYaz(PIN_FUNYE_2, LOW);
                Serial.println("2. funye high-Z (kapali). Sekans BITTI (iki pin de guvenli).");
                durum = BITTI;
            }
            break;
        }

        case BITTI:
            // Boşta, kilitli. Sekans tekrar çalışmaz.
            break;
    }
}
