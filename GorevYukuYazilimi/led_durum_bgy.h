#ifndef LED_DURUM_BGY_H
#define LED_DURUM_BGY_H

// Kurtarma beacon zamanlamasi. LED flash ile buzzer bip AYNI millis() fazindan
// beslenir -> dogal senkron. (gorevyuku.cpp beacon_guncelle() de bunlari kullanir.)
#define BEACON_BIP_MS       200   // ms - beacon "acik" suresi
#define BEACON_PERIYOT_MS  1000   // ms - beacon periyodu (BIP_MS acik + kalani sonuk)

// Gorev yukunun 4 gosterge cikisi: 3 durum LED'i (26/4/25) + kurtarma beacon LED'i (13)
struct LedDurumBgy { bool led1; bool led2; bool led3; bool beacon; };

// LED karar mantigi — hardware'siz, saf. Yalniz sistem_hazir + uctu + indi'ye bagli.
//   sistem_hazir: setup() bitince true; false iken 3 durum LED'i 1 Hz blink (config)
//   uctu        : irtifa/ivme esigini gecince true (ucusa gecti)
//   indi        : indikten sonra latch true -> kurtarma beacon (en oncelikli)
//   now_ms      : millis()  (blink/flash fazi icin)
static inline LedDurumBgy hesapla_led_durumu_bgy(bool sistem_hazir, bool uctu,
                                                 bool indi, unsigned long now_ms) {
    LedDurumBgy d = {false, false, false, false};

    // 1) Indi (en oncelikli): 4 cikis da buzzer ritminde senkron flash = kurtarma beacon
    if (indi) {
        bool flash = (now_ms % BEACON_PERIYOT_MS) < BEACON_BIP_MS;
        d.led1 = d.led2 = d.led3 = d.beacon = flash;
        return d;
    }

    // 2) Config bitmedi -> 3 durum LED'i 1 Hz blink (beacon kapali)
    if (!sistem_hazir) {
        bool acik = ((now_ms / 500) % 2 == 0);
        d.led1 = d.led2 = d.led3 = acik;
        return d;
    }

    // 3) Config bitti:
    //    - bekliyor (uctu=false) -> 3 LED sabit
    //    - ucusta   (uctu=true)  -> hepsi kapali (default)
    if (!uctu) {
        d.led1 = d.led2 = d.led3 = true;
    }
    return d;
}

#endif // LED_DURUM_BGY_H
