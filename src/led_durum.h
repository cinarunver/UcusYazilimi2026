#ifndef LED_DURUM_H
#define LED_DURUM_H

// Ucus durum makinesi (main.cpp'den buraya tasindi; hem firmware hem host testi kullanir)
enum UcusDurumu {
    HAZIR      = 0, // Rampa uzerinde, kalkis bekleniyor
    YUKSELIYOR = 1, // Kalkis algilandi, yukselme fazi
    INIS_1     = 2, // Apogee gecildi, drogue parasut acildi
    INIS_2     = 3, // Alcak irtifa, ana parasut acildi
    INDI       = 4  // Yere inis tamamlandi, sistem pasif
};

// 3 gosterge LED'inin anlik yanma durumu
struct LedDurum { bool led1; bool led2; bool led3; };

// LED karar mantigi — hardware'siz, saf. Yalnizca mod + sistem_hazir + durum'a bagli.
//   normal_mod : true = MOD_BEKLEME (normal ucus), false = SIT/SUT
//   sistem_hazir: setup() bittiginde true; false iken config blink
//   now_ms      : millis() (blink fazi icin)
static inline LedDurum hesapla_led_durumu(bool normal_mod, bool sistem_hazir,
                                          UcusDurumu durum, unsigned long now_ms) {
    LedDurum d = {false, false, false};

    // SIT/SUT: config/HAZIR gostergesi yok. LED'ler durumdan turer (drogue/ana), led3 kapali.
    if (!normal_mod) {
        d.led1 = (durum >= INIS_1);
        d.led2 = (durum >= INIS_2);
        d.led3 = false;
        return d;
    }

    // Normal mod, config bitmedi -> 3'u birlikte 1 Hz blink
    if (!sistem_hazir) {
        bool acik = ((now_ms / 500) % 2 == 0);
        d.led1 = d.led2 = d.led3 = acik;
        return d;
    }

    // Normal mod, config bitti -> state machine
    switch (durum) {
        case HAZIR:      d.led1 = d.led2 = d.led3 = true;  break; // hepsi sabit
        case YUKSELIYOR: /* hepsi kapali */                break;
        case INIS_1:     d.led1 = true;                    break;
        case INIS_2:     d.led1 = true; d.led2 = true;     break;
        case INDI:       d.led1 = d.led2 = d.led3 = true;  break;
    }
    return d;
}

#endif // LED_DURUM_H
