// Gorev yuku saf LED karar mantigi host birim testi (framework yok).
// Calistir: g++ -std=c++11 GorevYukuYazilimi/host_led_durum_bgy.cpp -o /tmp/test_bgy && /tmp/test_bgy
#include <cassert>
#include <cstdio>
#include "led_durum_bgy.h"

static void esit(const char* ad, LedDurumBgy g, bool a, bool b, bool c, bool bc) {
    if (g.led1 != a || g.led2 != b || g.led3 != c || g.beacon != bc) {
        printf("FAIL %s: beklenen (%d,%d,%d,bc=%d) geldi (%d,%d,%d,bc=%d)\n",
               ad, a, b, c, bc, g.led1, g.led2, g.led3, g.beacon);
        assert(false);
    }
    printf("OK   %s\n", ad);
}

int main() {
    // --- CONFIG (sistem_hazir=false, indi=false) -> 3 LED 1 Hz blink, beacon kapali ---
    esit("config blink ON  @0",   hesapla_led_durumu_bgy(false, false, false, 0),   true,  true,  true,  false);
    esit("config blink ON  @499", hesapla_led_durumu_bgy(false, false, false, 499), true,  true,  true,  false);
    esit("config blink OFF @500", hesapla_led_durumu_bgy(false, false, false, 500), false, false, false, false);
    esit("config blink OFF @999", hesapla_led_durumu_bgy(false, false, false, 999), false, false, false, false);
    esit("config blink ON  @1000",hesapla_led_durumu_bgy(false, false, false, 1000),true,  true,  true,  false);

    // --- BEKLIYOR (sistem_hazir=true, uctu=false) -> 3 LED sabit, beacon kapali ---
    esit("bekliyor solid", hesapla_led_durumu_bgy(true, false, false, 1234), true, true, true, false);

    // --- UCUSTA (sistem_hazir=true, uctu=true, indi=false) -> hepsi kapali ---
    esit("ucusta off", hesapla_led_durumu_bgy(true, true, false, 1234), false, false, false, false);

    // --- INDI (kurtarma beacon) -> 4 cikis da buzzer ritminde senkron flash ---
    // BEACON_BIP_MS=200 acik, BEACON_PERIYOT_MS=1000
    esit("indi flash ON  @0",   hesapla_led_durumu_bgy(true, true, true, 0),   true,  true,  true,  true);
    esit("indi flash ON  @199", hesapla_led_durumu_bgy(true, true, true, 199), true,  true,  true,  true);
    esit("indi flash OFF @200", hesapla_led_durumu_bgy(true, true, true, 200), false, false, false, false);
    esit("indi flash OFF @999", hesapla_led_durumu_bgy(true, true, true, 999), false, false, false, false);
    esit("indi flash ON  @1000",hesapla_led_durumu_bgy(true, true, true, 1000),true,  true,  true,  true);

    // --- INDI onceligi: sistem_hazir=false olsa bile indi kazanir (beacon) ---
    esit("indi > config @0", hesapla_led_durumu_bgy(false, false, true, 0), true, true, true, true);

    printf("\nTUM TESTLER GECTI\n");
    return 0;
}
