// Saf LED karar mantigi host birim testi (framework yok).
// Calistir: g++ -std=c++11 test/host_led_durum.cpp -o /tmp/test_led && /tmp/test_led
#include <cassert>
#include <cstdio>
#include "../src/led_durum.h"

static void esit(const char* ad, LedDurum g, bool a, bool b, bool c) {
    if (g.led1 != a || g.led2 != b || g.led3 != c) {
        printf("FAIL %s: beklenen (%d,%d,%d) geldi (%d,%d,%d)\n",
               ad, a, b, c, g.led1, g.led2, g.led3);
        assert(false);
    }
    printf("OK   %s\n", ad);
}

int main() {
    // --- NORMAL MOD, config bitmedi (sistem_hazir=false) -> 1 Hz blink ---
    // now=0..499 -> yanik (hepsi), now=500..999 -> sonuk
    esit("config blink ON  @0",   hesapla_led_durumu(true, false, HAZIR, 0),   true,  true,  true);
    esit("config blink ON  @499", hesapla_led_durumu(true, false, HAZIR, 499), true,  true,  true);
    esit("config blink OFF @500", hesapla_led_durumu(true, false, HAZIR, 500), false, false, false);
    esit("config blink OFF @999", hesapla_led_durumu(true, false, HAZIR, 999), false, false, false);
    esit("config blink ON  @1000",hesapla_led_durumu(true, false, HAZIR, 1000),true,  true,  true);

    // --- NORMAL MOD, config bitti (sistem_hazir=true) -> state machine ---
    esit("HAZIR solid",      hesapla_led_durumu(true, true, HAZIR,      1234), true,  true,  true);
    esit("YUKSELIYOR off",   hesapla_led_durumu(true, true, YUKSELIYOR, 1234), false, false, false);
    esit("INIS_1 -> LED1",   hesapla_led_durumu(true, true, INIS_1,     1234), true,  false, false);
    esit("INIS_2 -> LED1+2", hesapla_led_durumu(true, true, INIS_2,     1234), true,  true,  false);
    esit("INDI  -> LED1+2+3",hesapla_led_durumu(true, true, INDI,       1234), true,  true,  true);

    // --- SIT/SUT modu (normal_mod=false): config/HAZIR yok, drogue/ana durumdan ---
    esit("SUT HAZIR hepsi off",  hesapla_led_durumu(false, false, HAZIR,      0), false, false, false);
    esit("SUT YUKSELIYOR off",   hesapla_led_durumu(false, true,  YUKSELIYOR, 0), false, false, false);
    esit("SUT INIS_1 -> LED1",   hesapla_led_durumu(false, true,  INIS_1,     0), true,  false, false);
    esit("SUT INIS_2 -> LED1+2", hesapla_led_durumu(false, true,  INIS_2,     0), true,  true,  false);
    esit("SUT INDI led3 kapali", hesapla_led_durumu(false, true,  INDI,       0), true,  true,  false);

    printf("\nTUM TESTLER GECTI\n");
    return 0;
}
