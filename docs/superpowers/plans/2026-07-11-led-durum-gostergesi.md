# LED Durum Göstergesi Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 3 gösterge LED'ini (LED1/LED2/LED3), config aşamasında 1 Hz blink; HAZIR'da sabit; kalkışta sönme; drogue/ana/indi olaylarında state-machine'e göre sırayla yanma davranışıyla sürmek.

**Architecture:** LED karar mantığı saf (hardware'siz) bir fonksiyona çıkarılır → host'ta g++ ile birim test edilir. `src/main.cpp` bu fonksiyonu her Task1 döngüsünde ve setup'taki bekleme döngüsünde çağırarak LED'leri sürer. LED yazımı tek merkezde (`led_uygula`) toplanır; `Funye*Atesle` ve mod-geçiş bloğundaki dağınık LED yazımları kaldırılır.

**Tech Stack:** C++11, ESP32 Arduino (PlatformIO `env:ucus`), host birim testi düz g++ + assert (framework yok).

## Global Constraints

- LED pinleri sabit: `PIN_LED_1`=GPIO26 (LED1/drogue), `PIN_LED_2`=GPIO4 (LED2/ana), `PIN_LED_3`=GPIO25 (LED3/indi). `PIN_LED`=GPIO13 kapsam dışı.
- Karar mantığı yalnızca 3 girdiye bağlı: **mod** (normal vs SIT/SUT), **`sistem_hazir`**, **`durum`**. Başka hiçbir sinyal LED'i doğrudan sürmez.
- Blink non-blocking: `millis()` tabanlı, ekstra `delay` yok. Faz: `on = ((now_ms / 500) % 2 == 0)` → 1 Hz.
- SIT/SUT modunda config blink / HAZIR-solid **devreye girmez**; LED'ler `durum`'dan türeyen drogue/ana göstergesini korur (led3 kapalı).
- Latch ayrı bayrakla değil, `durum` sırasından türetilir (INIS_2 ⇒ LED1 zaten yanık).
- Fünye pin yazımları (`PIN_FUNYE_1/2`) `Funye*Atesle` içinde **kalır** — sadece LED yazımları taşınır.

---

### Task 1: Saf LED karar fonksiyonu + host birim testi

**Files:**
- Create: `src/led_durum.h`
- Test: `test/host_led_durum.cpp`

**Interfaces:**
- Consumes: (yok — ilk task)
- Produces:
  - `enum UcusDurumu { HAZIR=0, YUKSELIYOR=1, INIS_1=2, INIS_2=3, INDI=4 };`
  - `struct LedDurum { bool led1; bool led2; bool led3; };`
  - `LedDurum hesapla_led_durumu(bool normal_mod, bool sistem_hazir, UcusDurumu durum, unsigned long now_ms);`

- [ ] **Step 1: Host testini yaz (failing)**

Create `test/host_led_durum.cpp`:

```cpp
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
```

- [ ] **Step 2: Testi çalıştır, derleme hatasıyla başarısız olduğunu doğrula**

Run: `g++ -std=c++11 test/host_led_durum.cpp -o /tmp/test_led && /tmp/test_led`
Expected: FAIL — `fatal error: ../src/led_durum.h: No such file or directory` (henüz oluşturulmadı)

- [ ] **Step 3: `src/led_durum.h` başlığını oluştur**

Create `src/led_durum.h`:

```cpp
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
```

- [ ] **Step 4: Testi çalıştır, geçtiğini doğrula**

Run: `g++ -std=c++11 test/host_led_durum.cpp -o /tmp/test_led && /tmp/test_led`
Expected: PASS — her satır `OK ...`, son satır `TUM TESTLER GECTI`

- [ ] **Step 5: Commit**

```bash
git add src/led_durum.h test/host_led_durum.cpp
git commit -m "feat: saf LED karar fonksiyonu + host birim testi

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: `main.cpp` entegrasyonu (led_uygula, sistem_hazir, LED yazımlarını merkezleştirme)

**Files:**
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes (Task 1'den): `led_durum.h` → `enum UcusDurumu`, `struct LedDurum`, `hesapla_led_durumu(...)`
- Produces: `void led_uygula();` (firmware içi), `volatile bool sistem_hazir`

- [ ] **Step 1: Başlığı dahil et**

`src/main.cpp` içinde `#include "esp_heap_caps.h"` satırının hemen altına ekle:

```cpp
#include "led_durum.h"       // LED karar mantigi (enum UcusDurumu burada)
```

- [ ] **Step 2: main.cpp'deki `enum UcusDurumu` tanımını kaldır (artık header'da)**

`src/main.cpp` içindeki şu bloğu **sil** (yaklaşık satır 232-239):

```cpp
// Uçuş Durum Makinesi (State Machine)
enum UcusDurumu {
    HAZIR      = 0, // Rampa üzerinde, kalkış bekleniyor
    YUKSELIYOR = 1, // Kalkış algılandı, yükselme fazı
    INIS_1     = 2, // Apogee geçildi, drogue (küçük) paraşüt açıldı
    INIS_2     = 3, // Alçak irtifaya inildi, ana paraşüt açıldı
    INDI       = 4  // Yere iniş tamamlandı, sistem pasif
};
```

Hemen altındaki `UcusDurumu durum = HAZIR;` satırı **kalır** (silme). Yerine sadece bir yorum bırak:

```cpp
// Uçuş Durum Makinesi (enum UcusDurumu artik led_durum.h icinde)
UcusDurumu durum = HAZIR;
```

- [ ] **Step 3: `sistem_hazir` global bayrağını ekle**

`UcusDurumu durum = HAZIR;` satırının hemen altına ekle:

```cpp
// setup() tamamlaninca true olur; false iken LED'ler config blink yapar
volatile bool sistem_hazir = false;
```

- [ ] **Step 4: `led_uygula()` fonksiyonunu ekle**

`funye_guncelle()` fonksiyonunun kapanış `}` satırının hemen altına ekle:

```cpp
// --- LED GOSTERGE SURUCUSU (tek merkez) ---
// Karar mantigi led_durum.h'deki saf fonksiyonda; burada sadece pinlere yazilir.
void led_uygula() {
    bool normal_mod = (sitSutMod == MOD_BEKLEME);
    LedDurum d = hesapla_led_durumu(normal_mod, sistem_hazir, durum, millis());
    digitalWrite(PIN_LED_1, d.led1 ? HIGH : LOW);
    digitalWrite(PIN_LED_2, d.led2 ? HIGH : LOW);
    digitalWrite(PIN_LED_3, d.led3 ? HIGH : LOW);
}
```

- [ ] **Step 5: `Funye1Atesle` içindeki LED yazımını kaldır**

`src/main.cpp` `Funye1Atesle()` içinde şu iki satırı **sil**:

```cpp
        // Gorsel gosterge: drogue LED her modda yakilir (latch — Durdur'a kadar yanik)
        digitalWrite(PIN_LED_DROGUE, HIGH);
```

(Fonksiyonun kalanı — `PIN_FUNYE_1` yazımı, `funye1_baslangic`, `ayrilma1 = true` — aynen kalır.)

- [ ] **Step 6: `Funye2Atesle` içindeki LED yazımını kaldır**

`src/main.cpp` `Funye2Atesle()` içinde şu satırı **sil**:

```cpp
        digitalWrite(PIN_LED_ANA, HIGH);
```

(`PIN_FUNYE_2` yazımı ve kalanı aynen kalır.)

- [ ] **Step 7: Mod-geçiş bloğundaki LED yazımlarını kaldır**

`Task1code` içindeki mod-geçiş sıfırlama bloğunda şu iki satırı **sil**:

```cpp
        digitalWrite(PIN_LED_DROGUE, LOW);
        digitalWrite(PIN_LED_ANA, LOW);
```

(Aynı bloktaki `PIN_FUNYE_1/2` LOW yazımları ve state sıfırlamaları **kalır**. LED'leri artık her döngüde `led_uygula()` kuruyor.)

- [ ] **Step 8: Task1 döngüsünde `led_uygula()` çağır**

`Task1code` içinde `funye_guncelle();` çağrısının hemen altına ekle (uçuş algoritması switch'inden önce):

```cpp
    // --- LED GOSTERGELERINI GUNCELLE ---
    led_uygula();
```

- [ ] **Step 9: setup()'ta config blink için `led_uygula()` çağır**

`setup()` içindeki BNO055 kalibrasyon bekleme döngüsünde (`while (cal_sys < ...)`), `vTaskDelay(500 / portTICK_PERIOD_MS);` satırının hemen **üstüne** ekle:

```cpp
            led_uygula(); // config blink (sistem_hazir=false -> 1 Hz)
```

Bu, en uzun bekleme fazı (max ~15 sn) boyunca 3 LED'in 1 Hz yanıp sönmesini sağlar.

- [ ] **Step 10: setup() sonunda `sistem_hazir = true` yap**

`setup()` içinde `telemetryQueue = xQueueCreate(...)` bloğunun **hemen üstüne** (RTOS görevleri oluşturulmadan önce) ekle:

```cpp
    // Tum baslangic isleri bitti — LED'ler artik state machine'e gore (HAZIR = solid)
    sistem_hazir = true;
```

- [ ] **Step 11: Firmware'i derle (doğrulama)**

Run: `pio run -e ucus`
Expected: `SUCCESS` — derleme hatasız tamamlanır. (`pio` PATH'te yoksa: `~/.platformio/penv/bin/pio run -e ucus` veya VS Code PlatformIO "Build".)

- [ ] **Step 12: Host testini tekrar çalıştır (regresyon)**

Run: `g++ -std=c++11 test/host_led_durum.cpp -o /tmp/test_led && /tmp/test_led`
Expected: PASS — `TUM TESTLER GECTI` (header değişmedi, hâlâ geçmeli).

- [ ] **Step 13: Commit**

```bash
git add src/main.cpp
git commit -m "feat: LED gostergelerini tek merkezli led_uygula() ile state machine'e bagla

Config blink (sistem_hazir), HAZIR-solid, kalkista sonme, drogue/ana/indi
sirali yanma. Funye* ve mod-gecis blogundaki dagintik LED yazimlari kaldirildi.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Donanım / SUT Doğrulama (manuel — commit sonrası)

Firmware host'ta birim-testli ama LED davranışı ancak donanımda/SUT ile görülür. `pio run -e sitsut --target upload` sonrası SUT senaryosu besleyip gözlemle:

- Açılışta (kalibrasyon beklerken) 3 LED **1 Hz blink**.
- Setup bitince HAZIR'da 3 LED **sabit yanık**.
- SUT sentetik kalkış → 3 LED **söner** (YUKSELIYOR).
- Drogue emri → **LED1** yanar; ana emri → **LED1+LED2**; iniş → **LED1+LED2+LED3**.
- SIT/SUT'a geçiş ve DURDUR sonrası LED davranışı bozulmuyor (config blink SIT/SUT'ta yok).

## Notlar

- `PIN_LED_DROGUE` / `PIN_LED_ANA` define'ları (main.cpp) `setup()`'taki başlangıç LOW yazımında hâlâ kullanılıyor; kaldırılmadılar (kapsam dışı, zararsız).
- `test/host_led_durum.cpp` düz g++ ile çalışır; PlatformIO `pio test` runner'ı için tasarlanmadı (kendi `main()`'i var), doğrudan g++ ile koşulur.
