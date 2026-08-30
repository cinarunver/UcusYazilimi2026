# F1: Uçuş Algoritması Header Çıkarımı — Uygulama Planı

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `src/main.cpp` içindeki uçuş karar mantığını (Kalman, eğim, dikey hız, durum makinesi, yedek tetik, SUT durum bitleri) Arduino bağımlılığı olmayan tek bir saf C++ başlığına taşımak; firmware ve MATLAB'in aynı dosyayı derlemesini mümkün kılmak.

**Architecture:** Yeni `UcusAlgoritmasi/ucus_algoritmasi.h` saf karar mantığını içerir; girdi ham sensör okuması, çıktı fünye **emri**. `main.cpp` bu başlığı `#include` eder, taşınan kodu siler ve emri donanıma çeviren ince bir çağrı yerine indirger. Fünye pinlerini süren kod `main.cpp`'de kalır. Eşikler `#define` yerine `UcusAyar` yapısında çalışma zamanı alanı olur; `#define`'lar varsayılan değer olarak korunur.

**Tech Stack:** C++17 (saf, kütüphanesiz), PlatformIO, Unity test çatısı, `platform = native` (Mac üzerinde donanımsız test), ESP32 Arduino framework (firmware tarafı).

**Spec:** `docs/superpowers/specs/2026-08-30-matlab-ucus-algoritmasi-dogrulama-design.md`

## Global Constraints

Bu kısıtlar her görev için geçerlidir:

- **Davranış değişmez.** Bu bir taşıma işlemidir, iyileştirme değil. Bir mantık hatası fark edilirse düzeltilmez — not edilir ve ayrı iş kalemi olarak raporlanır.
- **Fünye pin güvenlik bloğuna dokunulmaz.** `src/main.cpp:600-675` arasındaki `funye_pin_serbest`, `funye_pin_ates`, `Funye1Atesle`, `Funye2Atesle`, `funye_guncelle` tek karakter değişmez. Fünye pinleri `setup()`'ta OUTPUT yapılmaz kuralı yerinde kalır.
- **Saf modül fünye ateşlemez, emir üretir.** `ucus_algoritmasi.h` içinde `digitalWrite`, `pinMode`, `millis`, `micros`, `Serial` veya herhangi bir Arduino sembolü geçmez.
- **Eşik değerleri değişmez.** `KALKIS_IVME_ESIGI = 20.0`, `APOGEE_IRTIFA_FARKI = 10.0`, `MIN_DIKEY_HIZ = 3.0`, `MAX_EGLIM = 75.0`, `APOGEE_MIN_IRTIFA = AYRILMA2_MESAFE`, `AYRILMA2_MESAFE = 550.0`, `INIS_HIZ_ESIGI = 2.0`, `INIS_IRTIFA_ESIGI = 20.0`, Kalman irtifa `(1.5, 1.5, 0.1)`, Kalman ivme/gyro `(2.906, 9.982, 0.3884)`. Yalnız yerleri değişir.
- **Yorum blokları birebir taşınır.** Özellikle `APOGEE_MIN_IRTIFA`, `MAX_EGLIM`, `APOGEE_SERBEST_IVME`, yedek tetik ve "EULER ACILARI FILTRELENMEZ" gerekçeleri kodun değerli kısmıdır; kısaltılmaz.
- **Commit mesajlarına `Co-Authored-By` satırı eklenmez.**

---

## Dosya Yapısı

| Dosya | Sorumluluk |
| :--- | :--- |
| `UcusAlgoritmasi/ucus_algoritmasi.h` (yeni) | Saf uçuş karar mantığının tamamı. Firmware ve MATLAB ortak kaynağı. |
| `test/test_ucus_algo/test_algo.cpp` (yeni) | Saf modülün donanımsız birim testleri. `platform = native` ile Mac'te koşar. |
| `platformio.ini` (değişecek) | `[env:native]` ortamı eklenir. |
| `src/main.cpp` (değişecek) | Taşınan kod silinir, başlık include edilir, çağrı yeri yazılır. |
| `test/test_ucus/test_main.cpp` (değişecek) | Kopyalanmış sabitler ve mantık kopyaları başlıkla değiştirilir. |
| `SİT_SUT/sit_sut_test.py` (değişecek) | SUT koşumunu CSV'ye kaydeden `--kaydet` seçeneği eklenir. |
| `docs/superpowers/altin_referans/` (yeni) | Taşıma öncesi firmware'in SUT çıktısı (eşdeğerlik kanıtı). |

---

## Görev Sırası ve Gerekçesi

Görev 1 **ilk sırada olmak zorunda**: altın referans, `main.cpp` değiştirilmeden önce mevcut firmware'den yakalanmalıdır. Taşımadan sonra yakalanan bir referans hiçbir şey kanıtlamaz.

Görev 8 donanım gerektirir. Kart o an elde yoksa Görev 2-7 tamamlanabilir; native testler ara kapı görevi görür, Görev 1 ve 8 kart geldiğinde koşulur. Bu durumda Görev 6'nın commit'i "eşdeğerlik henüz doğrulanmadı" notuyla işaretlenir.

---

### Task 1: Altın Referans Yakalama (taşımadan ÖNCE)

**Files:**
- Modify: `SİT_SUT/sit_sut_test.py:92-141` (`run_sut_scenario`)
- Create: `docs/superpowers/altin_referans/sut_referans.csv`

**Interfaces:**
- Consumes: yok
- Produces: `docs/superpowers/altin_referans/sut_referans.csv` — `enjekte_irtifa,enjekte_roll,enjekte_pitch,enjekte_az,durum_bitleri` sütunlu CSV. Görev 8 bunu karşılaştırma tabanı olarak kullanır.

**Ön koşul:** ESP32 kartı USB'ye bağlı, `sitsut` firmware'i yüklü, `upload_port = /dev/cu.usbserial-0001` erişilebilir.

**Neden zaman değil enjekte değer:** SUT gerçek zamanlı seri iletişimdir; ham zaman damgası koşumdan koşuma oynar. Karşılaştırma **enjekte edilen örnek ile o örneğe karşılık dönen durum bitleri** çifti üzerinden yapılır — bu, enjekte edilen diziye göre deterministiktir.

- [ ] **Step 1: `run_sut_scenario`'ya kayıt parametresi ekle**

`SİT_SUT/sit_sut_test.py` içinde metot imzasını ve gövdesini şu şekilde değiştir:

```python
    def run_sut_scenario(self, kayit_yolu=None):
        """Tam sentetik ucus profili: kalkis -> apogee -> drogue -> ana parasut -> inis.

        kayit_yolu verilirse her enjekte edilen ornek ve o ornege karsilik
        okunan durum bitleri CSV'ye yazilir (altin referans / esdegerlik kanidi).
        """
        kayit = []
        # ... mevcut profil uretimi aynen kalir ...
```

Mevcut gövdedeki her `self.send_sut_data(...)` çağrısından hemen sonra şu satırları ekle:

```python
            bits = self.oku_durum_bitleri()      # Adim 2'de eklenecek
            if kayit_yolu is not None:
                kayit.append((alt, roll, pitch, az, bits))
```

Metodun sonuna:

```python
        if kayit_yolu is not None:
            import csv, os
            os.makedirs(os.path.dirname(kayit_yolu), exist_ok=True)
            with open(kayit_yolu, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["enjekte_irtifa", "enjekte_roll", "enjekte_pitch",
                            "enjekte_az", "durum_bitleri"])
                for satir in kayit:
                    w.writerow(["%.3f" % satir[0], "%.3f" % satir[1],
                                "%.3f" % satir[2], "%.3f" % satir[3],
                                "0x%02X" % satir[4]])
            print("Altin referans yazildi: %s (%d satir)" % (kayit_yolu, len(kayit)))
```

- [ ] **Step 2: Durum bitlerini okuyan yardımcıyı ekle**

`SITSUT_DURUM_BOYUT` 6 bayttır (`SITSUT_HEADER`, komut, durum_bitleri, ... , `SITSUT_FOOTER1`, `SITSUT_FOOTER2`). `SutTester` sınıfına ekle:

```python
    def oku_durum_bitleri(self, timeout_s=0.25):
        """Tek bir 6 baytlik Tablo 6 durum paketi okur, durum bitleri baytini
        dondurur. Paket gelmezse 0xFF dondurur (kayitta 'cevap yok' isareti)."""
        import time
        son = time.time() + timeout_s
        tampon = bytearray()
        while time.time() < son:
            if self.ser.in_waiting:
                tampon.extend(self.ser.read(self.ser.in_waiting))
                while len(tampon) >= 6:
                    if tampon[0] == 0xAA and tampon[4] == 0x0D and tampon[5] == 0x0A:
                        return tampon[2]
                    tampon.pop(0)
        return 0xFF
```

- [ ] **Step 3: Menüye referans yakalama seçeneği ekle**

`main()` içindeki menüye (`sit_sut_test.py:203` civarı) satır ekle:

```python
    print("5: SUT Senaryosu + ALTIN REFERANS kaydi (docs/superpowers/altin_referans/)")
```

ve seçim işleyicisine:

```python
        elif secim == "5":
            t.send_command(CMD_SUT_BASLAT)
            time.sleep(1.2)   # TEST_AKTIVASYON_GECIKME_MS + pay
            t.run_sut_scenario(kayit_yolu="docs/superpowers/altin_referans/sut_referans.csv")
            t.send_command(CMD_DURDUR)
```

- [ ] **Step 4: Mevcut firmware'i karta yükle ve referansı yakala**

```bash
pio run -e sitsut --target upload
python3 "SİT_SUT/sit_sut_test.py"      # menuden 5 sec
```

Beklenen: `Altin referans yazildi: docs/superpowers/altin_referans/sut_referans.csv (N satir)`

- [ ] **Step 5: Referansın anlamlı olduğunu doğrula**

```bash
head -3 docs/superpowers/altin_referans/sut_referans.csv
awk -F, 'NR>1 {print $5}' docs/superpowers/altin_referans/sut_referans.csv | sort -u
```

Beklenen: `durum_bitleri` sütununda **birden fazla farklı değer** görülmeli (0x00 ile başlayıp kalkış/alçalma/emir bitleri yandıkça artan değerler). Tek bir değer varsa (özellikle hep `0xFF`) senaryo karta ulaşmamıştır — devam etme, seri bağlantıyı ve `sitsut` firmware'inin yüklü olduğunu kontrol et.

- [ ] **Step 6: Commit**

```bash
git add "SİT_SUT/sit_sut_test.py" docs/superpowers/altin_referans/sut_referans.csv
git commit -m "SUT altin referans kaydi eklendi (tasima oncesi davranis)"
```

---

### Task 2: Native Test Ortamı + Başlık İskeleti

**Files:**
- Create: `UcusAlgoritmasi/ucus_algoritmasi.h`
- Create: `test/test_ucus_algo/test_algo.cpp`
- Modify: `platformio.ini` (dosya sonuna ekleme)

**Interfaces:**
- Consumes: yok
- Produces:
  - `enum UcusDurumu { HAZIR=0, YUKSELIYOR=1, INIS_1=2, INIS_2=3, INDI=4 }`
  - `class SimpleKalmanFilter` — `SimpleKalmanFilter()`, `SimpleKalmanFilter(float mea_e, float est_e, float q)`, `void ayarla(float mea_e, float est_e, float q)`, `float updateEstimate(float mea)`
  - Görev 3-5 bu başlığı genişletir; Görev 6 firmware'den, Görev 7 mevcut testten kullanır.

- [ ] **Step 1: Başarısız testi yaz**

`test/test_ucus_algo/test_algo.cpp`:

```cpp
// Saf ucus algoritmasi birim testleri — DONANIM GEREKTIRMEZ.
// Calistir: pio test -e native
#include <unity.h>
#include "ucus_algoritmasi.h"

void setUp(void) {}
void tearDown(void) {}

void test_durum_enum_degerleri(void) {
    TEST_ASSERT_EQUAL_INT(0, HAZIR);
    TEST_ASSERT_EQUAL_INT(1, YUKSELIYOR);
    TEST_ASSERT_EQUAL_INT(2, INIS_1);
    TEST_ASSERT_EQUAL_INT(3, INIS_2);
    TEST_ASSERT_EQUAL_INT(4, INDI);
}

// Ilk cagri olcumu oldugu gibi dondurur (first_run yolu).
void test_kalman_ilk_cagri(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.0f, kf.updateEstimate(42.0f));
}

// Varsayilan kurucu + ayarla(), parametreli kurucuyla ayni sonucu vermeli.
// UcusHal icinde filtreler dizi uyesi oldugu icin varsayilan kurucu SART.
void test_kalman_varsayilan_kurucu_ayarla_ile_ayni(void) {
    SimpleKalmanFilter a(2.906f, 9.982f, 0.3884f);
    SimpleKalmanFilter b;
    b.ayarla(2.906f, 9.982f, 0.3884f);
    for (int i = 0; i < 20; i++) {
        float m = 10.0f + (float)i;
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, a.updateEstimate(m), b.updateEstimate(m));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_durum_enum_degerleri);
    RUN_TEST(test_kalman_ilk_cagri);
    RUN_TEST(test_kalman_varsayilan_kurucu_ayarla_ile_ayni);
    return UNITY_END();
}
```

- [ ] **Step 2: Native ortamı `platformio.ini`'ye ekle**

Dosyanın **sonuna** ekle (mevcut `[env:esp32dev]` bloğuna dokunma — onun `test_filter = test_ucus`'ü bu yeni dizini almasını zaten engelliyor):

```ini
; ── Saf ucus algoritmasi birim testleri (Mac uzerinde, KART GEREKMEZ) ──
;   pio test -e native
[env:native]
platform    = native
test_filter = test_ucus_algo
build_flags = -std=gnu++17 -I UcusAlgoritmasi
```

- [ ] **Step 3: Testi koş, BAŞARISIZ olduğunu gör**

```bash
pio test -e native
```

Beklenen: derleme hatası — `fatal error: ucus_algoritmasi.h: No such file or directory`

- [ ] **Step 4: Başlığın ilk hâlini yaz**

`UcusAlgoritmasi/ucus_algoritmasi.h`:

```cpp
#ifndef UCUS_ALGORITMASI_H
#define UCUS_ALGORITMASI_H

// ============================================================================
//  TRAKYA ROKET 2026 — SAF UCUS ALGORITMASI
// ============================================================================
//  Bu baslik ARDUINO BAGIMSIZDIR. Icinde digitalWrite, pinMode, millis,
//  micros, Serial veya baska bir Arduino sembolu GECMEZ.
//
//  Ayni dosyayi IKI tuketici derler:
//    1) src/main.cpp        — ucan firmware
//    2) MatlabSim/mex/...   — MATLAB simulasyonu (bkz. F2)
//  Amac: test edilen kod ile ucan kodun FIZIKSEL OLARAK AYNI dosya olmasi.
//
//  KURAL: Bu modul FUNYE ATESLEMEZ, EMIR uretir. Pini suren tek yer
//  main.cpp icindeki funye_pin_ates()'tir.
// ============================================================================

#include <math.h>
#include <stdint.h>

// --- UCUS DURUM MAKINESI ---
enum UcusDurumu {
    HAZIR      = 0, // Rampa uzerinde, kalkis bekleniyor
    YUKSELIYOR = 1, // Kalkis algilandi, yukselme fazi
    INIS_1     = 2, // Apogee gecildi, drogue parasut acildi
    INIS_2     = 3, // Alcak irtifa, ana parasut acildi
    INDI       = 4  // Yere inis tamamlandi, sistem pasif
};

// --- KALMAN FILTRESI SINIFI ---
// main.cpp'den bire bir tasindi. Tek eklenti: varsayilan kurucu + ayarla().
// Gerekce: UcusHal icinde dizi uyesi olarak tutulabilmesi icin varsayilan
// kurucu gerekiyor; parametreler ucus_sifirla() icinde ayarla() ile verilir.
class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter()
      : err_measure(1.0f), err_estimate(1.0f), q(0.01f),
        last_estimate(0), kalman_gain(0), first_run(true) {}

    SimpleKalmanFilter(float mea_e, float est_e, float q_) :
      err_measure(mea_e), err_estimate(est_e), q(q_),
      last_estimate(0), kalman_gain(0), first_run(true) {}

    void ayarla(float mea_e, float est_e, float q_) {
      err_measure = mea_e; err_estimate = est_e; q = q_;
      last_estimate = 0; kalman_gain = 0; first_run = true;
    }

    float updateEstimate(float mea) {
      if (first_run) {
        last_estimate = mea;
        first_run = false;
      }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      float current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabsf(last_estimate - current_estimate) * q;
      last_estimate = current_estimate;
      return current_estimate;
    }
  private:
    float err_measure;
    float err_estimate;
    float q;
    float last_estimate;
    float kalman_gain;
    bool first_run;
};

#endif // UCUS_ALGORITMASI_H
```

> Not: `main.cpp` orijinalinde `fabs` kullanıyor (double taşmalı). Burada `fabsf` yazıldı — `test/test_ucus/test_main.cpp:96` zaten `fabsf` kullanıyor ve float argümanda ikisi aynı sonucu verir. Görev 7'de her iki testin de geçmesi bunu doğrular.

- [ ] **Step 5: Testi koş, GEÇTİĞİNİ gör**

```bash
pio test -e native
```

Beklenen: `3 Tests 0 Failures 0 Ignored — PASSED`

- [ ] **Step 6: Commit**

```bash
git add UcusAlgoritmasi/ucus_algoritmasi.h test/test_ucus_algo/test_algo.cpp platformio.ini
git commit -m "Saf ucus algoritmasi basligi ve native test ortami eklendi"
```

---

### Task 3: Ayar Yapısı ve Eğim (Tilt) Hesabı

**Files:**
- Modify: `UcusAlgoritmasi/ucus_algoritmasi.h`
- Modify: `test/test_ucus_algo/test_algo.cpp`
- Reference: `src/main.cpp:1055-1070` (taşınacak eğim hesabı), `src/main.cpp:276-435` (eşikler)

**Interfaces:**
- Consumes: Görev 2'nin `enum UcusDurumu`, `SimpleKalmanFilter`
- Produces:
  - `struct UcusAyar` — alanlar: `kalkis_ivme_esigi`, `apogee_irtifa_farki`, `min_dikey_hiz`, `max_eglim`, `apogee_min_irtifa`, `ayrilma2_mesafe`, `inis_hiz_esigi`, `inis_irtifa_esigi`, `motor_yanma_sure_us`, `durum_min_irtifa_esigi`, `durum_aci_esigi`, `durum_yatay_ivme_esigi`, `kf_irtifa_p[3]`, `kf_ivme_p[3]`, `kf_gyro_p[3]`, `apogee_aday`
  - `float ucus_egim_acisi(const float euler[3], const float kuat[4], bool sut_modu)` — dereceden dönüş, 0° = tam dik

- [ ] **Step 1: Başarısız testleri yaz**

`test/test_ucus_algo/test_algo.cpp` dosyasına ekle:

```cpp
// --- AYAR VARSAYILANLARI: main.cpp'deki #define degerleriyle BIREBIR olmali ---
void test_ayar_varsayilanlari(void) {
    UcusAyar a;
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  20.0f, a.kalkis_ivme_esigi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  10.0f, a.apogee_irtifa_farki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,   3.0f, a.min_dikey_hiz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  75.0f, a.max_eglim);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 550.0f, a.apogee_min_irtifa);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 550.0f, a.ayrilma2_mesafe);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,   2.0f, a.inis_hiz_esigi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  20.0f, a.inis_irtifa_esigi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 100.0f, a.durum_min_irtifa_esigi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  45.0f, a.durum_aci_esigi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  30.0f, a.durum_yatay_ivme_esigi);
    TEST_ASSERT_EQUAL_UINT32(3000000u, a.motor_yanma_sure_us);
    TEST_ASSERT_EQUAL_UINT8(0, a.apogee_aday);
}

// Tam dik roket: egim 0.
void test_eglim_dik_sifir(void) {
    float e[3] = {0,0,0};
    float q[4] = {1,0,0,0};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ucus_egim_acisi(e, q, false));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ucus_egim_acisi(e, q, true));
}

// SUT modunda egim EULER'den, donanim modunda KUATERNIYON'dan gelir.
// Ayni fiziksel yonelim icin iki yol AYNI degeri vermeli (main.cpp:1044-1054
// yorumundaki matematiksel esitlik).
void test_eglim_quat_euler_ile_ayni(void) {
    // roll = 30 derece, pitch = 0  ->  cos(egim) = cos(30)*cos(0)
    float e[3] = {30.0f, 0.0f, 0.0f};   // roll, pitch, yaw
    // Ayni donusun kuaterniyonu: x ekseni etrafinda 30 derece
    float yari = 15.0f * (float)M_PI / 180.0f;
    float q[4] = {cosf(yari), sinf(yari), 0.0f, 0.0f};  // qw,qx,qy,qz
    float sut  = ucus_egim_acisi(e, q, true);
    float donanim = ucus_egim_acisi(e, q, false);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, sut);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, sut, donanim);
}

// KRITIK: acos girdisi 1.0'i asarsa NaN doner ve apogee ASLA tetiklenmez.
// Kalman/sayisal gurultuye karsi kirpma sart (main.cpp:1069).
void test_eglim_nan_uretmiyor(void) {
    float e[3] = {0,0,0};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    // Kuaterniyon normu 1'in altina duserse 1-2*(qx^2+qy^2) > 1 olur
    float qbozuk[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    qbozuk[1] = -0.001f;   // kucuk negatif kare -> yine <=1, guvenli taraf
    TEST_ASSERT_FALSE(isnan(ucus_egim_acisi(e, q, false)));
    TEST_ASSERT_FALSE(isnan(ucus_egim_acisi(e, qbozuk, false)));
    // Asiri deger: cos_val hesabi -1'in altina dusse bile NaN olmamali
    float qasiri[4] = {0.0f, 1.0f, 1.0f, 0.0f};   // 1-2*(1+1) = -3
    TEST_ASSERT_FALSE(isnan(ucus_egim_acisi(e, qasiri, false)));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, ucus_egim_acisi(e, qasiri, false));
}

// Takla halindeki roket guvenlik kapisini kapatmali (egim > MAX_EGLIM).
void test_eglim_tumbling_buyuk_aci(void) {
    UcusAyar a;
    float e[3] = {100.0f, 0.0f, 0.0f};
    float q[4] = {1,0,0,0};
    TEST_ASSERT_TRUE(ucus_egim_acisi(e, q, true) > a.max_eglim);
}
```

`main()` içine ekle:

```cpp
    RUN_TEST(test_ayar_varsayilanlari);
    RUN_TEST(test_eglim_dik_sifir);
    RUN_TEST(test_eglim_quat_euler_ile_ayni);
    RUN_TEST(test_eglim_nan_uretmiyor);
    RUN_TEST(test_eglim_tumbling_buyuk_aci);
```

- [ ] **Step 2: Testi koş, BAŞARISIZ olduğunu gör**

```bash
pio test -e native
```

Beklenen: derleme hatası — `'UcusAyar' was not declared` ve `'ucus_egim_acisi' was not declared`

- [ ] **Step 3: Eşik varsayılanlarını, `UcusAyar`'ı ve eğim fonksiyonunu ekle**

`ucus_algoritmasi.h` içinde `SimpleKalmanFilter` sınıfından **sonra**, `#endif`'ten önce ekle:

```cpp
// ============================================================================
//  ESIK VARSAYILANLARI
// ============================================================================
//  Bu #define'lar artik dogrudan kullanilmaz; UcusAyar alanlarinin VARSAYILAN
//  degerleridir. Calisma zamaninda degistirilebilir olmalari sart: MATLAB
//  tarafi (F4) bu esikleri Monte Carlo ile supurup eniyileyecek. #define
//  kalsaydi supurulemezlerdi. Firmware varsayilani kullanir -> davranis ayni.
// ============================================================================
#define KALKIS_IVME_ESIGI     20.0f  // m/s^2 - Z ekseninde bu ivmenin ustu = kalkis (BNO055)
#define APOGEE_IRTIFA_FARKI   10.0f  // m     - Max irtifadan bu kadar dusunce apogee sayilir (BME280)
#define MIN_DIKEY_HIZ          3.0f  // m/s   - Dikey hiz bunun altina indiyse yukselis bitti (BME280)
#define MAX_EGLIM             75.0f  // derece - Bu acidan fazla egimde apogee sayilmaz (guvenlik)
#define AYRILMA2_MESAFE      550.0f  // m     - Bu irtifanin altinda ana parasut acilir
#define APOGEE_MIN_IRTIFA  AYRILMA2_MESAFE  // m - Bu irtifa asilmadan apogee ARANMAZ
#define INIS_HIZ_ESIGI         2.0f  // m/s   - Bu degerin alti = yerde sayilir
#define INIS_IRTIFA_ESIGI     20.0f  // m     - Bu irtifanin alti = yerde sayilir

// SUT durum bitleri (Tablo 5) esikleri
#define DURUM_MIN_IRTIFA_ESIGI  100.0f  // m      - Bit 2 esigi
#define DURUM_ACI_ESIGI          45.0f  // derece - Bit 3 esigi (aci kriteri)
#define DURUM_YATAY_IVME_ESIGI   30.0f  // m/s^2  - Bit 3 esigi (yatay ivme kriteri)
#define MOTOR_YANMA_SURE_US   3000000u  // us     - Bit 1: kalkistan sonra motor yanma onlem suresi

// Kriter C (serbest dusus imzasi) — TANIMLI AMA DEVREDE DEGIL.
// Deger tezgahta olculup VE terimi olarak baglanacak. Ayrintili gerekce icin
// bkz. spec §8.5 ve bu satirlarin main.cpp'deki orijinal yorum blogu.
#define APOGEE_SERBEST_IVME     9.81f
#define APOGEE_SERBEST_TOLERANS 2.5f

// ============================================================================
//  UCUS AYARLARI — calisma zamani parametreleri
// ============================================================================
struct UcusAyar {
    float kalkis_ivme_esigi      = KALKIS_IVME_ESIGI;
    float apogee_irtifa_farki    = APOGEE_IRTIFA_FARKI;
    float min_dikey_hiz          = MIN_DIKEY_HIZ;
    float max_eglim              = MAX_EGLIM;
    float apogee_min_irtifa      = APOGEE_MIN_IRTIFA;
    float ayrilma2_mesafe        = AYRILMA2_MESAFE;
    float inis_hiz_esigi         = INIS_HIZ_ESIGI;
    float inis_irtifa_esigi      = INIS_IRTIFA_ESIGI;

    float durum_min_irtifa_esigi = DURUM_MIN_IRTIFA_ESIGI;
    float durum_aci_esigi        = DURUM_ACI_ESIGI;
    float durum_yatay_ivme_esigi = DURUM_YATAY_IVME_ESIGI;
    uint32_t motor_yanma_sure_us = MOTOR_YANMA_SURE_US;

    float kf_irtifa_p[3] = {1.5f, 1.5f, 0.1f};          // (e_mea, e_est, q)
    float kf_ivme_p[3]   = {2.906f, 9.982f, 0.3884f};
    float kf_gyro_p[3]   = {2.906f, 9.982f, 0.3884f};

    // Apogee kriteri adayi (spec §8.5). 0 = A0 = mevcut ucan mantik.
    // Firmware DAIMA 0 kullanir; digerleri yalniz MATLAB karsilastirmasi icin.
    uint8_t apogee_aday = 0;
};

// ============================================================================
//  EGIM (TILT) ACISI — 0 derece = tam dik
// ============================================================================
//  DONANIM MODUNDA KUATERNIYONDAN hesaplanir, Euler'den DEGIL: Euler
//  acilarindaki sarma bu degeri bozup funye guvenlik kapisini yanlislikla
//  acabiliyordu. Kuaterniyon cevrimsel degildir, sarmaz, gimbal lock'a girmez.
//
//  Iki formul MATEMATIKSEL OLARAK AYNI buyuklugu verir — govde +Z ekseninin
//  dusey (dunya +Z) ile acisi:
//      cos(egim) = R[2][2] = cos(pitch)*cos(roll) = 1 - 2*(qx^2 + qy^2)
//  q ile -q ayni donusu temsil ettiginden isaret normalizasyonu bu ifadeyi
//  etkilemez (qx^2 + qy^2 degismez).
//
//  SUT'ta gercek kuaterniyon yoktur (kimlik enjekte edilir); egim, yer
//  istasyonunun gonderdigi sentetik roll/pitch'ten hesaplanir.
//
//  KIRPMA SART: Sayisal tasma acos'u tanimsiz yapar (NaN). NaN gelirse
//  (egim < max_eglim) karsilastirmasi DAIMA false doner ve APOGEE HIC
//  TETIKLENMEZ = PARASUT ACILMAZ. Bu yuzden cos_val [-1, 1] araligina kirpilir.
//
//  euler[3] : roll, pitch, yaw  [derece]
//  kuat[4]  : qw, qx, qy, qz
static inline float ucus_egim_acisi(const float euler[3], const float kuat[4],
                                    bool sut_modu) {
    const float DEG2RAD = 0.017453292519943295f;
    const float RAD2DEG = 57.29577951308232f;
    float cos_val;
    if (sut_modu) {
        cos_val = cosf(euler[1] * DEG2RAD) * cosf(euler[0] * DEG2RAD); // pitch * roll
    } else {
        cos_val = 1.0f - 2.0f * (kuat[1] * kuat[1] + kuat[2] * kuat[2]);
    }
    if (cos_val >  1.0f) cos_val =  1.0f;
    if (cos_val < -1.0f) cos_val = -1.0f;
    return acosf(cos_val) * RAD2DEG;
}
```

> Dikkat: `main.cpp:1064`'te sıra `cos(pitch) * cos(roll)` şeklindedir; çarpma değişmeli olduğundan sıra sonucu etkilemez, ancak burada aynı sıra korunmuştur. `euler` dizisinin indeks düzeni `{roll, pitch, yaw}`'dır — `main.cpp`'deki `roll`/`pitch` değişkenleriyle Görev 6'da bu düzende eşleştirilecektir.

- [ ] **Step 4: Testi koş, GEÇTİĞİNİ gör**

```bash
pio test -e native
```

Beklenen: `8 Tests 0 Failures 0 Ignored — PASSED`

- [ ] **Step 5: Commit**

```bash
git add UcusAlgoritmasi/ucus_algoritmasi.h test/test_ucus_algo/test_algo.cpp
git commit -m "Esik ayarlari ve egim acisi hesabi saf module tasindi"
```

---

### Task 4: Dikey Hız Türevi (dt girdili)

**Files:**
- Modify: `UcusAlgoritmasi/ucus_algoritmasi.h`
- Modify: `test/test_ucus_algo/test_algo.cpp`
- Reference: `src/main.cpp:687-716` (taşınacak `hesapla_dikey_hiz`)

**Interfaces:**
- Consumes: Görev 3'ün `UcusAyar`
- Produces:
  - `struct UcusHizHal { float onceki_irtifa; uint32_t onceki_t_us; float son_dikey_hiz; }`
  - `float ucus_dikey_hiz(UcusHizHal& h, float guncel_irtifa, uint32_t t_us)`

**Davranış korunumu — üç yol birebir taşınacak:**
1. İlk ölçüm (`onceki_t_us == 0`): durumu doldurur, `0.0` döner
2. `dt <= 0`: **eski hızı korur** (0 dönmez — bu detay `main.cpp:703`'te açıkça böyle)
3. Normal: `(irtifa farkı) / dt`, durumu günceller

- [ ] **Step 1: Başarısız testleri yaz**

`test/test_ucus_algo/test_algo.cpp` dosyasına ekle:

```cpp
// Ilk cagri: gecmis yok, 0 donmeli ve durum dolmali.
void test_dikey_hiz_ilk_cagri_sifir(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ucus_dikey_hiz(h, 100.0f, 1000000u));
    TEST_ASSERT_EQUAL_UINT32(1000000u, h.onceki_t_us);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 100.0f, h.onceki_irtifa);
}

// Yukselis: 100 m -> 150 m, 0.5 s -> +100 m/s
void test_dikey_hiz_yukselis(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    ucus_dikey_hiz(h, 100.0f, 1000000u);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, ucus_dikey_hiz(h, 150.0f, 1500000u));
}

// Inis: 150 m -> 100 m, 0.5 s -> -100 m/s
void test_dikey_hiz_inis(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    ucus_dikey_hiz(h, 150.0f, 1000000u);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, ucus_dikey_hiz(h, 100.0f, 1500000u));
}

// dt <= 0: ESKI HIZ KORUNUR (0 DONMEZ). main.cpp:703 bu sekilde davranir;
// 0 dondurmek apogee aninda sahte "hiz sifirlandi" isareti uretirdi.
void test_dikey_hiz_dt_sifir_eski_hizi_korur(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    ucus_dikey_hiz(h, 100.0f, 1000000u);
    float v = ucus_dikey_hiz(h, 150.0f, 1500000u);   // +100 m/s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, v);
    // Ayni zaman damgasiyla tekrar cagir -> dt = 0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, v, ucus_dikey_hiz(h, 900.0f, 1500000u));
}
```

`main()` içine ekle:

```cpp
    RUN_TEST(test_dikey_hiz_ilk_cagri_sifir);
    RUN_TEST(test_dikey_hiz_yukselis);
    RUN_TEST(test_dikey_hiz_inis);
    RUN_TEST(test_dikey_hiz_dt_sifir_eski_hizi_korur);
```

- [ ] **Step 2: Testi koş, BAŞARISIZ olduğunu gör**

```bash
pio test -e native
```

Beklenen: derleme hatası — `'UcusHizHal' was not declared`

- [ ] **Step 3: Uygula**

`ucus_algoritmasi.h` içinde `ucus_egim_acisi`'ndan **sonra** ekle:

```cpp
// ============================================================================
//  ANLIK DIKEY HIZ (Vz) — mikrosaniye hassas turev
// ============================================================================
//      Vz = (Guncel Irtifa - Onceki Irtifa) / dt        dt = t_us farki [s]
//
//  Zaman DISARIDAN verilir (main.cpp micros(), MATLAB simulasyon saati).
//  Modul icinde micros() cagrilmaz — bu, modulun Arduino'suz derlenebilmesinin
//  ve MATLAB'de deterministik kosturulabilmesinin sarti.
struct UcusHizHal {
    float    onceki_irtifa;
    uint32_t onceki_t_us;
    float    son_dikey_hiz;
};

static inline float ucus_dikey_hiz(UcusHizHal& h, float guncel_irtifa, uint32_t t_us) {
    // Ilk olcum kontrolu
    if (h.onceki_t_us == 0u) {
        h.onceki_t_us   = t_us;
        h.onceki_irtifa = guncel_irtifa;
        return 0.0f;
    }

    float delta_t = (float)(t_us - h.onceki_t_us) / 1000000.0f;

    // Bolme hatasi (divide-by-zero) korumasi.
    // ESKI HIZI KORUR, 0 DONMEZ: apogee civarinda sahte "hiz sifirlandi"
    // isareti uretmemek icin. main.cpp:703 ile ayni davranis.
    if (delta_t <= 0.0f) {
        return h.son_dikey_hiz;
    }

    float hiz_z = (guncel_irtifa - h.onceki_irtifa) / delta_t;

    h.onceki_t_us   = t_us;
    h.onceki_irtifa = guncel_irtifa;
    h.son_dikey_hiz = hiz_z;
    return hiz_z;
}
```

- [ ] **Step 4: Testi koş, GEÇTİĞİNİ gör**

```bash
pio test -e native
```

Beklenen: `12 Tests 0 Failures 0 Ignored — PASSED`

- [ ] **Step 5: Commit**

```bash
git add UcusAlgoritmasi/ucus_algoritmasi.h test/test_ucus_algo/test_algo.cpp
git commit -m "Dikey hiz turevi saf module tasindi (dt disaridan)"
```

---

### Task 5: `ucus_adim()` — Durum Makinesi, Yedek Tetik, Durum Bitleri

**Files:**
- Modify: `UcusAlgoritmasi/ucus_algoritmasi.h`
- Modify: `test/test_ucus_algo/test_algo.cpp`
- Reference: `src/main.cpp:1087-1225` (taşınacak blok)

**Interfaces:**
- Consumes: Görev 2-4'ün tamamı (`UcusDurumu`, `SimpleKalmanFilter`, `UcusAyar`, `ucus_egim_acisi`, `UcusHizHal`, `ucus_dikey_hiz`)
- Produces:
  - `struct UcusGirdi`, `struct UcusHal`, `struct UcusCikti` (alanlar aşağıda)
  - `void ucus_sifirla(UcusHal& h, const UcusAyar& a)`
  - `void ucus_adim(UcusHal& h, const UcusGirdi& g, UcusCikti& c)`
  - Görev 6 (firmware) ve Görev 7 (mevcut test) bunları kullanır; F2'deki MEX köprüsü de bu imzayı hedefler.

**Durum biti sabitleri** (`main.cpp:243-250`'den taşınır): `ST_BIT_KALKIS (1u<<0)`, `ST_BIT_MOTOR_YANMA (1u<<1)`, `ST_BIT_MIN_IRTIFA (1u<<2)`, `ST_BIT_ACI_ESIK (1u<<3)`, `ST_BIT_ALCALMA (1u<<4)`, `ST_BIT_DROGUE_EMIR (1u<<5)`, `ST_BIT_ANA_IRTIFA (1u<<6)`, `ST_BIT_ANA_EMIR (1u<<7)`

- [ ] **Step 1: Başarısız testleri yaz**

`test/test_ucus_algo/test_algo.cpp` dosyasına ekle. Bu testler `test/test_ucus/test_main.cpp:416-510`'daki senaryoların **gerçek koda bağlanmış** hâlidir; oradaki gerekçe yorumları korunmuştur.

```cpp
// --- ucus_adim() TEST YARDIMCILARI ---

// Tek bir adim kosar. euler {roll,pitch,yaw}, kuat {qw,qx,qy,qz}.
static UcusCikti adim(UcusHal& h, float irtifa, float az, float egim_derece,
                      uint32_t t_us) {
    UcusGirdi g;
    g.ham_irtifa  = irtifa;
    g.ham_ivme[0] = 0.0f; g.ham_ivme[1] = 0.0f; g.ham_ivme[2] = az;
    g.ham_euler[0] = egim_derece; g.ham_euler[1] = 0.0f; g.ham_euler[2] = 0.0f;
    g.ham_kuat[0] = 1.0f; g.ham_kuat[1] = 0.0f;
    g.ham_kuat[2] = 0.0f; g.ham_kuat[3] = 0.0f;
    g.t_us     = t_us;
    g.sut_modu = true;          // egim EULER'den -> test kontrolu kolay
    g.filtrele = true;          // gercek ucus yolu: filtre zinciri devrede
    UcusCikti c;
    ucus_adim(h, g, c);
    return c;
}

// Kalman gecikmesini atlayarak irtifayi hedefe oturtur. Kalman ilk cagrida
// olcumu oldugu gibi alir, sonraki cagrilarda yakinsar; testlerin esik
// gecislerini olcebilmesi icin once yakinsama yaptirilir.
static UcusCikti oturt(UcusHal& h, float irtifa, float az, float egim,
                       uint32_t& t_us, int adet = 40) {
    UcusCikti c;
    for (int i = 0; i < adet; i++) {
        t_us += 10000u;                       // 100 Hz
        c = adim(h, irtifa, az, egim, t_us);
    }
    return c;
}

/* Kalkisi garantileyen yardimci.

   DIKKAT — Kalman zayiflatmasi: ivme filtresi (2.906, 9.982, 0.3884) bir
   basamak girdiyi TEK adimda gecirmez. Filtre sifirdayken 25 m/s^2 enjekte
   edilirse kestirim 25 degil ~10.9 cikar; 20 m/s^2'lik kalkis esigi ASILMAZ
   ve durum makinesi HAZIR'da kalir. (Ilk cagri istisnadir: first_run yolu
   olcumu oldugu gibi alir.)

   Bu yuzden testlerde kalkis TEK bir adimla yapilmaz. Burada hem esikten
   belirgin yuksek bir ivme (40) hem de birkac adim kullanilir. */
static void kalkis_yap(UcusHal& h, uint32_t& t_us) {
    for (int i = 0; i < 10; i++) {
        t_us += 10000u;
        adim(h, 0.0f, 40.0f, 0.0f, t_us);
    }
}

// Kalkis tespiti: Z ekseninde esik ustu ivme -> YUKSELIYOR
void test_sm_kalkis(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    UcusCikti c = adim(h, 0.0f, 0.0f, 0.0f, t);
    TEST_ASSERT_EQUAL_INT(HAZIR, c.durum);
    kalkis_yap(h, t);
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, h.durum);
}

// REGRESYON (Kalman zayiflatmasi): tek adimlik 25 m/s^2 kalkis URETMEZ.
// Bu davranis kasitlidir — filtre motor titresimindeki tek ornekli
// sicramalarin kalkis sanilmasini engeller. Test bunu kayda gecirir.
void test_sm_kalkis_tek_adim_esigi_gecmez(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    adim(h, 0.0f, 0.0f, 0.0f, t);             // first_run'i 0 ile tuket
    t += 10000u;
    TEST_ASSERT_EQUAL_INT(HAZIR, adim(h, 0.0f, 25.0f, 0.0f, t).durum);
}

// Apogee tam kosul: T && A && B && D saglanir -> drogue EMRI
void test_sm_apogee_tam_kosul(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 600.0f, 0.0f, 5.0f, t);          // tirmanis, max_irtifa = 600
    UcusCikti c = oturt(h, 580.0f, 0.0f, 5.0f, t);   // 20 m dustu
    TEST_ASSERT_EQUAL_INT(INIS_1, c.durum);
    TEST_ASSERT_TRUE(h.ayrilma1);
}

// Egim kapisi: 100 derece = burun fiilen asagi donmus -> ATESLEME YOK.
// 45 derece ARTIK ENGELLENMIYOR (esik 75).
void test_sm_apogee_egim_engeller(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 600.0f, 0.0f, 100.0f, t);
    UcusCikti c = oturt(h, 580.0f, 0.0f, 100.0f, t);
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, c.durum);   // apogee sayilmadi
    TEST_ASSERT_FALSE(h.ayrilma1);

    UcusHal h2; ucus_sifirla(h2, a);
    uint32_t t2 = 1000000u;
    kalkis_yap(h2, t2);
    oturt(h2, 600.0f, 0.0f, 45.0f, t2);
    UcusCikti c2 = oturt(h2, 580.0f, 0.0f, 45.0f, t2);
    TEST_ASSERT_EQUAL_INT(INIS_1, c2.durum);      // 45 derece gecer
}

/* REGRESYON: RAMPADA ATESLEME.
   max_irtifa 0.0'dan baslar ve kalkista irtifaya esitlenmez. Basinc referansi
   setup()'ta olculdugu icin sonradan basinc yukselirse irtifa NEGATIF okur.
   O zaman A saglanir (0 - (-20) = 20 > 10), hiz padde ~0 oldugu icin B de
   saglanir, roket rampada dik oldugu icin D de saglanir — drogue RAMPADA
   acilirdi. Arama tabani (T) bunu keser: max_irtifa o anda ~0'dir. */
void test_sm_apogee_rampada_negatif_irtifa_atesleme_yok(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);                         // kalkis (ama irtifa yok)
    UcusCikti c = oturt(h, -20.0f, 0.0f, 0.0f, t);
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, c.durum);
    TEST_ASSERT_FALSE(h.ayrilma1);
}

// Arama tabani siniri: 550 gecilmeden apogee ARANMAZ.
void test_sm_apogee_arama_tabani_siniri(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 540.0f, 0.0f, 5.0f, t);          // taban ALTINDA tepe
    UcusCikti c = oturt(h, 500.0f, 0.0f, 5.0f, t);
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, c.durum);   // apogee aranmadi
    TEST_ASSERT_FALSE(h.ayrilma1);
}

// INIS_1 -> INIS_2: ana parasut irtifasi
void test_sm_inis1_ana_parasut(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 700.0f, 0.0f, 5.0f, t);
    oturt(h, 650.0f, 0.0f, 5.0f, t);          // drogue acildi -> INIS_1
    TEST_ASSERT_TRUE(h.ayrilma1);
    UcusCikti c = oturt(h, 400.0f, 0.0f, 5.0f, t);
    TEST_ASSERT_EQUAL_INT(INIS_2, c.durum);
    TEST_ASSERT_TRUE(h.ayrilma2);
}

/* ANA PARASUT YEDEK TETIGI — durum makinesinden BAGIMSIZ.
   Apogee kacirilirsa INIS_1'e hic girilmez; tek bir apogee kacirmasi IKI
   parasutu birden dusururdu. Yedek tetik ana parasutu yine acar. */
void test_sm_yedek_ana_apogee_kacti(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    // Egim kapisini surekli kapali tut -> apogee HIC yakalanmaz
    oturt(h, 700.0f, 0.0f, 120.0f, t);
    UcusCikti c = oturt(h, 400.0f, 0.0f, 120.0f, t);
    TEST_ASSERT_FALSE(h.ayrilma1);            // drogue hic acilmadi
    TEST_ASSERT_TRUE(h.ayrilma2);             // ama ana parasut acildi
    TEST_ASSERT_EQUAL_INT(INIS_2, c.durum);
}

// Esigi YUKARI gecerken max_irtifa da irtifayla birlikte artar -> tetiklenmez.
void test_sm_yedek_ana_yukselirken_atesleme_yok(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    // 300 -> 600 m tirmanis: her adimda irtifa == max_irtifa
    for (float irt = 300.0f; irt <= 600.0f; irt += 10.0f) {
        t += 10000u;
        adim(h, irt, 0.0f, 120.0f, t);
    }
    TEST_ASSERT_FALSE(h.ayrilma2);
}

// Rampada: kalkis olmadan (HAZIR) tetiklenmemeli.
void test_sm_yedek_ana_rampada_atesleme_yok(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    oturt(h, 0.0f, 0.0f, 0.0f, t);
    TEST_ASSERT_FALSE(h.ayrilma2);
}

// INIS_2 -> INDI: hiz ~0 ve cok alcak
void test_sm_inis2_yere_inis(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 700.0f, 0.0f, 5.0f, t);
    oturt(h, 650.0f, 0.0f, 5.0f, t);
    oturt(h, 400.0f, 0.0f, 5.0f, t);          // INIS_2
    UcusCikti c = oturt(h, 10.0f, 0.0f, 5.0f, t);
    TEST_ASSERT_EQUAL_INT(INDI, c.durum);
}

// EMIR KENARI: funye emri yalniz BIR adim boyunca true olmali. Seviye olsaydi
// main.cpp her dongude Funye1Atesle() cagirir, funye_guncelle() 400 ms sonra
// pini birakinca funye YENIDEN enerjilenirdi.
void test_funye_emri_kenar_tek_sefer(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 600.0f, 0.0f, 5.0f, t);
    int emir_sayisi = 0;
    for (int i = 0; i < 60; i++) {
        t += 10000u;
        UcusCikti c = adim(h, 580.0f, 0.0f, 5.0f, t);
        if (c.funye1_emir) emir_sayisi++;
    }
    TEST_ASSERT_EQUAL_INT(1, emir_sayisi);
}

/* DURUM BITLERI GOZLEM/EMIR AYRIMI (Tablo 5).
   Gozlem bitleri (0,2,3,4,6) durum makinesine BAGLANMAZ: apogee kapisi
   (max_eglim) bloke olsa bile roket fiilen alcaliyorsa bit4 yanmali. Aksi
   halde tek bir kapi 4 biti birden dusurur ve yer istasyonu "alcaliyor ama
   emir yok" teshisini goremez. */
void test_durum_bitleri_gozlem_emirden_bagimsiz(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    kalkis_yap(h, t);
    oturt(h, 700.0f, 0.0f, 120.0f, t);        // egim kapisi KAPALI
    UcusCikti c = oturt(h, 650.0f, 0.0f, 120.0f, t);
    TEST_ASSERT_TRUE (c.durum_bitleri & ST_BIT_ALCALMA);       // gozlem yandi
    TEST_ASSERT_FALSE(c.durum_bitleri & ST_BIT_DROGUE_EMIR);   // emir yanmadi
}

// Rampada bit6 yanmamali: irtifa 0 < 550 ama max_irtifa esigi gecmedi.
void test_durum_biti_ana_irtifa_rampada_yanmaz(void) {
    UcusAyar a; UcusHal h; ucus_sifirla(h, a);
    uint32_t t = 1000000u;
    UcusCikti c = oturt(h, 0.0f, 0.0f, 0.0f, t);
    TEST_ASSERT_FALSE(c.durum_bitleri & ST_BIT_ANA_IRTIFA);
}
```

`main()` içine ekle:

```cpp
    RUN_TEST(test_sm_kalkis);
    RUN_TEST(test_sm_kalkis_tek_adim_esigi_gecmez);
    RUN_TEST(test_sm_apogee_tam_kosul);
    RUN_TEST(test_sm_apogee_egim_engeller);
    RUN_TEST(test_sm_apogee_rampada_negatif_irtifa_atesleme_yok);
    RUN_TEST(test_sm_apogee_arama_tabani_siniri);
    RUN_TEST(test_sm_inis1_ana_parasut);
    RUN_TEST(test_sm_yedek_ana_apogee_kacti);
    RUN_TEST(test_sm_yedek_ana_yukselirken_atesleme_yok);
    RUN_TEST(test_sm_yedek_ana_rampada_atesleme_yok);
    RUN_TEST(test_sm_inis2_yere_inis);
    RUN_TEST(test_funye_emri_kenar_tek_sefer);
    RUN_TEST(test_durum_bitleri_gozlem_emirden_bagimsiz);
    RUN_TEST(test_durum_biti_ana_irtifa_rampada_yanmaz);
```

- [ ] **Step 2: Testi koş, BAŞARISIZ olduğunu gör**

```bash
pio test -e native
```

Beklenen: derleme hatası — `'UcusGirdi' was not declared`, `'ucus_adim' was not declared`

- [ ] **Step 3: Uygula**

`ucus_algoritmasi.h` içinde `ucus_dikey_hiz`'dan **sonra** ekle:

```cpp
// ============================================================================
//  SUT DURUM BITLERI (Tablo 5)
// ============================================================================
//  Tablo 5 IKI tur bit tanimlar; ikisini KARISTIRMA:
//    GOZLEM (bit 0,2,3,4,6) = roketin fiziksel olarak ne yaptigi.
//    EMIR   (bit 5,7)       = UKB'nin funye karari.  (bit1 = zamanlayici)
#define ST_BIT_KALKIS        (1u << 0)
#define ST_BIT_MOTOR_YANMA   (1u << 1)
#define ST_BIT_MIN_IRTIFA    (1u << 2)
#define ST_BIT_ACI_ESIK      (1u << 3)
#define ST_BIT_ALCALMA       (1u << 4)
#define ST_BIT_DROGUE_EMIR   (1u << 5)
#define ST_BIT_ANA_IRTIFA    (1u << 6)
#define ST_BIT_ANA_EMIR      (1u << 7)

// ============================================================================
//  GIRDI / DURUM / CIKTI
// ============================================================================
struct UcusGirdi {
    float    ham_irtifa;      // BME280, yer referansina gore [m] — FILTRESIZ
    float    ham_ivme[3];     // BNO055 lineer ivme x,y,z [m/s^2] — FILTRESIZ
    float    ham_euler[3];    // roll, pitch, yaw [derece] — FILTRELENMEZ (cevrimsel)
    float    ham_kuat[4];     // qw, qx, qy, qz — FILTRELENMEZ (birim kisiti)
    uint32_t t_us;            // main.cpp'de micros(), MATLAB'de simulasyon saati
    bool     sut_modu;        // true: egim euler'den, false: kuaterniyondan
    bool     filtrele;        // false: ham degerler DOGRUDAN kullanilir (SUT enjeksiyonu
                              // bugun de filtresiz calisir — bkz. Gorev 6 Adim 4 notu)
};

struct UcusHal {
    UcusAyar ayar;

    SimpleKalmanFilter kf_ivme[3];
    SimpleKalmanFilter kf_gyro[3];
    SimpleKalmanFilter kf_irtifa;
    UcusHizHal hiz;

    UcusDurumu durum;
    float      max_irtifa;
    bool       ayrilma1;
    bool       ayrilma2;
    uint32_t   kalkis_t_us;
    uint8_t    durum_bitleri;
};

struct UcusCikti {
    UcusDurumu durum;
    bool       funye1_emir;    // SEVIYE DEGIL KENAR — yalniz bir adim true
    bool       funye2_emir;
    uint8_t    durum_bitleri;
    float      irtifa;         // filtrelenmis — telemetri + SD log
    float      dikey_hiz;
    float      eglim_acisi;
    float      max_irtifa;
    float      ivme[3];        // filtrelenmis — telemetri + SD log
};

// ============================================================================
static inline void ucus_sifirla(UcusHal& h, const UcusAyar& a) {
    h.ayar = a;
    for (int i = 0; i < 3; i++) {
        h.kf_ivme[i].ayarla(a.kf_ivme_p[0], a.kf_ivme_p[1], a.kf_ivme_p[2]);
        h.kf_gyro[i].ayarla(a.kf_gyro_p[0], a.kf_gyro_p[1], a.kf_gyro_p[2]);
    }
    h.kf_irtifa.ayarla(a.kf_irtifa_p[0], a.kf_irtifa_p[1], a.kf_irtifa_p[2]);
    h.hiz.onceki_irtifa = 0.0f;
    h.hiz.onceki_t_us   = 0u;
    h.hiz.son_dikey_hiz = 0.0f;
    h.durum         = HAZIR;
    h.max_irtifa    = 0.0f;
    h.ayrilma1      = false;
    h.ayrilma2      = false;
    h.kalkis_t_us   = 0u;
    h.durum_bitleri = 0u;
}

// ============================================================================
//  TEK ADIM — ham sensor girdisinden funye EMRINE
// ============================================================================
static inline void ucus_adim(UcusHal& h, const UcusGirdi& g, UcusCikti& c) {
    const UcusAyar& a = h.ayar;

    c.funye1_emir = false;
    c.funye2_emir = false;

    // --- 1) FILTRELEME ---
    float ivme[3];
    float irtifa;
    if (g.filtrele) {
        for (int i = 0; i < 3; i++) ivme[i] = h.kf_ivme[i].updateEstimate(g.ham_ivme[i]);
        irtifa = h.kf_irtifa.updateEstimate(g.ham_irtifa);
    } else {
        // SUT: veri zaten yer istasyonundan sentetik geliyor, filtre uygulanmaz.
        // Bu, tasima oncesi main.cpp davranisinin BIREBIR korunmasidir.
        for (int i = 0; i < 3; i++) ivme[i] = g.ham_ivme[i];
        irtifa = g.ham_irtifa;
    }

    // --- 2) TURETILMIS BUYUKLUKLER ---
    float eglim_acisi = ucus_egim_acisi(g.ham_euler, g.ham_kuat, g.sut_modu);
    float dikey_hiz   = ucus_dikey_hiz(h.hiz, irtifa, g.t_us);

    // --- 3) UCUS ALGORITMASI ---
    switch (h.durum) {
        case HAZIR:
            // Kalkis tespiti: Z ekseninde yeterli ivme -> YUKSELIYOR
            // [ TODO ] BNO055 Z-ekseni yonu dogrulanmali!
            if (ivme[2] > a.kalkis_ivme_esigi) {
                h.durum       = YUKSELIYOR;
                h.kalkis_t_us = g.t_us;   // motor yanma onlem suresi (bit 1) referansi
            }
            break;

        case YUKSELIYOR:
            // Max irtifa guncelle (sadece yukselis fazinda)
            if (irtifa > h.max_irtifa) h.max_irtifa = irtifa;

            // ================================================================
            // APOGEE TESPITI:  T && A && B && D
            // ================================================================
            // [SENSOR 1 - BME280 Barometrik - 2 Kosul]
            //   Kriter A: Irtifa max degerden apogee_irtifa_farki kadar dustu
            //   Kriter B: Dikey hiz min_dikey_hiz altinda (yukselis bitti)
            //
            // [SENSOR 2 - BNO055 IMU]  >>> KRITER C: EKLENECEK, HENUZ YOK <<<
            //   Ozet: konvansiyon olculmeden VE olarak baglanamaz (drogue hic
            //   acilmayabilir), VEYA olarak da baglanmamali (baro yukseliste
            //   donarsa erken atesleme uretir). Once olcum, sonra VE.
            //   Aday karsilastirmasi icin bkz. spec §8.5 (UcusAyar::apogee_aday).
            //
            // [GUVENLIK KAPISI - BNO055]
            //   Kriter D: Egim acisi < max_eglim (burun asagi donmemis).
            //   Bu apogee algilamasi DEGIL; yanlis pozisyonda atesleme engeli.
            //
            // [ARAMA TABANI - BME280]
            //   Kriter T: max_irtifa, apogee_min_irtifa'yi gecmis olmali. Bu
            //   taban asilmadan apogee HIC aranmaz — rampada ve alcak irtifada
            //   yanlis atesleme kapisi.
            //
            // Bu kapilar SUT'ta da AYNEN gecerlidir — mod-kosullu istisna yoktur.
            // ================================================================
            if ((h.max_irtifa > a.apogee_min_irtifa) &&                 // T
                (h.max_irtifa - irtifa > a.apogee_irtifa_farki) &&      // A
                (dikey_hiz < a.min_dikey_hiz) &&                        // B
                (eglim_acisi < a.max_eglim)) {                          // D
                if (!h.ayrilma1) { c.funye1_emir = true; h.ayrilma1 = true; }
                h.durum = INIS_1;
            }
            break;

        case INIS_1:
            // Alcak irtifaya inildiginde ana parasutu ac -> 2. Ayrilma
            if ((irtifa < a.ayrilma2_mesafe) && (h.max_irtifa > a.ayrilma2_mesafe)) {
                if (!h.ayrilma2) { c.funye2_emir = true; h.ayrilma2 = true; }
                h.durum = INIS_2;
            }
            break;

        case INIS_2:
            // Yere inis tespiti: hiz sifira yakin + cok alcakta
            if ((dikey_hiz > -a.inis_hiz_esigi) && (irtifa < a.inis_irtifa_esigi)) {
                h.durum = INDI;
            }
            break;

        case INDI:
            break;   // Sistem pasif
    }

    // ================================================================
    // ANA PARASUT YEDEK TETIGI — DURUM MAKINESINDEN BAGIMSIZ
    // ================================================================
    // Sorun: ana parasut YALNIZ `case INIS_1` icinde acilir, oraya da yalnizca
    // apogee yakalanirsa girilir. Yani TEK bir apogee kacirmasi IKI parasutu
    // birden dusurur — roket burun asagi iner.
    //
    // Cozum: ana parasut karari GOZLEME dayansin, duruma degil. Kosul zaten
    // kendi kendini yukselise karsi korur: irtifa esigi yukari gecerken
    // max_irtifa da onunla birlikte artar, dolayisiyla iki sart AYNI ANDA
    // ancak esik BIR KEZ asilip sonra altina inildiginde saglanir.
    // Bu bir zamanlayici DEGILDIR; hicbir sart saate bakmaz.
    //
    // MANDAL ayrilma2'dir: funye surme suresi bittiginde temizlenen bayrak
    // kullanilsaydi bu blok her 400 ms'de funyeyi yeniden enerjilendirirdi.
    //
    // SINIRI: drogue hic acilmadiysa roket bu irtifada cok hizlidir ve ana
    // parasut acilista yirtilabilir. Yine de kurtarma sansi verir.
    if (!h.ayrilma2 &&
        (h.durum >= YUKSELIYOR) &&
        (h.max_irtifa > a.ayrilma2_mesafe) &&
        (irtifa < a.ayrilma2_mesafe) &&
        (dikey_hiz < a.min_dikey_hiz)) {
        c.funye2_emir = true;
        h.ayrilma2    = true;
        h.durum       = INIS_2;   // artik ana parasut altinda: inis tespiti devreye girsin
    }

    // --- SUT DURUM BITLERI (Tablo 5, latch) ---
    if (h.durum >= YUKSELIYOR)                          h.durum_bitleri |= ST_BIT_KALKIS;
    if ((h.durum_bitleri & ST_BIT_KALKIS) &&
        (g.t_us - h.kalkis_t_us >= a.motor_yanma_sure_us))
                                                        h.durum_bitleri |= ST_BIT_MOTOR_YANMA;
    if (irtifa > a.durum_min_irtifa_esigi)               h.durum_bitleri |= ST_BIT_MIN_IRTIFA;
    // bit3: govde acisi VEYA yatay eksen ivmesi esigi (Tablo 5 iki kriterli)
    if ((eglim_acisi > a.durum_aci_esigi) ||
        (sqrtf(ivme[0]*ivme[0] + ivme[1]*ivme[1]) > a.durum_yatay_ivme_esigi))
                                                        h.durum_bitleri |= ST_BIT_ACI_ESIK;
    // bit4: GOZLEM — tepe gecildi, irtifa dusuyor (funye kararindan bagimsiz)
    if ((h.durum_bitleri & ST_BIT_KALKIS) &&
        (h.max_irtifa - irtifa > a.apogee_irtifa_farki)) h.durum_bitleri |= ST_BIT_ALCALMA;
    if (h.ayrilma1)                                     h.durum_bitleri |= ST_BIT_DROGUE_EMIR;
    // bit6: GOZLEM — belirlenen irtifanin altina indi. max_irtifa kapisi,
    // rampada (irtifa=0 < esik) bitin yanmasini onler.
    if ((h.max_irtifa > a.ayrilma2_mesafe) &&
        (irtifa < a.ayrilma2_mesafe))                   h.durum_bitleri |= ST_BIT_ANA_IRTIFA;
    if (h.ayrilma2)                                     h.durum_bitleri |= ST_BIT_ANA_EMIR;

    // --- CIKTI ---
    c.durum         = h.durum;
    c.durum_bitleri = h.durum_bitleri;
    c.irtifa        = irtifa;
    c.dikey_hiz     = dikey_hiz;
    c.eglim_acisi   = eglim_acisi;
    c.max_irtifa    = h.max_irtifa;
    for (int i = 0; i < 3; i++) c.ivme[i] = ivme[i];
}
```

> **Taşımada bilinçli iki uyarlama** (ikisi de davranışı korur, gerekçeleri kayda geçer):
> 1. `main.cpp`'de `Funye1Atesle()` içindeki `if (!funye1_aktif)` koruması ateşlemeyi tekilleştiriyordu. Saf modül `funye1_aktif`'i görmediği için tekilleştirme `h.ayrilma1` mandalıyla yapılır — `ayrilma1` yalnızca mod geçişinde sıfırlanır, `funye1_aktif` gibi 400 ms sonra temizlenmez, dolayısıyla daha güçlü bir mandaldır. Emrin kenar olması `test_funye_emri_kenar_tek_sefer` ile bağlanmıştır.
> 2. `main.cpp`'de gyro filtreleri `kf_gyro*` ile ayrı ayrı uygulanır ama **uçuş kararında kullanılmaz** (yalnız telemetri/SD). `UcusHal` içinde tutulurlar ki filtre durumu tek yerde olsun; `ucus_adim` onları çağırmaz. Firmware gyro filtrelemesini Görev 6'da kendi tarafında sürdürür.

- [ ] **Step 4: Testi koş, GEÇTİĞİNİ gör**

```bash
pio test -e native
```

Beklenen: `26 Tests 0 Failures 0 Ignored — PASSED`

Bir test başarısız olursa: bu bir **taşıma hatasıdır**, testi gevşetme. `src/main.cpp:1087-1225` ile satır satır karşılaştır ve farkı bul.

- [ ] **Step 5: Commit**

```bash
git add UcusAlgoritmasi/ucus_algoritmasi.h test/test_ucus_algo/test_algo.cpp
git commit -m "Durum makinesi, yedek tetik ve durum bitleri saf module tasindi"
```

---

### Task 6: `main.cpp`'yi Saf Modüle Bağla

**Files:**
- Modify: `src/main.cpp` (satır 17-22, 276-435, 448-485, 687-716, 1055-1225)
- Modify: `platformio.ini` (`[common]` bloğuna `build_flags`)

**Interfaces:**
- Consumes: Görev 5'in tamamı (`UcusAyar`, `UcusHal`, `UcusGirdi`, `UcusCikti`, `ucus_sifirla`, `ucus_adim`)
- Produces: `ucus_hal` adlı global `UcusHal`; `durum`, `durum_bitleri`, `ayrilma1`, `ayrilma2`, `max_irtifa_degeri` artık ondan türer

- [ ] **Step 1: Include yolunu `[common]`'a ekle**

`platformio.ini` içindeki `[common]` bloğuna ekle:

```ini
build_flags = -I UcusAlgoritmasi
```

ve `[env:ucus]`, `[env:ucusdebug]`, `[env:sitsut]`, `[env:esp32dev]` bloklarının her birine satırı ekle:

```ini
build_flags   = ${common.build_flags}
```

- [ ] **Step 2: `main.cpp`'de başlığı include et ve taşınan tanımları sil**

`src/main.cpp:11` civarındaki son `#include`'dan sonra ekle:

```cpp
#include "ucus_algoritmasi.h"   // saf ucus karar mantigi (firmware + MATLAB ortak)
```

Ardından şu blokları **sil** (artık başlıktan geliyorlar):

- `enum UcusDurumu { ... };` (17-22)
- `#define KALKIS_IVME_ESIGI`, `APOGEE_IRTIFA_FARKI`, `MIN_DIKEY_HIZ`, `MAX_EGLIM`, `AYRILMA2_MESAFE`, `APOGEE_MIN_IRTIFA`, `INIS_HIZ_ESIGI`, `INIS_IRTIFA_ESIGI`, `APOGEE_SERBEST_IVME`, `APOGEE_SERBEST_TOLERANS`, `DURUM_MIN_IRTIFA_ESIGI`, `DURUM_ACI_ESIGI`, `DURUM_YATAY_IVME_ESIGI`, `MOTOR_YANMA_SURE_MS`, `ST_BIT_*` satırları
- `class SimpleKalmanFilter { ... };` (448-472)
- `SimpleKalmanFilter kf_ivmeX/Y/Z`, `kf_irtifa` tanımları (475-485). **`kf_gyroX/Y/Z` KALIR** — gyro filtreleme telemetri içindir, `ucus_adim` onu kullanmaz.
- `float hesapla_dikey_hiz(float)` fonksiyonunun tamamı (687-716)
- `float onceki_irtifa`, `unsigned long onceki_zaman` globalleri (441-442)

**Ayrıca sil:** `unsigned long kalkis_zaman` (431) ve `src/main.cpp:976`'daki sıfırlaması. Bu değişkeni yalnızca taşınan durum-biti bloğu kullanıyordu (`grep -n kalkis_zaman src/main.cpp` ile doğrula: kalan tek kullanım kalmamalı); karşılığı `UcusHal::kalkis_t_us`'tur.

**Sakla (gölge değişkenler):** `UcusDurumu durum` (414), `bool ayrilma1/ayrilma2` (428-429), `float max_irtifa_degeri` (430), `volatile uint16_t durum_bitleri` (432), `float anlik_dikey_hiz` (443), `float eglim_acisi` (444). Bunlar her döngüde `ucus_hal`/`cikti`'dan kopyalanır; telemetri, SD log, LED ve `gonder_durum_paketi()` onlara bakmaya devam eder, böylece dosyanın geri kalanı değişmez.

> **Tip farkı:** `main.cpp`'deki `durum_bitleri` `volatile uint16_t`, `UcusCikti::durum_bitleri` ise `uint8_t`. Bu bilinçlidir: Tablo 5 sekiz bit tanımlar, üst bayt bugün de daima 0'dır (`src/main.cpp:817-818` iki bayta ayırırken üst baytı zaten 0 gönderir). Atama genişletme yönündedir, bilgi kaybı yoktur. `main.cpp` tarafındaki `volatile uint16_t` bildirimini **değiştirme** — `gonder_durum_paketi()` ona bağlı.

- [ ] **Step 3: Global `ucus_hal`'i ekle ve `setup()`'ta sıfırla**

`UcusDurumu durum = HAZIR;` satırının yanına ekle:

```cpp
UcusHal ucus_hal;   // saf algoritmanin tum durumu (Kalman + state machine)
```

`setup()` içinde, sensör başlatmadan **önce** ekle:

```cpp
    ucus_sifirla(ucus_hal, UcusAyar());   // varsayilan esikler = eski #define degerleri
```

`src/main.cpp:971` civarındaki mod geçişi sıfırlamasında (`durum = HAZIR;` yazan yer) o satırı şununla değiştir:

```cpp
        ucus_sifirla(ucus_hal, UcusAyar());
        durum = HAZIR;
```

- [ ] **Step 4: Sensör okuma bloğunu ham değer üretecek şekilde ayarla**

`src/main.cpp:988-995`'te Kalman çağrılarını kaldır, ham değerleri sakla:

```cpp
        sensors_event_t a, g, o;
        bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
        ham_ivmeX = a.acceleration.x;    // filtreleme ucus_adim() icinde
        ham_ivmeY = a.acceleration.y;
        ham_ivmeZ = a.acceleration.z;
```

`bno.getEvent(&g, ...)` bloğu **değişmez** (gyro filtreleme telemetri için yerinde kalır).

`src/main.cpp:1029` satırındaki irtifa okumasını değiştir:

```cpp
        ham_irtifa = bme.readAltitude(referans_basinc);   // filtreleme ucus_adim() icinde
```

Global tanımlara ekle:

```cpp
float ham_ivmeX = 0.0, ham_ivmeY = 0.0, ham_ivmeZ = 0.0, ham_irtifa = 0.0;
```

SUT enjeksiyonu `src/main.cpp:1349-1362` arasındadır ve bugün filtrelenmiş globallere yazar. `ham_*` değişkenlerine yazacak şekilde değiştir (`basinc`, `roll`, `pitch`, `yaw`, `qx/qy/qz` satırlarına **dokunma** — onlar zaten ham gider):

```cpp
                      if (sitSutMod == MOD_SUT) {
                          // Gelen FLOAT32'ler BIG ENDIAN (Ek-7)
                          ham_irtifa = be32_to_float(&ttl_buf[1]);
                          basinc     = be32_to_float(&ttl_buf[5]);
                          ham_ivmeX  = be32_to_float(&ttl_buf[9]);
                          ham_ivmeY  = be32_to_float(&ttl_buf[13]);
                          ham_ivmeZ  = be32_to_float(&ttl_buf[17]);
                          roll   = be32_to_float(&ttl_buf[21]);
                          pitch  = be32_to_float(&ttl_buf[25]);
                          yaw    = be32_to_float(&ttl_buf[29]);
                          // SUT'ta 3D quaternion telemetrisi kullanilmaz -> kimlik (identity)
                          // gonderilir. eglim_acisi yine enjekte roll/pitch'ten hesaplanir.
                          qx = qy = qz = 0.0f;
                      }
```

> **`filtrele` bayrağının gerekçesi (dikkat):** SUT'ta enjekte edilen irtifa/ivme bugün Kalman'dan **geçmiyor** — donanım okuma bloğu (`src/main.cpp:987`) tamamen atlandığı için `kf_irtifa`/`kf_ivme*` SUT'ta hiç çağrılmıyor. Filtreyi koşulsuz `ucus_adim` içine almak bu davranışı sessizce değiştirirdi ve F1 artık "taşıma" olmaktan çıkardı. Bu yüzden `UcusGirdi::filtrele` alanı var: firmware `(sitSutMod != MOD_SUT)` verir → bugünkü davranış birebir korunur; MATLAB `true` verir → ham sensör modelinden gelen veri gerçek filtre zincirinden geçer (H4'ün gereği). Bayrak, bugüne kadar örtük olan bir davranışı arayüzde görünür kılar; değiştirmez.

- [ ] **Step 5: Eğim/hız hesabı ve algoritma bloğunu tek çağrıyla değiştir**

`src/main.cpp:1055-1225` arasındaki bloğun tamamını (eğim hesabı, `hesapla_dikey_hiz` çağrısı, `switch (durum)`, yedek tetik, durum bitleri) şununla değiştir:

```cpp
    // --- FUNYE ZAMANLAMA KONTROLU (Non-Blocking) ---
    funye_guncelle();

    // --- LED GOSTERGELERINI GUNCELLE ---
    led_uygula();

    // --- UCUS ALGORITMASI (saf modul — UcusAlgoritmasi/ucus_algoritmasi.h) ---
    UcusGirdi girdi;
    girdi.ham_irtifa   = ham_irtifa;
    girdi.ham_ivme[0]  = ham_ivmeX;
    girdi.ham_ivme[1]  = ham_ivmeY;
    girdi.ham_ivme[2]  = ham_ivmeZ;
    girdi.ham_euler[0] = roll;
    girdi.ham_euler[1] = pitch;
    girdi.ham_euler[2] = yaw;
    girdi.ham_kuat[0]  = sqrtf(fmaxf(0.0f, 1.0f - (qx*qx + qy*qy + qz*qz)));  // qw >= 0
    girdi.ham_kuat[1]  = qx;
    girdi.ham_kuat[2]  = qy;
    girdi.ham_kuat[3]  = qz;
    girdi.t_us         = (uint32_t)micros();
    girdi.sut_modu     = (sitSutMod == MOD_SUT);
    girdi.filtrele     = (sitSutMod != MOD_SUT);   // SUT bugun de filtresiz — davranis korunur

    UcusCikti cikti;
    ucus_adim(ucus_hal, girdi, cikti);

    // EMRI DONANIMA CEVIREN TEK YER. Funye pin guvenligi degismedi.
    if (cikti.funye1_emir) Funye1Atesle();
    if (cikti.funye2_emir) Funye2Atesle();

    // Golge degiskenler: telemetri, SD log ve LED kodu bunlara bakmaya devam eder.
    durum            = cikti.durum;
    durum_bitleri    = cikti.durum_bitleri;
    ayrilma1         = ucus_hal.ayrilma1;
    ayrilma2         = ucus_hal.ayrilma2;
    irtifa           = cikti.irtifa;
    anlik_dikey_hiz  = cikti.dikey_hiz;
    eglim_acisi      = cikti.eglim_acisi;
    max_irtifa_degeri = cikti.max_irtifa;
    ivmeX = cikti.ivme[0];  ivmeY = cikti.ivme[1];  ivmeZ = cikti.ivme[2];
```

> `ham_kuat[0]` (qw) yeniden kurulur çünkü `main.cpp` yalnız `qx/qy/qz`'yi saklar. Eğim formülü `1 - 2*(qx^2 + qy^2)` yalnız qx ve qy'yi kullandığından qw'nin değeri sonucu etkilemez; yine de arayüzün tam olması için doğru işaretle doldurulur.

- [ ] **Step 6: Tüm firmware ortamlarının derlendiğini doğrula**

```bash
pio run -e ucus && pio run -e ucusdebug && pio run -e sitsut
```

Beklenen: üçü de `SUCCESS`. Derleme hatası varsa Adım 2'de silinmesi gereken bir tanım kalmış veya gölge değişken eksik demektir.

- [ ] **Step 7: Native testlerin hâlâ geçtiğini doğrula**

```bash
pio test -e native
```

Beklenen: `26 Tests 0 Failures 0 Ignored — PASSED`

- [ ] **Step 8: Taşıma diff'ini satır satır gözden geçir**

Spec §4.5'in ikinci doğrulama kademesi. Otomatik değil, **gözle** yapılır.

```bash
git diff src/main.cpp | less
git diff src/main.cpp --stat
```

Her silinen bloğun `UcusAlgoritmasi/ucus_algoritmasi.h` içinde bir karşılığı olduğunu doğrula. Özellikle kontrol et:

- Silinen her `#define` başlıkta **aynı değerle** var mı?
- Apogee koşulunun dört terimi (`T && A && B && D`) aynı sırada ve aynı karşılaştırma operatörleriyle mi?
- Yedek tetiğin beş koşulu eksiksiz mi?
- Sekiz durum bitinin her biri aynı koşulla mı set ediliyor?
- Fünye bloğu diff'te **hiç görünmüyor** mu?

```bash
git diff src/main.cpp | grep -nE "^[-+].*(funye_pin|Funye1Atesle\(\)\{|Funye2Atesle\(\)\{|pinMode\(PIN_FUNYE)"
```

Beklenen: **çıktı boş.** Bir satır bile çıkarsa fünye güvenlik bloğuna dokunulmuş demektir — geri al.

- [ ] **Step 9: Commit**

```bash
git add src/main.cpp platformio.ini
git commit -m "main.cpp saf ucus algoritmasi moduluna baglandi"
```

---

### Task 7: Mevcut Test Dosyasındaki Kopyaları Kaldır

**Files:**
- Modify: `test/test_ucus/test_main.cpp:53-67` (kopya sabitler), `88-104` (kopya Kalman), `105-106` (kopya enum), `240-250` (kopya dikey hız), `416-510` (kopya durum makinesi mantığı)

**Interfaces:**
- Consumes: Görev 5'in `ucus_algoritmasi.h` arayüzü
- Produces: yok (test temizliği)

**Neden bu görev var:** Bu dosyadaki `apogee_karari()` ve `yedek_ana_tetik()` fonksiyonları `main.cpp` mantığının **elle senkron tutulan kopyalarıdır**. Bugün bu testler, test dosyasının kendi kopyasının kendisiyle tutarlı olduğunu doğrular — `main.cpp`'deki bir sapmayı yakalayamaz. F1'in amacı tam olarak bu sınıf drift'i ortadan kaldırmaktır.

- [ ] **Step 1: Kopya tanımları başlık include'u ile değiştir**

`test/test_ucus/test_main.cpp` içindeki `#include <Adafruit_BME280.h>` satırından sonra ekle:

```cpp
#include "ucus_algoritmasi.h"   // GERCEK ucus algoritmasi — kopya DEGIL
```

Sonra şu blokları **sil**:
- `--- UCUS ALGORITMASI SABITLERI (main.cpp ile senkron) ---` başlığı altındaki tüm `#define`'lar (53-67)
- `class SimpleKalmanFilter { ... };` kopyası (88-104)
- `enum UcusDurumu { ... };` kopyası (105-106)
- `float hesapla_dikey_hiz_test(...)` kopyası (240-250)

- [ ] **Step 2: Durum makinesi testlerini gerçek koda bağla**

`test_sm_*` testlerinin tamamını (416-510) sil ve yerine şunu yaz:

```cpp
// ============================================================
// DURUM MAKINESI TESTLERI
// ============================================================
// Bu testler artik GERCEK ucus_adim() fonksiyonunu cagirir. Ayrintili
// senaryo kapsami test/test_ucus_algo/test_algo.cpp icindedir (native,
// kart gerektirmez, `pio test -e native`). Burada yalnizca kart uzerinde
// derlenen kodun ayni sonucu verdigi dogrulanir — derleyici/float farki
// olup olmadigini yakalar.

static UcusCikti kart_adim(UcusHal& h, float irtifa, float az, float egim,
                           uint32_t t_us) {
    UcusGirdi g;
    g.ham_irtifa = irtifa;
    g.ham_ivme[0] = 0.0f; g.ham_ivme[1] = 0.0f; g.ham_ivme[2] = az;
    g.ham_euler[0] = egim; g.ham_euler[1] = 0.0f; g.ham_euler[2] = 0.0f;
    g.ham_kuat[0] = 1.0f; g.ham_kuat[1] = 0.0f;
    g.ham_kuat[2] = 0.0f; g.ham_kuat[3] = 0.0f;
    g.t_us = t_us; g.sut_modu = true; g.filtrele = true;
    UcusCikti c; ucus_adim(h, g, c); return c;
}

static UcusCikti kart_oturt(UcusHal& h, float irtifa, float az, float egim,
                            uint32_t& t_us, int adet = 40) {
    UcusCikti c;
    for (int i = 0; i < adet; i++) { t_us += 10000u; c = kart_adim(h, irtifa, az, egim, t_us); }
    return c;
}

// Kalkisi garantiler. Kalman ivme filtresi basamak girdiyi TEK adimda
// gecirmedigi icin kalkis birkac ornekte olur — bkz. test_algo.cpp icindeki
// kalkis_yap() yorumu.
static void kart_kalkis(UcusHal& h, uint32_t& t_us) {
    for (int i = 0; i < 10; i++) { t_us += 10000u; kart_adim(h, 0.0f, 40.0f, 0.0f, t_us); }
}

void test_sm_kalkis(void) {
    UcusHal h; ucus_sifirla(h, UcusAyar());
    uint32_t t = 1000000u;
    TEST_ASSERT_EQUAL_INT(HAZIR, kart_adim(h, 0.0f, 0.0f, 0.0f, t).durum);
    kart_kalkis(h, t);
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, h.durum);
}

void test_sm_apogee_tam_kosul(void) {
    UcusHal h; ucus_sifirla(h, UcusAyar());
    uint32_t t = 1000000u;
    kart_kalkis(h, t);
    kart_oturt(h, 600.0f, 0.0f, 5.0f, t);
    TEST_ASSERT_EQUAL_INT(INIS_1, kart_oturt(h, 580.0f, 0.0f, 5.0f, t).durum);
    TEST_ASSERT_TRUE(h.ayrilma1);
}

void test_sm_apogee_egim_engeller(void) {
    UcusHal h; ucus_sifirla(h, UcusAyar());
    uint32_t t = 1000000u;
    kart_kalkis(h, t);
    kart_oturt(h, 600.0f, 0.0f, 100.0f, t);
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, kart_oturt(h, 580.0f, 0.0f, 100.0f, t).durum);
    TEST_ASSERT_FALSE(h.ayrilma1);
}

void test_sm_yedek_ana_apogee_kacti(void) {
    UcusHal h; ucus_sifirla(h, UcusAyar());
    uint32_t t = 1000000u;
    kart_kalkis(h, t);
    kart_oturt(h, 700.0f, 0.0f, 120.0f, t);
    UcusCikti c = kart_oturt(h, 400.0f, 0.0f, 120.0f, t);
    TEST_ASSERT_FALSE(h.ayrilma1);
    TEST_ASSERT_TRUE(h.ayrilma2);
    TEST_ASSERT_EQUAL_INT(INIS_2, c.durum);
}

void test_sm_inis2_yere_inis(void) {
    UcusHal h; ucus_sifirla(h, UcusAyar());
    uint32_t t = 1000000u;
    kart_kalkis(h, t);
    kart_oturt(h, 700.0f, 0.0f, 5.0f, t);
    kart_oturt(h, 650.0f, 0.0f, 5.0f, t);
    kart_oturt(h, 400.0f, 0.0f, 5.0f, t);
    TEST_ASSERT_EQUAL_INT(INDI, kart_oturt(h, 10.0f, 0.0f, 5.0f, t).durum);
}
```

- [ ] **Step 3: `RUN_TEST` listesini güncelle**

`main()`/`setup()` içindeki `RUN_TEST` çağrılarından silinen testlere ait olanları kaldır. Kalması gerekenler:

```cpp
    RUN_TEST(test_sm_kalkis);
    RUN_TEST(test_sm_apogee_tam_kosul);
    RUN_TEST(test_sm_apogee_egim_engeller);
    RUN_TEST(test_sm_yedek_ana_apogee_kacti);
    RUN_TEST(test_sm_inis2_yere_inis);
```

Silinecekler: `test_sm_apogee_hafif_pozitif_hiz_gecer`, `test_sm_apogee_yukseliste_atesleme_yok`, `test_sm_apogee_rampada_negatif_irtifa_atesleme_yok`, `test_sm_apogee_arama_tabani_siniri`, `test_sm_inis1_ana_parasut`, `test_sm_yedek_ana_yukselirken_atesleme`, `test_sm_yedek_ana_cift_atesleme_yok`, `test_sm_yedek_ana_rampada_atesleme`, `test_dikey_hiz_*` (üçü de) — karşılıkları `test_algo.cpp`'de gerçek koda bağlı olarak duruyor.

`test_kalman_*`, `test_eglim_*`, `test_packet_*`, `test_crc16_*`, `test_cerceve_*` ve tüm `test_hw_*` testleri **olduğu gibi kalır**.

- [ ] **Step 4: Kart üzerinde testleri koş**

```bash
pio test -e esp32dev -f test_ucus
```

Beklenen: tüm `[A]` yazılım testleri PASS. `[B]` donanım testleri sensör/SD takılıysa PASS, takılı değilse FAIL — bu beklenen davranıştır ve bu görevin kapsamı dışındadır.

- [ ] **Step 5: Commit**

```bash
git add test/test_ucus/test_main.cpp
git commit -m "test_ucus kopyalari kaldirildi, testler gercek ucus_adim'a baglandi"
```

---

### Task 8: Bit-Bit Eşdeğerlik Doğrulaması

**Files:**
- Create: `docs/superpowers/altin_referans/sut_tasima_sonrasi.csv`
- Create: `docs/superpowers/altin_referans/KARSILASTIRMA.md`

**Interfaces:**
- Consumes: Görev 1'in `sut_referans.csv`'si, Görev 6'nın yeni firmware'i
- Produces: eşdeğerlik kanıtı — F1'in bitiş koşulu

**Ön koşul:** ESP32 kartı bağlı (Görev 1 ile aynı kart, aynı port).

- [ ] **Step 1: Yeni firmware'i yükle ve aynı senaryoyu koş**

```bash
pio run -e sitsut --target upload
python3 "SİT_SUT/sit_sut_test.py"      # menuden 5 sec
mv docs/superpowers/altin_referans/sut_referans.csv \
   docs/superpowers/altin_referans/sut_tasima_sonrasi.csv
git checkout docs/superpowers/altin_referans/sut_referans.csv
```

- [ ] **Step 2: İki kaydı karşılaştır**

```bash
diff docs/superpowers/altin_referans/sut_referans.csv \
     docs/superpowers/altin_referans/sut_tasima_sonrasi.csv && echo "OZDES"
```

Beklenen: `OZDES`

Fark varsa **taşıma hatalıdır.** Farklı satırları bul ve hangi bitin değiştiğine bak:

```bash
diff docs/superpowers/altin_referans/sut_referans.csv \
     docs/superpowers/altin_referans/sut_tasima_sonrasi.csv | head -20
```

Bit numarasından hangi mantığın kaydığını çıkar (`ST_BIT_*` tanımları `ucus_algoritmasi.h` içinde), `src/main.cpp` git geçmişindeki taşıma öncesi hâliyle karşılaştır, düzelt ve Adım 1'den tekrarla. **Referans dosyasını "eşitlemek için" DEĞİŞTİRME.**

- [ ] **Step 3: Sonucu belgele**

`docs/superpowers/altin_referans/KARSILASTIRMA.md`:

```markdown
# F1 Eşdeğerlik Doğrulaması

**Tarih:** <koşum tarihi>
**Kart:** ESP32 DevKit, `pio run -e sitsut`
**Senaryo:** `SİT_SUT/sit_sut_test.py` menü 5 (tam sentetik uçuş profili)

| Kayıt | Dosya | Commit |
| :--- | :--- | :--- |
| Taşıma öncesi | `sut_referans.csv` | <Görev 1 commit sha> |
| Taşıma sonrası | `sut_tasima_sonrasi.csv` | <Görev 6 commit sha> |

**Sonuç:** İki kayıt bit-bit özdeştir. `ucus_algoritmasi.h` çıkarımı uçuş
davranışını değiştirmemiştir.

**Kapsam:** Bu karşılaştırma SUT senaryosunun kapsadığı yolları doğrular
(kalkış, apogee, drogue, ana paraşüt, iniş). Senaryonun uyarmadığı yollar —
arıza modları, uç durumlar — `test/test_ucus_algo/test_algo.cpp` içindeki
birim testlerle kapsanır.
```

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/altin_referans/
git commit -m "F1 esdegerlik dogrulamasi: tasima oncesi/sonrasi SUT ciktilari ozdes"
```

---

## F1 Bitiş Koşulu

Aşağıdakilerin hepsi sağlandığında F1 tamamdır ve F2'ye (MEX köprüsü) geçilebilir:

- [ ] `pio test -e native` → 26 test, 0 hata
- [ ] `pio run -e ucus`, `-e ucusdebug`, `-e sitsut` → üçü de SUCCESS
- [ ] `pio test -e esp32dev -f test_ucus` → `[A]` yazılım testleri PASS
- [ ] `docs/superpowers/altin_referans/` içindeki iki CSV bit-bit özdeş
- [ ] `grep -nE "digitalWrite|pinMode|millis|micros|Serial" UcusAlgoritmasi/ucus_algoritmasi.h` → **çıktı boş**
- [ ] `src/main.cpp:600-675` (fünye pin güvenlik bloğu) git diff'te **değişmemiş**

## Kapsam Dışı — Ayrı İş Kalemleri

Bu plan sırasında karşılaşılacak ama **düzeltilmeyecek** konular. Fark edilirse not edilir, ayrı ele alınır:

1. **README §4.3 / kod tutarsızlığı:** README apogee Kriteri B'yi `Vz < 0` anlatıyor, kod `< MIN_DIKEY_HIZ` (+3 m/s) kullanıyor. Hangisinin doğru olduğu F4'te aday A3 ile ölçülecek (spec §8.5).
2. **README §4.1 filtre sayısı:** "Euler ×3 dahil 10 filtre" yazıyor, gerçek sayı 7. Doküman düzeltmesi.
3. **`APOGEE_SERBEST_IVME` (Kriter C):** tanımlı ama devrede değil. F4'te aday A1 olarak ölçülecek.
4. **BNO055 Z-ekseni yönü TODO'su** (`src/main.cpp:1091`): F3'te sensör modeli ile her iki konvansiyon koşulup çözülecek.
