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
//  Port (yeniden yazma) yaklasimi reddedildi: her main.cpp duzenlemesinde
//  sessizce eskiyen bir kopya uretirdi ve funye atesleme kararinda "MATLAB'de
//  optimize ettigim kod ucan kod degilmis" senaryosu kabul edilemez.
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

// ============================================================================
//  KALMAN FILTRESI
// ============================================================================
//  main.cpp'den bire bir tasindi. Tek eklenti: varsayilan kurucu + ayarla().
//  Gerekce: UcusHal icinde dizi uyesi olarak tutulabilmesi icin varsayilan
//  kurucu gerekiyor; parametreler ucus_sifirla() icinde ayarla() ile verilir.
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
// Konvansiyon olculmeden VE olarak baglanamaz (drogue hic acilmayabilir),
// VEYA olarak da baglanmamali (baro yukseliste donarsa erken atesleme
// uretir). Once olcum, sonra VE. Aday karsilastirmasi icin bkz. tasarim
// dokumani §8.5 (UcusAyar::apogee_aday).
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

    // Apogee kriteri adayi (tasarim dokumani §8.5). 0 = A0 = mevcut ucan mantik.
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
    // isareti uretmemek icin. main.cpp'deki orijinal davranisla ayni.
    if (delta_t <= 0.0f) {
        return h.son_dikey_hiz;
    }

    float hiz_z = (guncel_irtifa - h.onceki_irtifa) / delta_t;

    h.onceki_t_us   = t_us;
    h.onceki_irtifa = guncel_irtifa;
    h.son_dikey_hiz = hiz_z;
    return hiz_z;
}

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
    bool     filtrele;        // false: ham degerler DOGRUDAN kullanilir.
                              // SUT enjeksiyonu bugun de filtresiz calisir
                              // (donanim okuma blogu atlandigi icin kf_* hic
                              // cagrilmaz); bayrak o davranisi birebir korur.
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
            //   Aday karsilastirmasi icin bkz. tasarim dokumani §8.5.
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
    // Gozlem bitleri durum makinesine BAGLANMAZ: apogee kapisi (max_eglim)
    // bloke olsa bile roket fiilen alcaliyorsa bit4/bit6 yanmali. Aksi halde
    // tek bir kapi 4 biti birden dusurur ve yer istasyonu "alcaliyor ama emir
    // yok" teshisini goremez.
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

#endif // UCUS_ALGORITMASI_H
