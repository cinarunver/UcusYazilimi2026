// ============================================================================
//  Saf ucus algoritmasi birim testleri — DONANIM GEREKTIRMEZ.
//  Calistir:  pio test -e native
//
//  Bu testler GERCEK ucus_adim() fonksiyonunu cagirir — kopyasini degil.
//  (test/test_ucus/test_main.cpp'deki eski durum makinesi testleri main.cpp
//  mantiginin elle senkron tutulan kopyalarini test ediyordu; bir sapmayi
//  yakalayamazlardi. Bu dosya o sinif drift'i ortadan kaldirir.)
// ============================================================================
#include <unity.h>
#include "ucus_algoritmasi.h"

void setUp(void) {}
void tearDown(void) {}

// ============================================================================
//  TEMEL YAPILAR
// ============================================================================

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

// ============================================================================
//  EGIM (TILT)
// ============================================================================

// Tam dik roket: egim 0.
void test_eglim_dik_sifir(void) {
    float e[3] = {0,0,0};
    float q[4] = {1,0,0,0};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ucus_egim_acisi(e, q, false));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ucus_egim_acisi(e, q, true));
}

// SUT modunda egim EULER'den, donanim modunda KUATERNIYON'dan gelir.
// Ayni fiziksel yonelim icin iki yol AYNI degeri vermeli.
void test_eglim_quat_euler_ile_ayni(void) {
    float e[3] = {30.0f, 0.0f, 0.0f};   // roll, pitch, yaw
    float yari = 15.0f * (float)M_PI / 180.0f;
    float q[4] = {cosf(yari), sinf(yari), 0.0f, 0.0f};  // qw,qx,qy,qz
    float sut     = ucus_egim_acisi(e, q, true);
    float donanim = ucus_egim_acisi(e, q, false);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 30.0f, sut);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, sut, donanim);
}

// KRITIK: acos girdisi 1.0'i asarsa NaN doner ve apogee ASLA tetiklenmez.
void test_eglim_nan_uretmiyor(void) {
    float e[3] = {0,0,0};
    float q[4]      = {1.0f, 0.0f, 0.0f, 0.0f};
    float qasiri[4] = {0.0f, 1.0f, 1.0f, 0.0f};   // 1-2*(1+1) = -3  -> kirpilir
    TEST_ASSERT_FALSE(isnan(ucus_egim_acisi(e, q, false)));
    TEST_ASSERT_FALSE(isnan(ucus_egim_acisi(e, qasiri, false)));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 180.0f, ucus_egim_acisi(e, qasiri, false));
}

// Takla halindeki roket guvenlik kapisini kapatmali (egim > max_eglim).
void test_eglim_tumbling_buyuk_aci(void) {
    UcusAyar a;
    float e[3] = {100.0f, 0.0f, 0.0f};
    float q[4] = {1,0,0,0};
    TEST_ASSERT_TRUE(ucus_egim_acisi(e, q, true) > a.max_eglim);
}

// ============================================================================
//  DIKEY HIZ
// ============================================================================

void test_dikey_hiz_ilk_cagri_sifir(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ucus_dikey_hiz(h, 100.0f, 1000000u));
    TEST_ASSERT_EQUAL_UINT32(1000000u, h.onceki_t_us);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 100.0f, h.onceki_irtifa);
}

void test_dikey_hiz_yukselis(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    ucus_dikey_hiz(h, 100.0f, 1000000u);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, ucus_dikey_hiz(h, 150.0f, 1500000u));
}

void test_dikey_hiz_inis(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    ucus_dikey_hiz(h, 150.0f, 1000000u);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, ucus_dikey_hiz(h, 100.0f, 1500000u));
}

// dt <= 0: ESKI HIZ KORUNUR (0 DONMEZ). 0 dondurmek apogee aninda sahte
// "hiz sifirlandi" isareti uretirdi.
void test_dikey_hiz_dt_sifir_eski_hizi_korur(void) {
    UcusHizHal h = {0.0f, 0u, 0.0f};
    ucus_dikey_hiz(h, 100.0f, 1000000u);
    float v = ucus_dikey_hiz(h, 150.0f, 1500000u);   // +100 m/s
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, v);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, v, ucus_dikey_hiz(h, 900.0f, 1500000u));
}

// ============================================================================
//  DURUM MAKINESI — ucus_adim()
// ============================================================================

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

// Kalman gecikmesini asarak irtifayi hedefe oturtur.
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

   Bu yuzden testlerde kalkis TEK bir adimla yapilmaz. */
static void kalkis_yap(UcusHal& h, uint32_t& t_us) {
    for (int i = 0; i < 10; i++) {
        t_us += 10000u;
        adim(h, 0.0f, 40.0f, 0.0f, t_us);
    }
}

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
    oturt(h, 700.0f, 0.0f, 120.0f, t);        // egim kapisi surekli KAPALI
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
   (max_eglim) bloke olsa bile roket fiilen alcaliyorsa bit4 yanmali. */
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

// ============================================================================
int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_durum_enum_degerleri);
    RUN_TEST(test_kalman_ilk_cagri);
    RUN_TEST(test_kalman_varsayilan_kurucu_ayarla_ile_ayni);
    RUN_TEST(test_ayar_varsayilanlari);
    RUN_TEST(test_eglim_dik_sifir);
    RUN_TEST(test_eglim_quat_euler_ile_ayni);
    RUN_TEST(test_eglim_nan_uretmiyor);
    RUN_TEST(test_eglim_tumbling_buyuk_aci);
    RUN_TEST(test_dikey_hiz_ilk_cagri_sifir);
    RUN_TEST(test_dikey_hiz_yukselis);
    RUN_TEST(test_dikey_hiz_inis);
    RUN_TEST(test_dikey_hiz_dt_sifir_eski_hizi_korur);
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
    return UNITY_END();
}
