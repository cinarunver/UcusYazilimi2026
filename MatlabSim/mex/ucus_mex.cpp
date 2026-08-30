// ============================================================================
//  ucus_mex — MATLAB <-> GERCEK UCUS ALGORITMASI koprusu
// ============================================================================
//  Bu dosya UcusAlgoritmasi/ucus_algoritmasi.h'i DOGRUDAN include eder.
//  MATLAB'de kosturulan algoritma, karta yuklenen firmware ile FIZIKSEL
//  OLARAK AYNI kaynak dosyadir. Yeniden yazilmis bir kopya DEGILDIR.
//
//  KULLANIM (MATLAB):
//     hal            = ucus_mex('sifirla');            % varsayilan esikler
//     hal            = ucus_mex('sifirla', ayar);      % ozel esikler (F4 supurme)
//     [hal, cikti]   = ucus_mex('adim', hal, girdi);   % tek adim
//
//  `hal` MATLAB tarafinda OPAK bir uint8 dizisidir (UcusHal'in ham baytlari).
//  MEX DURUMSUZDUR: hicbir global tutmaz, her cagri hal'i alir ve dondurur.
//  Bu sayede parfor guvenlidir ve kosumlar birbirini kirletemez.
//
//  DERLEME:  MatlabSim/mex/derle.m
// ============================================================================

#include "mex.h"
#include "ucus_algoritmasi.h"
#include <string.h>
#include <type_traits>

// hal'in ham bayt olarak tasinabilmesi sart. Sinifa pointer/sanal fonksiyon
// eklenirse bu assert derleme aninda patlar — sessiz bozulma olmaz.
static_assert(std::is_trivially_copyable<UcusHal>::value,
              "UcusHal ham bayt olarak kopyalanabilir olmali (MEX koprusu buna dayanir)");

// --- Yardimci: struct'tan skaler alan oku, yoksa varsayilani birak ---
static void alan_oku(const mxArray* s, const char* ad, float& hedef) {
    const mxArray* f = mxGetField(s, 0, ad);
    if (f != NULL && !mxIsEmpty(f)) hedef = (float)mxGetScalar(f);
}
static void alan_oku_u32(const mxArray* s, const char* ad, uint32_t& hedef) {
    const mxArray* f = mxGetField(s, 0, ad);
    if (f != NULL && !mxIsEmpty(f)) hedef = (uint32_t)mxGetScalar(f);
}
static void alan_oku_u8(const mxArray* s, const char* ad, uint8_t& hedef) {
    const mxArray* f = mxGetField(s, 0, ad);
    if (f != NULL && !mxIsEmpty(f)) hedef = (uint8_t)mxGetScalar(f);
}
// Vektor alan (n elemanli) oku
static void alan_oku_vek(const mxArray* s, const char* ad, float* hedef, int n) {
    const mxArray* f = mxGetField(s, 0, ad);
    if (f == NULL || mxIsEmpty(f)) return;
    if ((int)mxGetNumberOfElements(f) != n)
        mexErrMsgIdAndTxt("ucus_mex:alanBoyutu", "'%s' alani %d elemanli olmali.", ad, n);
    double* p = mxGetPr(f);
    for (int i = 0; i < n; i++) hedef[i] = (float)p[i];
}

// --- girdi struct'indan zorunlu vektor oku ---
static void zorunlu_vek(const mxArray* s, const char* ad, float* hedef, int n) {
    const mxArray* f = mxGetField(s, 0, ad);
    if (f == NULL)
        mexErrMsgIdAndTxt("ucus_mex:eksikAlan", "girdi struct'inda '%s' alani yok.", ad);
    if ((int)mxGetNumberOfElements(f) != n)
        mexErrMsgIdAndTxt("ucus_mex:alanBoyutu", "'%s' alani %d elemanli olmali.", ad, n);
    double* p = mxGetPr(f);
    for (int i = 0; i < n; i++) hedef[i] = (float)p[i];
}
static double zorunlu_skaler(const mxArray* s, const char* ad) {
    const mxArray* f = mxGetField(s, 0, ad);
    if (f == NULL || mxIsEmpty(f))
        mexErrMsgIdAndTxt("ucus_mex:eksikAlan", "girdi struct'inda '%s' alani yok.", ad);
    return mxGetScalar(f);
}

// --- UcusAyar'i MATLAB struct'indan doldur (verilmeyen alanlar varsayilan) ---
static UcusAyar ayar_oku(const mxArray* s) {
    UcusAyar a;   // NSDMI varsayilanlari = firmware degerleri
    if (s == NULL || !mxIsStruct(s)) return a;
    alan_oku(s, "kalkis_ivme_esigi",      a.kalkis_ivme_esigi);
    alan_oku(s, "apogee_irtifa_farki",    a.apogee_irtifa_farki);
    alan_oku(s, "min_dikey_hiz",          a.min_dikey_hiz);
    alan_oku(s, "max_eglim",              a.max_eglim);
    alan_oku(s, "apogee_min_irtifa",      a.apogee_min_irtifa);
    alan_oku(s, "ayrilma2_mesafe",        a.ayrilma2_mesafe);
    alan_oku(s, "inis_hiz_esigi",         a.inis_hiz_esigi);
    alan_oku(s, "inis_irtifa_esigi",      a.inis_irtifa_esigi);
    alan_oku(s, "durum_min_irtifa_esigi", a.durum_min_irtifa_esigi);
    alan_oku(s, "durum_aci_esigi",        a.durum_aci_esigi);
    alan_oku(s, "durum_yatay_ivme_esigi", a.durum_yatay_ivme_esigi);
    alan_oku_u32(s, "motor_yanma_sure_us", a.motor_yanma_sure_us);
    alan_oku_vek(s, "kf_irtifa_p", a.kf_irtifa_p, 3);
    alan_oku_vek(s, "kf_ivme_p",   a.kf_ivme_p,   3);
    alan_oku_vek(s, "kf_gyro_p",   a.kf_gyro_p,   3);
    alan_oku_u8(s, "apogee_aday",  a.apogee_aday);
    return a;
}

// --- UcusHal'i uint8 mxArray olarak paketle ---
static mxArray* hal_paketle(const UcusHal& h) {
    mxArray* out = mxCreateNumericMatrix(1, sizeof(UcusHal), mxUINT8_CLASS, mxREAL);
    memcpy(mxGetData(out), &h, sizeof(UcusHal));
    return out;
}

// --- uint8 mxArray'den UcusHal cikar ---
static void hal_coz(const mxArray* a, UcusHal& h) {
    if (!mxIsUint8(a) || mxGetNumberOfElements(a) != sizeof(UcusHal))
        mexErrMsgIdAndTxt("ucus_mex:halBoyutu",
            "hal %d baytlik uint8 dizisi olmali (gelen: %d). 'sifirla' ile uret.",
            (int)sizeof(UcusHal), (int)mxGetNumberOfElements(a));
    memcpy(&h, mxGetData(a), sizeof(UcusHal));
}

// --- UcusCikti'yi MATLAB struct'ina cevir ---
static mxArray* cikti_paketle(const UcusCikti& c) {
    const char* alanlar[] = {"durum", "funye1_emir", "funye2_emir", "durum_bitleri",
                             "irtifa", "dikey_hiz", "eglim_acisi", "max_irtifa", "ivme"};
    mxArray* s = mxCreateStructMatrix(1, 1, 9, alanlar);
    mxSetField(s, 0, "durum",         mxCreateDoubleScalar((double)c.durum));
    mxSetField(s, 0, "funye1_emir",   mxCreateLogicalScalar(c.funye1_emir));
    mxSetField(s, 0, "funye2_emir",   mxCreateLogicalScalar(c.funye2_emir));
    mxSetField(s, 0, "durum_bitleri", mxCreateDoubleScalar((double)c.durum_bitleri));
    mxSetField(s, 0, "irtifa",        mxCreateDoubleScalar((double)c.irtifa));
    mxSetField(s, 0, "dikey_hiz",     mxCreateDoubleScalar((double)c.dikey_hiz));
    mxSetField(s, 0, "eglim_acisi",   mxCreateDoubleScalar((double)c.eglim_acisi));
    mxSetField(s, 0, "max_irtifa",    mxCreateDoubleScalar((double)c.max_irtifa));
    mxArray* iv = mxCreateDoubleMatrix(1, 3, mxREAL);
    double* ivp = mxGetPr(iv);
    for (int i = 0; i < 3; i++) ivp[i] = (double)c.ivme[i];
    mxSetField(s, 0, "ivme", iv);
    return s;
}

// ============================================================================
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1 || !mxIsChar(prhs[0]))
        mexErrMsgIdAndTxt("ucus_mex:kullanim",
            "Kullanim: hal = ucus_mex('sifirla'[, ayar])  |  "
            "[hal, cikti] = ucus_mex('adim', hal, girdi)");

    char komut[32];
    mxGetString(prhs[0], komut, sizeof(komut));

    // ---------------------------------------------------------------- sifirla
    if (strcmp(komut, "sifirla") == 0) {
        UcusAyar a = ayar_oku(nrhs >= 2 ? prhs[1] : NULL);
        UcusHal h;
        ucus_sifirla(h, a);
        plhs[0] = hal_paketle(h);
        return;
    }

    // ------------------------------------------------------------------- adim
    if (strcmp(komut, "adim") == 0) {
        if (nrhs != 3)
            mexErrMsgIdAndTxt("ucus_mex:kullanim",
                "[hal, cikti] = ucus_mex('adim', hal, girdi)");
        if (!mxIsStruct(prhs[2]))
            mexErrMsgIdAndTxt("ucus_mex:girdiTipi", "girdi bir struct olmali.");

        UcusHal h;
        hal_coz(prhs[1], h);

        UcusGirdi g;
        g.ham_irtifa = (float)zorunlu_skaler(prhs[2], "ham_irtifa");
        zorunlu_vek(prhs[2], "ham_ivme",  g.ham_ivme,  3);
        zorunlu_vek(prhs[2], "ham_euler", g.ham_euler, 3);
        zorunlu_vek(prhs[2], "ham_kuat",  g.ham_kuat,  4);
        g.t_us     = (uint32_t)zorunlu_skaler(prhs[2], "t_us");
        g.sut_modu = (zorunlu_skaler(prhs[2], "sut_modu") != 0.0);
        g.filtrele = (zorunlu_skaler(prhs[2], "filtrele") != 0.0);

        UcusCikti c;
        ucus_adim(h, g, c);

        plhs[0] = hal_paketle(h);
        if (nlhs >= 2) plhs[1] = cikti_paketle(c);
        return;
    }

    // ------------------------------------------------------------------- bilgi
    // Kopru ile basligin uyumlu oldugunu dogrulamak icin (surum/kayma teshisi)
    if (strcmp(komut, "bilgi") == 0) {
        const char* alanlar[] = {"hal_bayt", "ayar_bayt", "girdi_bayt", "cikti_bayt"};
        mxArray* s = mxCreateStructMatrix(1, 1, 4, alanlar);
        mxSetField(s, 0, "hal_bayt",   mxCreateDoubleScalar((double)sizeof(UcusHal)));
        mxSetField(s, 0, "ayar_bayt",  mxCreateDoubleScalar((double)sizeof(UcusAyar)));
        mxSetField(s, 0, "girdi_bayt", mxCreateDoubleScalar((double)sizeof(UcusGirdi)));
        mxSetField(s, 0, "cikti_bayt", mxCreateDoubleScalar((double)sizeof(UcusCikti)));
        plhs[0] = s;
        return;
    }

    mexErrMsgIdAndTxt("ucus_mex:komut",
        "Bilinmeyen komut '%s'. Gecerli: 'sifirla', 'adim', 'bilgi'.", komut);
}
