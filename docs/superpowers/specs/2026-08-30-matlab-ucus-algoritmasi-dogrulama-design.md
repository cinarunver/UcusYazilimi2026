# MATLAB Tabanlı Uçuş Algoritması Doğrulama ve Eniyileme — Tasarım

**Tarih:** 2026-08-30
**Durum:** Onaylandı, uygulamaya hazır
**Kapsam:** UKB uçuş algoritmasının (apogee tespiti, ayrılma kararları, durum makinesi) MATLAB'de kapalı döngü simülasyonla doğrulanması ve eşiklerinin Monte Carlo tabanlı eniyilenmesi.

---

## 1. Amaç

Dört hedef tek çerçeveyle karşılanır:

| # | Hedef | Çıktı |
| :-: | :--- | :--- |
| H1 | **Eşik eniyileme** | Gürültü ve senaryo dağılımı altında süpürülmüş yeni sabit tablosu + gerekçe |
| H2 | **Doğrulama kanıtı** | Senaryo kütüphanesi üzerinden grafik + tablo (rapora doğrudan girer) |
| H3 | **Yeni kriter denemesi** | Devrede olmayan `APOGEE_SERBEST_IVME` kriterinin ve alternatif apogee mantıklarının uçuş koduna alınmadan önce ölçülmesi |
| H4 | **Kalman / gürültü ayarı** | İrtifa Kalman'ının `(e_mea, e_est, q)` üçlüsünün apogee gecikmesi–yumuşatma dengesine göre ayarlanması |

## 2. Temel Tasarım Kararı: Sıfır Drift

Uçuş algoritması MATLAB'de **yeniden yazılmaz.** Karar mantığı `main.cpp`'den Arduino bağımlılığı olmayan saf bir C++ başlığına taşınır; hem uçan firmware hem MATLAB **aynı dosyayı** derler.

Gerekçe: fünye ateşleme kararında "MATLAB'de optimize ettiğim kod uçan kod değilmiş" senaryosu kabul edilebilir bir risk değildir. Port yaklaşımı her `main.cpp` düzenlemesinde sessizce eskiyen bir kopya üretir.

Bu, kod tabanında **zaten var olan bir desenin** genişletilmesidir: LED karar mantığı `hesapla_led_durumu()` adlı saf fonksiyonda, donanıma yazma ise `led_uygula()` içindedir (`src/main.cpp:26-60`, `src/main.cpp:679`). Aynı ayrım uçuş algoritmasına uygulanır.

### 2.1 Modül sınırı: geniş (ham sensör → emir)

Saf modül ham sensör okumasından fünye emrine kadar olan **tüm** zinciri kapsar: Kalman filtreleri, eğim (tilt) hesabı, dikey hız türevi, durum makinesi, ana paraşüt yedek tetiği, SUT durum bitleri.

Dar sınır (yalnız durum makinesi) reddedildi: Kalman ve Vz türevi `main.cpp`'de kalsaydı MATLAB'de ayrıca modellenmeleri gerekirdi, yani H4 için drift riski geri gelirdi.

---

## 3. Mimari

```
config/roket_params.m  →  model/roket.m  →  sensor/*.m  →  ucus_mex  →  emirler
   (itki, Cd, kütle)      (gerçek yörünge)  (gürültü,     (GERÇEK        (fünye 1/2)
                                  ↑          bias, arıza)   uçuş kodu)         │
                                  └───────────────────────────────────────────┘
                                     paraşüt açılınca sürükleme değişir
```

**Döngü kapalıdır.** Algoritma drogue emri verdiğinde roket modelinin sürükleme katsayısı değişir; sonraki yörünge algoritmanın kararına bağlıdır. Açık döngü (önce yörünge üret, sonra algoritmayı üzerinde koştur) kullanılamaz — geç apogee tespitinin iniş hızına ve sürüklenmeye etkisi ölçülemez, ki eniyilemenin asıl konusu budur.

### 3.1 Dizin yapısı

```
UcusAlgoritmasi/
  ucus_algoritmasi.h        ← main.cpp ve MEX ortak kaynağı

MatlabSim/
  config/
    roket_params.m          itki eğrisi, kütle, Cd, CP-CG, paraşüt Cd·A
    sensor_params.m         ölçülmüş gürültü/bias/drift parametreleri
  +model/
    roket.m                 3-DOF RK4 entegratör
    atmosfer.m              ISA (rho, p, T)
    itki.m                  .eng eğrisi enterpolasyonu
    parasut.m               sonlu açılma süreli Cd·A
  +sensor/
    baro.m                  BME280 modeli (dinamik basınç dahil)
    imu.m                   BNO055 modeli (bias, hizalama, doyma)
    ariza.m                 arıza enjeksiyonu
  mex/
    ucus_mex.cpp            MATLAB ↔ ucus_algoritmasi.h köprüsü
    derle.m                 mex derleme betiği
  senaryo/
    senaryolar.m            10 senaryoluk kütüphane (bkz. §7)
  kosum.m                   tek uçuş koşumu
  monte_carlo.m             N koşum, parametre dağılımlarından örnekleme
  eniyile.m                 ızgara + örüntü arama, CRN
  raporla.m                 figür ve tablo üretimi
  hil_dogrula.m             gerçek kart karşılaştırması (SUT protokolü)
```

---

## 4. C++ Tarafı

### 4.1 `UcusAlgoritmasi/ucus_algoritmasi.h` arayüzü

Arduino bağımlılığı yok, global değişken yok, donanım erişimi yok.

```cpp
struct UcusGirdi {              // HAM sensör — filtreleme modülün içinde
    float ham_irtifa;           // BME280, yer referansına göre [m]
    float ham_ivme[3];          // BNO055 lineer ivme [m/s^2]
    float ham_euler[3];         // roll, pitch, yaw [derece]
    float ham_kuat[4];          // qw, qx, qy, qz  (SUT'ta kimlik kuaterniyon)
    uint32_t t_us;              // micros() DIŞARIDAN verilir
    bool   sut_modu;            // tilt kaynağı: euler mi kuaterniyon mu
};

struct UcusHal {                // TÜM durum burada; main.cpp'de global kalmaz
    SimpleKalmanFilter kf_ivme[3], kf_gyro[3], kf_irtifa;
    float    onceki_irtifa;
    uint32_t onceki_t_us;
    UcusDurumu durum;
    float    max_irtifa;
    bool     ayrilma1, ayrilma2;
    uint32_t kalkis_t_us;
    uint8_t  durum_bitleri;
};

struct UcusCikti {
    UcusDurumu durum;
    bool     funye1_emir, funye2_emir;   // seviye değil KENAR — yalnız bir kez true
    uint8_t  durum_bitleri;
    float    irtifa, dikey_hiz, eglim_acisi, max_irtifa;  // telemetri + SD log
};

void ucus_sifirla(UcusHal&);
void ucus_adim(UcusHal&, const UcusGirdi&, UcusCikti&);
```

**Kritik:** Saf modül fünye **ateşlemez, emir üretir.** Pini süren tek yer `funye_pin_ates()` olarak kalır. Güvenlik katmanı korunur, üstelik karar mantığı artık o pinlere erişimi olmayan bir dosyadadır.

### 4.2 `main.cpp`'den taşınanlar

| Taşınan | Mevcut konum |
| :--- | :--- |
| `enum UcusDurumu` | `src/main.cpp:17-22` |
| `class SimpleKalmanFilter` (zaten saf) | `src/main.cpp:448-472` |
| `kf_*` filtre örnekleri → `UcusHal` alanları | `src/main.cpp:475-485` |
| Eşik `#define`'ları | `src/main.cpp:276-435` arası ilgili satırlar |
| Eğim (tilt) hesabı | `src/main.cpp:1060-1070` |
| `hesapla_dikey_hiz` (`micros()` → `t_us` girdisi) | `src/main.cpp:687` |
| Durum makinesi + yedek tetik + SUT durum bitleri | `src/main.cpp:1087-1225` |

Yorum blokları **birebir** taşınır, silinmez — gerekçeler (özellikle `APOGEE_MIN_IRTIFA`, `MAX_EGLIM`, yedek tetik ve `APOGEE_SERBEST_IVME` notları) kodun değerli kısmıdır.

### 4.3 `main.cpp`'de değişmeyenler

- **Fünye pin güvenlik bloğu** (`src/main.cpp:600-675`): `funye_pin_ates`, `funye_pin_serbest`, `Funye1Atesle`, `Funye2Atesle`, `funye_guncelle`. Tek karakter dokunulmaz. Fünye pinleri `setup()`'ta OUTPUT yapılmaz kuralı aynen yerinde kalır.
- I²C sensör okuma, `led_uygula`, LoRa, SD, FreeRTOS task yapısı, SİT/SUT parser, telemetri paketleme.
- Eşik **değerleri** (yalnız yer değiştirir).

### 4.4 Yeni çağrı yeri

```cpp
UcusGirdi g = { ham_irtifa, {ax,ay,az}, {roll,pitch,yaw},
                {qw,qx,qy,qz}, micros(), (sitSutMod == MOD_SUT) };
UcusCikti c;
ucus_adim(ucus_hal, g, c);

if (c.funye1_emir) Funye1Atesle();   // emri donanıma çeviren tek yer
if (c.funye2_emir) Funye2Atesle();
durum = c.durum;  durum_bitleri = c.durum_bitleri;
```

### 4.5 Taşımanın doğrulaması

Davranış değişmez; kod yer değiştirir. Üç kademeli kanıt:

1. Mevcut `test/test_ucus/test_main.cpp` birim testleri geçer.
2. Taşıma diff'i satır satır gözden geçirilir (yalnız yer değiştirme olduğu teyit edilir).
3. Eski ve yeni firmware aynı SUT senaryosunda **bit-bit özdeş durum bitleri** üretir (mevcut `SİT_SUT/sit_sut_test.py` ile; ek araç gerekmez).

---

## 5. MEX Köprüsü

`MatlabSim/mex/ucus_mex.cpp`, `ucus_algoritmasi.h`'i doğrudan `#include` eder.

**Durum aktarımı:** `UcusHal` MATLAB tarafında opak `uint8` byte dizisi olarak tutulur.

```matlab
[hal, cikti] = ucus_mex(hal, girdi);
```

MEX durumsuzdur → `parfor` güvenlidir, bellek yönetimi gerekmez, koşumlar birbirini kirletemez.

**Başarım:** adım başı çağrı maliyeti ~10 µs. 60 s uçuş @ 100 Hz = 6000 adım ≈ 0.1 s/koşum. 10.000 koşumluk Monte Carlo tek çekirdekte ~20 dk, `parfor` ile dakikalar mertebesinde. Eniyileme için yeterlidir.

**Ön koşul:** Xcode Command Line Tools (MATLAB R2024a `mex` C++ derleyicisi için).

---

## 6. MATLAB Modelleri

### 6.1 Roket modeli — 3-DOF düzlemsel

1-DOF (yalnız dikey) yetersizdir: `MAX_EGLIM` kapısını, weathercocking'i ve motor arızası sonrası yatay uçuşu üretemez, yani senaryoların yarısı sınanamaz. 6-DOF gereksizdir: algoritma roll/yaw'a bakmaz, `MAX_EGLIM` yalnız burun açısına bakar.

- **Durum:** `[x, z, vx, vz, theta, omega]`
- **Kuvvetler:** itki (gövde ekseninde, `.eng` eğrisinden enterpolasyon), sürükleme (`-0.5*rho*v^2*Cd*A`, hız vektörüne ters), ağırlık, paraşüt sürüklemesi
- **Moment:** CP−CG kolundan aerodinamik toparlanma momenti → weathercocking ayrıca modellenmeden doğal olarak çıkar
- **Atmosfer:** ISA (`rho`, `p`, `T`); basınç zaten BME280 modelini beslemek için gereklidir
- **Rüzgar:** sabit bileşen + türbülans
- **Paraşüt:** drogue ve ana için ayrı `Cd·A`, **sonlu açılma süresi** (açılma şoku hem iniş profilini hem baro ölçümünü etkiler)
- **Entegrasyon:** sabit adım RK4 @ 1 kHz; sensör 100 Hz'de örneklenir. `ode45` kullanılmaz — kapalı döngüde algoritmanın kararı olay ürettiğinden sabit adım hem daha temiz hem daha hızlıdır.

### 6.2 Sensör modeli

Bu katman çerçevenin en kritik parçasıdır: eniyilenen eşikler burada modellenen gürültüye göre optimize olur.

**BME280 → irtifa**
- beyaz gürültü, kuantizasyon, okuma gecikmesi
- **dinamik basınç hatası:** `dp ~= k * 0.5 * rho * v^2`, `k` = statik delik yerleşimi katsayısı. Bu terim kilittir — README §4.3'te üçüncü onay kriterinin varlık sebebi olarak anlatılan "sahte irtifa düşüşü" mekanizması tam olarak budur. `k` belirsizdir, Monte Carlo'da dağılım olarak süpürülür.
- ayrılma şoku basınç transiyenti

**BNO055 → ivme / euler / kuaterniyon**
- gürültü, bias, bias sürüklenmesi
- **eksen hizalama hatası** (montaj) — `KALKIS_IVME_ESIGI`'yi doğrudan etkiler
- doyma (motor yanmasında lineer ivme doyabilir)
- motor titreşimi (geniş bantlı)
- `src/main.cpp:1091`'deki "BNO055 Z-ekseni yönü doğrulanmalı" TODO'su: her iki eksen konvansiyonu da koşulup çözülür

**Arıza enjeksiyonu:** sensör donması (son değeri tekrarlar), NaN üretimi, I²C kopması, baro tıkanması.

### 6.3 Sensör modeli kalibrasyonu — zorunlu adım

Gürültü parametreleri varsayılmaz, **ölçülür**:

1. Kart hareketsizken 5 dakika `ucusdebug` logu → beyaz gürültü σ, bias, kısa süreli drift doğrudan çıkarılır.
2. Titreşim genliği için statik ateşleme veya araba testi kaydı kullanılır.
3. Dinamik basınç katsayısı `k` ölçülemiyorsa geniş bir dağılımla süpürülür ve sonuçların ona duyarlılığı raporlanır.

Bu adım atlanırsa eniyileme kendi varsayımına optimize olur (dairesel akıl yürütme) ve mutlak sayısal sonuçlar geçersiz olur.

---

## 7. Senaryo Kütüphanesi

| # | Senaryo | Sınadığı |
| :-: | :--- | :--- |
| 1 | Nominal | temel davranış |
| 2 | Motor erken sönme (apogee ~600 m) | `APOGEE_MIN_IRTIFA` arama tabanı |
| 3 | Motor arızası → yatay uçuş | `MAX_EGLIM` güvenlik kapısı |
| 4 | Takla (tumbling) | sahte apogee reddi |
| 5 | Yayvan tepe (düşük Cd, uzun apogee) | `APOGEE_IRTIFA_FARKI` yeterliliği |
| 6 | Güçlü rüzgar / weathercocking | tilt kapısının nominal uçuşu bloke **etmemesi** |
| 7 | Kötü statik delik (yüksek dinamik basınç hatası) | sahte irtifa düşüşüne dayanıklılık |
| 8 | Apogee'de baro donması | ana paraşüt yedek tetiği |
| 9 | IMU doyma / eksen hizalama hatası | kalkış tespiti |
| 10 | Drogue hiç açılmadı | ana paraşüt yedek tetiği (son çare yolu) |

Her senaryo için tek figür üretilir: irtifa / Vz / tilt zaman serisi, durum geçişleri, fünye emri anları, gerçek apogee işareti. Bu figürler doğrudan mühendislik tasarım raporuna girer.

---

## 8. Eniyileme

### 8.1 Karar değişkenleri

`KALKIS_IVME_ESIGI`, `APOGEE_IRTIFA_FARKI`, `MIN_DIKEY_HIZ`, `MAX_EGLIM`, `APOGEE_MIN_IRTIFA`, ve irtifa Kalman'ının `(e_mea, e_est, q)` üçlüsü.

`AYRILMA2_MESAFE` sabit tutulur — takım/şartname kararıdır, yazılım kararı değildir.

### 8.2 Sert kısıtlar

İhlal eden parametre seti **reddedilir**, cezalandırılmaz:

1. Hiçbir koşumda gerçek `Vz > 0` iken drogue emri verilmez
2. Hiçbir koşumda gerçek tilt > 45° iken drogue emri verilmez
3. Drogue kaçırma oranı sıfırdır (yedek tetik dahil, her koşumda iki paraşüt de açılır)
4. Yere çarpma hızı < 10 m/s

Kısıtlar **modelin gerçek (truth) durumu** üzerinden değerlendirilir, sensör okuması üzerinden değil. Kısıt 2'deki 45°, eniyilenen `MAX_EGLIM` değişkeninden bağımsız sabit bir güvenlik tanımıdır — eniyilenen bir eşik kendi kısıtı olarak kullanılamaz, aksi halde optimizasyon kısıtı gevşeterek "iyileşir".

Kısıt ile amaç bilinçli olarak ayrılmıştır: güvenlik ihlali ceza terimiyle takas edilebilecek bir büyüklük değildir. 1000 koşumun 999'unda 0.1 s daha hızlı apogee, birinde erken ateşleme — kabul edilemez.

### 8.3 Amaç fonksiyonu (minimize)

- `E[|dt_apogee|]` — drogue emri ile gerçek apogee arasındaki süre
- `P95[|dt_apogee|]` — kuyruk kritiktir, ortalama tek başına yanıltıcıdır
- `E[|h_ana - 550|]` — ana paraşüt açılma irtifası hatası
- yatay sürüklenme (geç açılan drogue = daha uzağa düşme)

Terimler normalize edilip ağırlıklı toplanır; ağırlıklar `config` içinde açıkça tutulur.

### 8.4 Yöntem

Amaç fonksiyonu Monte Carlo gürültülü ve süreksizdir (fünye ateşler/ateşlemez), bu nedenle gradyan tabanlı yöntemler (`fmincon`) uygun değildir.

1. **Kaba ızgara süpürme** — her eşik için 5-7 nokta. Aynı zamanda duyarlılık analizidir: hangi eşik gerçekten belirleyici, hangisi önemsiz. Bu tablo doğrudan rapora girer.
2. **Yerel örüntü arama** — kaba ızgaranın en iyi noktasından başlayarak (`patternsearch`; Global Optimization Toolbox yoksa elle koordinat inişi).
3. **Ortak rastgele sayılar (CRN)** — her parametre seti **aynı** tohum kümesiyle koşulur. CRN atlanırsa Monte Carlo gürültüsü parametreler arası farkı örter ve sonuçlar anlamsız çıkar.

### 8.5 Aday kriter denemesi (H3)

Saf modül, apogee kriterini bir **aday seti** üzerinden koşabilecek şekilde parametrelendirilir; her aday `UcusHal` içindeki bir yapılandırma alanıyla seçilir, `#ifdef` kullanılmaz (MEX'ten çalışma zamanında değiştirilebilmeli).

Değerlendirilecek adaylar:

| Aday | Tanım | Sorulan soru |
| :--- | :--- | :--- |
| A0 | Mevcut: `T && A && B && D` | referans |
| A1 | A0 `&&` serbest düşüş imzası (`abs(ivme) - 9.81 < TOLERANS`) | `APOGEE_SERBEST_IVME` VE olarak bağlanırsa drogue kaçırma oranı sıfır kalır mı? |
| A2 | A0 `\|\|` plato dedektörü (irtifa türevi N örnek boyunca band içinde) | yayvan tepede gecikmeyi azaltır mı, sahte pozitif üretir mi? |
| A3 | A0, `B` kriteri `Vz < 0` ile (README'nin anlattığı hâli) | §11 bulgu 1'in cevabı |

Her aday **aynı** senaryo kütüphanesi ve **aynı** CRN tohumlarıyla koşulur; karşılaştırma tablosu §8.2 kısıtları ve §8.3 amaç terimleri üzerinden verilir. Bir aday ancak tüm sert kısıtları geçerse uçuş koduna alınması önerilir.

---

## 9. HIL Son Kapı

Eniyilenmiş eşiklerle firmware derlenir; senaryo kütüphanesinden 10-20 senaryo gerçek ESP32 kartında mevcut SİT/SUT protokolü üzerinden koşulur; kartın döndürdüğü durum bitleri MATLAB tahminiyle karşılaştırılır.

Çıktı: "simülasyon ile kart aynı kararı verdi" tablosu. Bu, tüm zincirin geçerlilik kanıtıdır ve rapordaki doğrulama bölümüne girer.

Gerçek zamanlı olduğu için eniyilemede kullanılamaz (10.000 koşum ≈ 7 gün); rolü yalnızca son doğrulamadır.

---

## 10. Geçerlilik Sınırları

**Bu çerçeve kanıtlar:**
- Algoritmanın modellenen her senaryoda doğru kararı verdiğini
- Eşik setlerinin birbirine göre güvenliğini ve isabetini
- Hangi eşiğin kritik, hangisinin önemsiz olduğunu (duyarlılık)
- Kod ile dokümantasyon arasındaki tutarsızlıkları

**Bu çerçeve kanıtlamaz:**
- Modellenmemiş bir arıza modunu — simülasyon ancak içine konan senaryolar kadar iyidir
- Sensör modeli §6.3'e göre kalibre edilmezse mutlak sayısal doğruluğu
- Donanım ve zamanlama hatalarını (I²C kilitlenmesi, task açlığı, brown-out) — bunlar HIL ve gerçek uçuş testinin konusudur

---

## 11. Yan Bulgular (bu tasarım sırasında tespit edildi)

Tasarım aşamasında kod ile dokümantasyon arasında iki tutarsızlık çıktı. İkisi de ayrı iş kalemidir, bu tasarımın kapsamında değildir, ancak kayda geçirilir:

1. **Apogee Kriteri B eşiği:** README §4.3 kriteri `Vz < 0` olarak anlatır; kod (`src/main.cpp:1138`) `anlik_dikey_hiz < MIN_DIKEY_HIZ` yani `< +3 m/s` kullanır. Hangisinin doğru olduğu simülasyonla belirlenip ikisi hizalanmalıdır.
2. **Kalman filtre sayısı:** README §4.1 "Euler ×3 dahil toplam 10 filtre" der; kod (`src/main.cpp:481`) Euler için Kalman olmadığını açıkça belirtir. Gerçek sayı **7**'dir (3 ivme + 3 gyro + 1 irtifa).

Bu iki bulgu, çerçevenin daha kurulmadan değer ürettiğinin göstergesidir.

---

## 12. Uygulama Fazları

Çerçeve tek seferde değil, her biri kendi başına doğrulanabilir dört fazda kurulur. Her faz sonunda çalışan ve kanıtlanabilir bir çıktı vardır; bir sonrakine ancak o kanıt alındıktan sonra geçilir.

| Faz | Kapsam | Bitiş kanıtı |
| :-: | :--- | :--- |
| **F1** | `ucus_algoritmasi.h` çıkarımı + `main.cpp` yeniden bağlanması | Birim testler geçer; eski/yeni firmware aynı SUT senaryosunda bit-bit özdeş durum bitleri üretir (§4.5) |
| **F2** | MEX köprüsü + tek koşum boru hattı (basit yörünge, gürültüsüz) | MATLAB'den koşulan algoritma, F1'deki SUT senaryosuyla aynı durum geçişlerini üretir |
| **F3** | 3-DOF roket modeli + sensör modeli + §6.3 kalibrasyonu + senaryo kütüphanesi | 10 senaryonun figürleri üretilir; nominal uçuşun apogee'si roketin bilinen tasarım apogee'siyle tutarlıdır |
| **F4** | Monte Carlo + eniyileme + aday kriter karşılaştırması + HIL son kapı | Eşik tablosu + duyarlılık tablosu + "sim ile kart aynı kararı verdi" tablosu |

F1 tek başına da değerlidir: uçuş kodunu test edilebilir hâle getirir ve `main.cpp`'yi küçültür; sonraki fazlar yapılmasa bile kalıcı bir iyileştirmedir.
