# Simülasyon Bulguları

**Tarih:** 2026-08-30 · **Model:** `MatlabSim`, OpenRocket "Akdoğan" verileriyle kalibre
**Algoritma:** `UcusAlgoritmasi/ucus_algoritmasi.h` — karta yüklenecek kodun kendisi

---

## Model doğrulaması

Altı bağımsız ölçütte OpenRocket'ın kendi simülasyonuna karşı doğrulandı. Tek serbest parametre gövde `Cd`'siydi (apogee'ye göre 0.500 bulundu); diğer her şey `.ork` dosyasından geldi.

| Ölçüt | OpenRocket | Model | Fark |
| :--- | ---: | ---: | ---: |
| Apogee | 3703.3 m | 3704.4 m | %0.03 |
| Apogee zamanı | 27.81 s | 27.67 s | %0.5 |
| Maks hız | 276.8 m/s | 275.5 m/s | %0.5 |
| Rampa çıkış hızı | 31.0 m/s | 31.1 m/s | %0.3 |
| Yere çarpma | 6.3 m/s | 5.7 m/s | %10 |
| Apogee eğimi | 24.7° | 24.1° (düzeltmeli) / 11.5° (ham) | bkz. Bulgu 3 |

---

## Bulgu 1 — Apogee 550 m altında kalırsa HİÇBİR paraşüt açılmıyor

**Önem: yüksek.** Kurtarma tamamen başarısız, 62 m/s ile yere çarpma.

Motor erken sönmesi senaryosunda (itki 1.4 s'de kesiliyor) apogee 476 m'de kalıyor. `APOGEE_MIN_IRTIFA = 550 m` arama tabanı apogee tespitini bloke ediyor — bu doğru ve amaçlanan davranış. Ama **ana paraşüt yedek tetiği de** `max_irtifa > AYRILMA2_MESAFE` (550 m) koşuluna bağlı olduğundan o da tetiklenmiyor.

Sonuç: 550 m'nin altında zirve yapan her uçuş, iki kurtarma yolunu da kaybediyor.

Bu, `main.cpp`'deki `APOGEE_MIN_IRTIFA` yorumunun uyardığı rampa koruması ile aynı eşiğin **bilinmeyen maliyeti**. İki eşiğin `#define APOGEE_MIN_IRTIFA AYRILMA2_MESAFE` ile birbirine bağlanmış olması bu tek noktayı yaratıyor.

**Karar takımın:** Düşük apogee'li bir uçuşta kurtarma isteniyor mu? İsteniyorsa ana paraşüt yedek tetiğinin tabanı apogee arama tabanından ayrılmalı (örn. yedek tetik 200 m'den itibaren geçerli olsun). İstenmiyorsa bu bilinçli bir kabul olarak rapora yazılmalı.

---

## Bulgu 2 — Barometre donması iki kurtarma yolunu birden düşürüyor

**Önem: yüksek.** Kurtarma tamamen başarısız, 208 m/s ile yere çarpma.

Apogee civarında barometre son değerini tekrarlamaya başlarsa (I²C kopması, statik delik tıkanması):

- Apogee kriteri A (`max_irtifa − irtifa > 10 m`) sağlanamaz → drogue yok
- Ana paraşüt yedek tetiği de `irtifa < 550` koşuluna bağlı → o da yok

Her iki yol da **aynı sensöre** dayanıyor. Yedek tetiğin varlık sebebi "tek bir apogee kaçırması iki paraşütü birden düşürmesin" idi; ama sensör seviyesinde yedeklilik yok.

**Öneri:** IMU tabanlı bağımsız bir yol düşünülmeli — örneğin kalkıştan sonra geçen süreye veya ivme imzasına dayalı, yalnızca barometre sağlıksızken devreye giren bir son çare. Bu yeni bir tasarım kararıdır, bu simülasyonun kapsamı dışında.

---

## Bulgu 3 — MAX_EGLIM marjı bu modelle CEVAPLANAMAZ

**Önem: yüksek (belirsizlik olarak).** Bu bölümden eşik önerisi üretilmemelidir.

2D yunuslama modeli apogee civarındaki eğimi düşük tahmin ediyor: OpenRocket referansı 24.7°, model 11.5°. Sebep, apogee civarında dinamik basıncın çok düşük ve hücum açısının büyük olması — bu rejimi doğrusal 2D moment modeli yakalayamıyor. `Cn_alpha`'yı fiziksel değeri 6.4'ten 35'e çıkarmak sayıyı tutturuyor ama `.ork` verisiyle çelişiyor ve tırmanış eğimini bozuyor.

Tek noktada kalibre edilmiş bir düzeltme katsayısı denendi ve **savunulamaz** bulundu — katsayı, drogue kaçırma oranını doğrudan belirliyor:

| `egim_duzeltme` | Drogue kaçırma (20 koşum, ortak tohum) |
| :---: | :---: |
| 1.00 (ham model) | 6 / 20 |
| 1.50 | 16 / 20 |
| 2.09 (referansa kalibre) | 16 / 20 |

Katsayı düşük rüzgârlı koşumlarda bile görülen eğimi 98°'ye çıkarıp kapıyı haksız yere kapatıyor. Varsayılan bu yüzden **1.0** (düzeltme kapalı) bırakıldı.

**Çözüm yolu:** OpenRocket'ta birkaç rüzgâr hızında (0 / 5 / 10 / 15 / 20 m/s) koşum yapıp apogee eğimi–rüzgâr referans eğrisi çıkarmak. Takımda OpenRocket zaten var; bu birkaç saatlik iş ve `MAX_EGLIM` sorusunu kapatır.

> Ham modelde bile 6/20 (%30) drogue kaçırma görülüyor. Bu oran doğruysa ayrıca incelenmeli; model belirsizliği çözülmeden yorumlanmamalı.

---

## Bulgu 4 — `APOGEE_IRTIFA_FARKI` beklendiği gibi ve tahmin edilebilir davranıyor

Ortak rastgele sayılarla süpürme (20 koşum, aynı tohumlar):

| `apogee_irtifa_farki` | Apogee gecikmesi | İrtifa kaybı |
| ---: | ---: | ---: |
| 5 m | 1.13 s | ~5 m |
| 10 m (mevcut) | 1.55 s | ~11 m |
| 20 m | 2.13 s | ~21 m |
| 30 m | 2.61 s | ~32 m |

İlişki doğrusala yakın ve sert kısıt ihlali yok. Mevcut 10 m makul; düşürmenin kazancı küçük (0.4 s), gürültüye karşı payı azaltır.

---

## Bulgu 5 — Kod ile dokümantasyon arasında iki tutarsızlık

Tasarım aşamasında tespit edildi, hâlâ açık:

1. **Apogee Kriteri B:** README §4.3 `Vz < 0` diyor, kod (`main.cpp:1138`) `< MIN_DIKEY_HIZ` yani `< +3 m/s` kullanıyor.
2. **Kalman filtre sayısı:** README §4.1 "Euler ×3 dahil 10 filtre" diyor, kod Euler için Kalman olmadığını belirtiyor. Gerçek sayı **7**.

---

## Senaryo kütüphanesi sonuçları

10 senaryo, `kos_senaryolar()` ile üretilir; şekiller `cikti/senaryo/` altındadır.

| # | Senaryo | Sonuç | Not |
| :-: | :--- | :---: | :--- |
| 1 | Nominal | GEÇTİ | gecikme +1.56 s |
| 2 | Motor erken sönme | **KALDI** | Bulgu 1 |
| 3 | Apogee civarı kararlılık kaybı | GEÇTİ | gecikme +2.90 s |
| 4 | Takla (tumbling) | GEÇTİ | gecikme +1.54 s |
| 5 | Yayvan tepe (düşük Cd) | GEÇTİ | gecikme +1.56 s |
| 6 | Güçlü rüzgâr (18 m/s) | GEÇTİ | gecikme +8.11 s — dikkat |
| 7 | Kötü statik delik | GEÇTİ | erken ateşleme yok |
| 8 | Apogee'de baro donması | **KALDI** | Bulgu 2 |
| 9 | IMU doyma (4G) | GEÇTİ | kalkış tespiti sağlam |
| 10 | Drogue fünyesi çalışmadı | GEÇTİ | yedek tetik ana paraşütü açtı |

---

## Geçerlilik sınırları

**Geçerli:** yörünge (doğrulama tablosuna göre), durum makinesi mantığı, eşiklerin birbirine göre etkisi, arıza modlarına tepki.

**Geçerli değil:**
- **Sensör gürültü parametreleri kalibre edilmedi.** Karttan ölçüm bekliyor (hareketsiz kartta 5 dk `ucusdebug` logu). Gürültüye duyarlı her sonuç geçicidir.
- **Apogee civarı eğim dinamiği** — Bulgu 3.
- Model 3-DOF düzlemseldir; ayrılma sonrası yalnız ana gövde izlenir; paraşüt altında gövde aerodinamiği yerine basit sönümleme vardır.

---

## Sıradaki adımlar

1. **Karttan gürültü kalibrasyonu** (5 dk `ucusdebug` logu) — gürültüye bağlı tüm sonuçları geçerli kılar.
2. **OpenRocket rüzgâr taraması** — Bulgu 3'ü kapatır.
3. **Bulgu 1 ve 2 için takım kararı** — eşik ayrımı ve sensör yedekliliği tasarım kararlarıdır.
4. **F1 Görev 1/6/7/8** (kart gerektirir) — firmware'in saf modüle bağlanması ve bit-bit eşdeğerlik doğrulaması.
