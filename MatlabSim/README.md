# MatlabSim — Uçuş Algoritması Simülasyonu

Bu klasör, **karta yüklenecek gerçek uçuş algoritmasını** MATLAB'de bir uçuşun
içinde koşturur. Algoritma yeniden yazılmadı: `ucus_mex`, doğrudan
`UcusAlgoritmasi/ucus_algoritmasi.h` dosyasını derler — firmware ile aynı kaynak.

## Nereden başlanır

Dosyalar kendi konumlarını bulur — hangi klasörden çalıştırdığın önemli değil.
VSCode'da `oyna.m`'i açıp **F5**'e basman yeterli.

```matlab
oyna                  % tek uçuş + grafikler
kos_senaryolar()      % 10 doğrulama senaryosu
monte_carlo([], 50)   % belirsizlik altında 50 uçuş
supur('apogee_irtifa_farki', [5 10 20 30], 20)   % eşik süpürme
```

Bulgular için [BULGULAR.md](BULGULAR.md).

`oyna.m` üç uçuş koşar (nominal, eşik denemesi, arıza senaryosu), özetleri yazar
ve grafikleri `cikti/` altına kaydeder. Değiştirip tekrar çalıştıracağın dosya budur.

## Dosyalar

| Dosya | Ne işe yarar |
| :--- | :--- |
| `oyna.m` | **Giriş noktası.** Denemelerini burada yaparsın. |
| `config/roket_params.m` | Roketin fiziği: kütle, itki eğrisi, Cd, paraşütler, rüzgâr, sensör gürültüsü. |
| `sim_ucus.m` | Kapalı döngü simülasyon: 3-DOF roket modeli + sensör modeli + algoritma. |
| `ciz.m` | Dört panelli grafik (irtifa / Vz / eğim / durum makinesi). |
| `ozet.m` | Sayısal özet + sert kısıt kontrolü. |
| `mex/ucus_mex.cpp` | MATLAB ↔ C++ köprüsü. |
| `mex/derle.m` | Köprüyü derler. |
| `test_mex.m` | Köprünün doğrulaması (18 test). |
| `senaryolar.m` | 10 senaryoluk doğrulama kütüphanesi. |
| `kos_senaryolar.m` | Senaryoları koşar, tablo + şekil üretir. |
| `ariza_yok.m` | Arıza enjeksiyonu alanları (baro donması, IMU doyma, takla…). |
| `monte_carlo.m` | Belirsizlik dağılımları altında N uçuş. |
| `supur.m` | Tek eşiği ortak rastgele sayılarla süpürür. |
| **`BULGULAR.md`** | **Simülasyondan çıkan bulgular — önce bunu oku.** |

## İki ayrı şeyi değiştirebilirsin

### 1. Roketi değiştirmek → `config/roket_params.m`

İtki eğrisi, kütle, Cd, paraşüt `Cd·A`, rüzgâr. Bunlar **roketin** özellikleri,
yazılımın değil. İçindekiler `config/rocket.ork` dosyasından çıkarılmış gerçek
"Akdoğan" verileridir (`[ORK]` etiketli satırlar). `[TAHMIN]` etiketli olanlar
hâlâ varsayım — ölçtüğünde güncelle.

### 2. Algoritmayı değiştirmek → `oyna.m` içindeki `ayar` yapısı

Bunlar uçuş yazılımının eşikleri. Yazmadığın her eşik firmware'deki
varsayılanında kalır.

```matlab
ayar = struct();
ayar.apogee_irtifa_farki = 25;      % 10 yerine 25 m
ayar.max_eglim           = 60;      % 75 yerine 60 derece
s = sim_ucus(p, ayar);
```

| Alan | Varsayılan | Ne yapar |
| :--- | :---: | :--- |
| `kalkis_ivme_esigi` | 20 m/s² | Kalkış tespiti |
| `apogee_irtifa_farki` | 10 m | Apogee: maks. irtifadan bu kadar düşünce |
| `min_dikey_hiz` | 3 m/s | Apogee: hız bunun altına inince |
| `max_eglim` | 75° | Ateşleme izni (güvenlik kapısı) |
| `apogee_min_irtifa` | 550 m | Bu irtifa aşılmadan apogee aranmaz |
| `ayrilma2_mesafe` | 550 m | Ana paraşüt irtifası |
| `inis_hiz_esigi` | 2 m/s | İniş tespiti |
| `inis_irtifa_esigi` | 20 m | İniş tespiti |
| `kf_irtifa_p` | `[1.5 1.5 0.1]` | İrtifa Kalman'ı `(e_mea, e_est, q)` |

> Buradaki değişiklikler **yalnızca simülasyondadır**. Beğendiğin bir eşiği
> uçuracaksan `UcusAlgoritmasi/ucus_algoritmasi.h` içindeki `#define`
> varsayılanını da güncellemen gerekir.

## Arıza senaryosu denemek

`roket_params.m` içindeki gürültü/arıza alanlarıyla oynayarak:

```matlab
p.baro_dinamik_k = 0.06;   % kötü statik delik → sahte irtifa düşüşü
p.ruzgar         = 15;     % güçlü rüzgâr → weathercocking, eğim kapısı zorlanır
p.baro_sigma     = 3.0;    % gürültülü barometre
```

## Grafik nasıl okunur

- **Gri çizgi** = algoritmanın *gördüğü* (gürültülü sensör + Kalman)
- **Siyah çizgi** = gerçek (algoritma bunu asla görmez)
- **Turuncu dikey çizgi** = drogue emri, **mavi** = ana paraşüt emri
- **Turuncu üçgen** = gerçek apogee

Algoritmanın iyiliği, turuncu çizginin turuncu üçgene ne kadar yakın olduğudur.

## Model doğrulaması

Roket verileri `config/rocket.ork` (OpenRocket 24.12, "Akdoğan") dosyasından
çıkarıldı. Model, OpenRocket'ın kendi simülasyonuna karşı doğrulandı:

| | OpenRocket | Bu sim | Fark |
| :--- | ---: | ---: | ---: |
| Apogee | 3703.3 m | 3704.4 m | %0.03 |
| Apogee zamanı | 27.81 s | 27.67 s | %0.5 |
| Maks hız | 276.8 m/s | 275.5 m/s | %0.5 |
| Rampa çıkış hızı | 31.0 m/s | 31.1 m/s | %0.3 |
| Yere çarpma | 6.3 m/s | 5.7 m/s | %10 |
| Apogee eğimi | 24.7° | 11.5° | bkz. BULGULAR.md Bulgu 3 |

Tek serbest parametre gövde `Cd`'siydi; apogee'ye göre süpürülüp 0.500 bulundu.
Diğer her şey (itki eğrisi, kütleler, paraşüt `Cd·A`, ayrılma kütlesi, CP−CG,
atalet momenti, rüzgâr, rampa) doğrudan `.ork` dosyasından geldi.

## Şu an geçerli olan ve olmayan

**Geçerli:** yörünge (yukarıdaki tabloya göre), algoritmanın mantığı, durum
geçişleri, eşiklerin birbirine göre etkisi.

**Henüz geçerli değil:** sensör gürültüsüne bağlı sonuçlar. Gürültü
parametreleri **kalibre edilmedi** — tasarım dokümanı §6.3: bu değerler
uydurulmaz, karttan ölçülür (hareketsiz kartta 5 dakikalık `ucusdebug` logu →
σ, bias, drift). O yapılmadan çıkan eşik önerileri dairesel akıl yürütmedir.

**Model basitleştirmeleri:** 3-DOF düzlemsel (roll/yaw yok — algoritma zaten
onlara bakmıyor); ayrılma sonrası yalnız ana gövde izlenir (burun/görev yükü
bölümü 5.13 kg ile ayrı iner, modellenmedi); paraşüt altında gövde
aerodinamiği yerine basit sönümleme.

## Testler

```bash
cd MatlabSim && matlab -batch "test_mex"        # köprü doğrulaması (18 test)
~/.platformio/penv/bin/pio test -e native        # saf algoritma (26 test)
```

Köprüyü yeniden derlemek (başlığı değiştirdiysen):

```bash
cd MatlabSim/mex && matlab -batch "derle"
```
