# MatlabSim — Uçuş Algoritması Simülasyonu

Bu klasör, **karta yüklenecek gerçek uçuş algoritmasını** MATLAB'de bir uçuşun
içinde koşturur. Algoritma yeniden yazılmadı: `ucus_mex`, doğrudan
`UcusAlgoritmasi/ucus_algoritmasi.h` dosyasını derler — firmware ile aynı kaynak.

## Nereden başlanır

```matlab
cd MatlabSim
oyna
```

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

## İki ayrı şeyi değiştirebilirsin

### 1. Roketi değiştirmek → `config/roket_params.m`

İtki eğrisi, kütle, Cd, paraşüt `Cd·A`, rüzgâr. Bunlar **roketin** özellikleri,
yazılımın değil. Şu an içindekiler temsili değerlerdir — gerçek roket verilerini
buraya yazman gerekiyor.

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

## Şu an geçerli olan ve olmayan

**Geçerli:** algoritmanın mantığı, durum geçişleri, eşiklerin birbirine göre
etkisi, senaryolara tepkisi.

**Henüz geçerli değil:** mutlak sayılar. İki sebepten —

1. `config/roket_params.m` içindeki roket verileri temsili.
2. Sensör gürültü parametreleri **kalibre edilmedi**. Tasarım dokümanı §6.3:
   bu değerler uydurulmaz, karttan ölçülür (hareketsiz kartta 5 dakikalık
   `ucusdebug` logu → σ, bias, drift). O yapılmadan çıkan eşik önerileri
   dairesel akıl yürütmedir.

## Testler

```bash
cd MatlabSim && matlab -batch "test_mex"        # köprü doğrulaması (18 test)
~/.platformio/penv/bin/pio test -e native        # saf algoritma (26 test)
```

Köprüyü yeniden derlemek (başlığı değiştirdiysen):

```bash
cd MatlabSim/mex && matlab -batch "derle"
```
