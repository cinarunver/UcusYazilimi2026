# Görev Yükü Hava Yoğunluğu + Gerçek Zamanlı İllüstrasyon — Tasarım

**Tarih:** 2026-07-29
**Kapsam:** `GorevYukuYazilimi/gorevyuku.cpp`, `src/main.cpp` (yalnız SD loglama), `YerIstasyonu26/YerIstasyonu2026.py`

## Amaç

1. Görev yükü BME280'den nemli hava yoğunluğunu hesaplayıp havadan yollasın; yoğunluk yer istasyonunda görünsün.
2. Görev yükünün hareket verilerinden yer istasyonunda gerçek zamanlı 3B yönelim illüstrasyonu üretilsin.
3. Uçuş logları SD karta Excel'de doğrudan açılabilir CSV olarak, her uçuşta ayrı dosyaya yazılsın.

Ana aviyoniğe (`main.cpp`) yoğunluk **eklenmez** — görev yükünün işi kendi hareket ve hava ölçüm verilerini iletmek, UKB'nin işi uçuş algoritması. Main'de yalnız SD loglama (dosya adı + CSV formatı) değişir; uçuş yolu, eşikler, durum makinesi ve fünye mantığı ellenmez.

## 1. Hava yoğunluğu (nemli hava)

Kalman'dan geçmiş `basinc` / `sicaklik` / `nem` üçlüsünden:

```
es(T) = 6.1078 · 10^(7.5·T / (T + 237.3))     [hPa]   Tetens, su üzeri
pv    = (RH/100) · es                          [hPa]
pd    = p − pv                                 [hPa]
ρ     = (pd·100)/(Rd·Tk) + (pv·100)/(Rv·Tk)    [kg/m³]
Rd = 287.058, Rv = 461.495 J/(kg·K),  Tk = T + 273.15
```

Yoğunluğa ikinci bir Kalman **uygulanmaz** — girdilerin üçü de zaten filtreli, türev değere ek filtre yalnız gecikme bindirir.

Savunma sınırları: `RH` [0,100]'e kırpılır, `pv > p` olursa `p`'ye kırpılır (fiziksel olarak imkânsız; bozuk sensörde negatif `pd` üretmesin), `Tk` alt sınırı 1 K (sıfıra bölme), Tetens paydası `T + 237.3` sıfıra yaklaşırsa hesap atlanır.

`es` ve `pv` ara değerleri **yalnız SD'ye** yazılır, havadan gitmez: `p`, `T`, `RH` zaten pakette olduğundan yerde birebir türetilebilirler, 4 bayt boşa gitmez.

## 2. Görev yükü wire paketi: 24B → 32B

| # | alan | tip | ölçek |
|---|---|---|---|
| 0 | basinc | uint16 | ×10 (hPa) |
| 1 | sicaklik | int16 | ×100 |
| 2 | nem | uint16 | ×100 |
| 3 | irtifa | int16 | ×10 |
| 4 | **yogunluk** | **uint16** | **×1000 (kg/m³)** |
| 5-6 | gpsEnlem, gpsBoylam | int32 ×2 | ×1e7 |
| 7 | ivmeToplam | int16 | ×100 |
| 8-10 | **qx, qy, qz** | **int16 ×3** | **×10000** |
| 11-13 | gyroX, gyroY, gyroZ | int16 ×3 | ×10 |

Python format: `'<HhHh2i4h'` → **`'<HhHhH2i7h'`** (32B). Çerçeve `[0xAA][0x55][LEN=32][payload][CRC_HI][CRC_LO]` = 37B.

Yoğunluk `uint16` ×1000 → 0…65.535 kg/m³, çözünürlük 0.001. Deniz seviyesi 1.225 → `1225`.

Quaternion `main.cpp` ile birebir: `w < 0` ise üç bileşenin işareti çevrilir (`w ≥ 0` garantisi), yer istasyonunda `w = √(1 − x² − y² − z²)`. Gyro pakette kalır — quaternion yönelimi, gyro dönüş *hızını* verir; illüstrasyondaki spin barları gyro'yu kullanır.

## 3. SD kart loglama

### Dosya adlandırma — her uçuşta yeni dosya

Açılışta ilk boş isim seçilir:

- UKB: `/ukb_log.csv`, `/ukb_log1.csv`, `/ukb_log2.csv`, …
- Görev yükü: `/gy_log.csv`, `/gy_log1.csv`, `/gy_log2.csv`, …

Sıra 999'a kadar taranır; dolarsa son dosyaya append edilir (log kaybetmektense birleştir). Seçilen dosya adı LoRa teşhis mesajı olarak yayınlanır, böylece hangi uçuşun hangi dosyaya yazıldığı yerden görülür.

Bu, dosya adının kartı ayırt ettiği eski düzenin (`ukb_1` / `gy_1` vs `gy_2`) yerini alır. İki kart kendi SD'sine yazdığından çakışma olmaz.

### CSV formatı — Türkçe Excel uyumlu

Ayraç `;`, ondalık ayırıcı `,` (ör. `1013,25`). Türkçe Excel'de çift tıkla açılır, sütunlar ve sayılar doğru gelir. Python tarafında `pd.read_csv(f, sep=';', decimal=',')`.

Ondalık dönüşümü `snprintf` sonrası satır içindeki `.` karakterlerinin `,` ile değiştirilmesiyle yapılır (alan ayracı zaten `;` olduğu için çakışma yok).

Her iki dosya da başlık satırı ile başlar ve ilk sütun `zaman_ms` (`millis()`) olur — zamansız uçuş logu analiz edilemiyor.

**UKB (`ukb_log*.csv`):**
`zaman_ms;ivmeX;ivmeY;ivmeZ;ivmeToplam;gyroX;gyroY;gyroZ;roll;pitch;yaw;qx;qy;qz;irtifa_m;dikeyHiz_ms;eglim_derece;gpsEnlem;gpsBoylam;ayrilma1;ayrilma2;ucus_durumu`

**Görev yükü (`gy_log*.csv`):**
`zaman_ms;basinc_hPa;sicaklik_C;nem_pct;irtifa_m;es_hPa;pv_hPa;yogunluk_kgm3;gpsEnlem;gpsBoylam;ivmeX;ivmeY;ivmeZ;ivmeToplam;qx;qy;qz;gyroX;gyroY;gyroZ`

GPS 7 haneye çıkarılır (wire ×1e7 ile aynı çözünürlük).

### Taşma düzeltmesi

`bufferla_ve_yaz_sd()` içinde `snprintf` kesme yaptığında (satır tampondan uzunsa) dönüş değeri tampon boyunu aşar; sonraki `memcpy` bu değerle çağrıldığı için ping-pong tamponunun dışına yazar. Dönüş değeri tampon boyuna kırpılır. Satır uzadığı için `temp_line` her iki dosyada 320 bayta çıkarılır.

## 4. Yer istasyonu

### 4.1 Mevcut hata: roket paketi quaternion'ı Euler sanıyor

Firmware `TelemetryWire`'a `qx, qy, qz` (×10000, int16) basıyor; yer istasyonu `'<3hH3h2iB'` ile aynı baytları `roll, pitch (×100), yaw (uint16 ×100)` olarak çözüyor. Boyut tesadüfen eşit (23B) olduğundan CRC tutuyor ve paket geçerli görünüyor, ama:

- `qx = 0.35` → ekranda `roll = 35.00°`
- `qz` negatifken `yaw` slotu unsigned okunduğu için 600°+ saçma değer

Sonuç: roket 3B modeli ve CSV'deki roll/pitch/yaw sütunları hâlihazırda hatalı. Format `'<7h2iB'` yapılır, quaternion çözülür, roll/pitch/yaw quaternion'dan türetilir (CSV, yapay ufuk ve mevcut göstergeler için).

### 4.2 Parse ve ölçekler

`PAYLOAD_PACKET_FORMAT = '<HhHhH2i7h'` (32B), `WIRE_OLCEK_YOGUNLUK = 1000.0`, `WIRE_OLCEK_QUAT = 10000.0`.

Her iki parse fonksiyonu quaternion'dan `w`'yi geri hesaplar (`1 − x² − y² − z²` negatife düşerse 0'a kırpılır) ve roll/pitch/yaw'ı standart quaternion→Euler dönüşümüyle üretir; pitch'te `asin` argümanı [−1,1]'e kırpılır.

### 4.3 Arayüz

- Görev yükü paneline **Hava Yoğunluğu (kg/m³)** satırı.
- `Rocket3DWidget` quaternion tabanlı olur: `set_quat(x, y, z)` ile rotasyon matrisi kurulup `glMultMatrixf` ile uygulanır — Euler'e hiç uğranmaz, gimbal lock yok. Geriye dönük `set_angles` kaldırılır.
- 3B sekmesi **Roket | Görev Yükü** yan yana iki bağımsız model.
- Her ikisi için **yapay ufuk** (roll/pitch), **yaw pusulası** ve **3 eksen dönüş hızı barı** (gyro).
- Simülatör her iki paketi de yeni formatta üretir (quaternion + yoğunluk).
- CSV şemalarına yeni alanlar eklenir.

## Doğrulama

- Firmware: `pio run` her iki hedef için temiz derlenir.
- Round-trip: firmware quantize ölçekleri ile Python parse ölçeklerinin birebir olduğu, `struct.calcsize` = 32 / 23 kontrolü.
- Yoğunluk: 1013.25 hPa / 15 °C / %0 RH → ≈1.2250 kg/m³; %100 RH → ≈1.2197 kg/m³ (nem düzeltmesi doğru yönde).
- Yer istasyonu simülatör modunda açılır, yoğunluk ve iki 3B model gerçek zamanlı döner.
