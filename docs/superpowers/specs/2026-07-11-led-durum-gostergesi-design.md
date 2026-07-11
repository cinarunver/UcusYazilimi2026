# LED Durum Göstergesi Tasarımı

**Tarih:** 2026-07-11
**Dosya:** `src/main.cpp`
**Kapsam:** Normal uçuş sırasında 3 LED'in (LED1/LED2/LED3) uçuş fazına ve config aşamasına göre sürülmesi.

## Amaç

Roket kartındaki 3 gösterge LED'ini, açılıştan iniş tamamlanmasına kadar sistemin
hangi aşamada olduğunu sahadan tek bakışta okunabilir hale getirecek şekilde
düzenlemek. Ateşleme göstergeleri (drogue/ana) mevcut davranışını korur; üstüne
"config heartbeat", "armed/HAZIR" ve "indi" durumları eklenir.

## LED Pinleri (mevcut, değişmiyor)

- `PIN_LED_1` = GPIO26 → **LED1** (drogue / 1. ayrılma göstergesi)
- `PIN_LED_2` = GPIO4  → **LED2** (ana paraşüt / 2. ayrılma göstergesi)
- `PIN_LED_3` = GPIO25 → **LED3** (indi göstergesi — yeni kullanım)

`PIN_LED` (GPIO13) bu tasarımın kapsamı dışında, boş kalır.

## Durum Tablosu (Normal uçuş — MOD_BEKLEME)

| Aşama                     | LED1 | LED2 | LED3 | Davranış |
|---------------------------|:----:|:----:|:----:|----------|
| Açılış + konfigürasyon    |  ◐   |  ◐   |  ◐   | 3'ü birlikte **1 Hz blink** |
| HAZIR (rampada, armed)    |  ●   |  ●   |  ●   | 3'ü de **sabit yanık** |
| YUKSELIYOR (kalkış)       |  ○   |  ○   |  ○   | Hepsi **söner** |
| INIS_1 (drogue atıldı)    |  ●   |  ○   |  ○   | LED1 yanar (latch) |
| INIS_2 (ana atıldı)       |  ●   |  ●   |  ○   | LED1+LED2 (latch) |
| INDI (yere indi)          |  ●   |  ●   |  ●   | LED1+LED2+LED3 |

Latch doğal olarak `durum`'dan gelir: INIS_2 state'i INIS_1'i geçmiş demektir,
dolayısıyla LED1 zaten yanıktır. Ayrı bir latch bayrağı gerekmez.

## Girdiler (LED'ler yalnızca bunlara bağlı)

1. **Mod:** normal (MOD_BEKLEME) vs SIT/SUT
2. **`sistem_hazir` bayrağı:** config bitti mi (blink ↔ solid ayrımı)
3. **`durum`:** uçuş state machine değeri

Buzzer, ham sensör değerleri, GPS fix vb. LED'leri doğrudan sürmez. (GPS fix
beklentisi istenmedi; istenirse yalnızca `sistem_hazir` koşuluna eklenir.)

## SIT/SUT Modu

Bu şema **yalnızca normal modda** çalışır. SIT/SUT'a geçilirse config blink /
HAZIR-solid devreye girmez. SIT/SUT'ta LED'ler mevcut drogue/ana gösterge
davranışını (`durum`'a göre türetilen ayrılma göstergesi) korur.

## Uygulama Kararları

### 1. Tek merkezli LED kontrolü — `led_guncelle()`

Şu an `Funye1Atesle` / `Funye2Atesle` içinde dağınık `digitalWrite(PIN_LED_...)`
çağrıları var. Bunlar LED yazımından arındırılır (fünye pin yazımları kalır).
Yerine Task1 döngüsünde her turda çağrılan tek bir `led_guncelle()` fonksiyonu
gelir. Bu fonksiyon LED'leri `mod + sistem_hazir + durum`'dan türetir. Böylece
tek doğruluk kaynağı olur, blink ile latch birbiriyle çakışmaz.

Blink zamanlaması non-blocking: `millis()` tabanlı, 500 ms aç / 500 ms kapa
(1 Hz), ayrı bir `delay` kullanılmadan.

### 2. `sistem_hazir` bayrağı

Global `volatile bool sistem_hazir = false;`. `setup()` tüm başlangıç işlerini
(sensör init, kalibrasyon, referans basınç, SD, LoRa config) bitirip Task'ları
başlatmadan hemen önce `true` yapılır. `false` iken config blink, `true` iken
HAZIR-solid (durum HAZIR ise).

## Mod geçişi ile etkileşim

Task1'deki mevcut mod-geçiş sıfırlama bloğu ([main.cpp:635](../../../src/main.cpp#L635))
LED'leri LOW yapıyor; `led_guncelle()` her döngüde yeniden kurduğu için bu blok
LED yazımından çıkarılabilir ya da olduğu gibi bırakılabilir (bir sonraki turda
`led_guncelle()` doğru durumu geri kurar). Uygulama planında netleştirilecek.

## Test / Doğrulama

- SUT ile sentetik uçuş: HAZIR-solid → kalkış (hepsi söner) → drogue (LED1) →
  ana (LED2) → indi (LED3) sekansı gözlemlenir. (SUT'ta config blink beklenmez.)
- Açılışta 1 Hz blink'in `setup()` boyunca sürüp HAZIR'da solid'e döndüğü
  doğrulanır.
- SIT/SUT'a geçiş ve DURDUR sonrası LED davranışının bozulmadığı kontrol edilir.
