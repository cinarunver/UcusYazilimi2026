function p = roket_params()
%ROKET_PARAMS  Simulasyonun TUM fiziksel girdileri tek yerde.
%
%   Burasi senin oynayacagin ilk dosya. Roketin gercek degerlerini buraya
%   yazdiginda simulasyon o rokete gore kosar.
%
%   >>> UYARI <<<
%   Asagidaki degerler TEMSILI baslangic degerleridir — senin roketinin
%   olculmus verileri DEGILDIR. Mutlak sayilar (apogee irtifasi, inis hizi)
%   ancak bunlari gercek verilerle degistirdiginde anlamli olur. Profilin
%   SEKLI (tirmanis -> tepe -> drogue -> ana -> inis) simdiden dogrudur.
%
%   Her satirin sonundaki [OLCULDU] / [TAHMIN] etiketini guncelle ki hangi
%   sayiya guvenecegin belli olsun.

% ====================================================================
%  ROKET — kutle ve gemetri
% ====================================================================
p.m_kalkis     = 25.0;      % kg    kalkis kutlesi (yakit dahil)      [TAHMIN]
p.m_yakit      = 5.0;       % kg    harcanan yakit kutlesi            [TAHMIN]
p.cap          = 0.150;     % m     govde capi                       [TAHMIN]
p.Cd           = 0.45;      % -     govde surukleme katsayisi         [TAHMIN]
p.CP_CG        = 1.2;       % m     basinc merkezi - agirlik merkezi  [TAHMIN]
p.Cn_alpha     = 12.0;      % 1/rad normal kuvvet egimi               [TAHMIN]
p.sonumleme    = 0.35;      % -     yunuslama sonumleme katsayisi     [TAHMIN]

% ====================================================================
%  MOTOR — itki egrisi
% ====================================================================
%  [zaman_s  itki_N] tablosu. Gercek .eng dosyandaki egriyi buraya tasi;
%  ara degerler dogrusal enterpolasyonla bulunur, tablonun disi 0 N'dur.
p.itki_egrisi = [ ...
    0.00     0
    0.05  2900
    0.50  2750
    1.00  2650
    2.00  2500
    3.00  2350
    3.80  1800
    4.00     0 ];                                                  % [TAHMIN]

% ====================================================================
%  PARASUTLER
% ====================================================================
p.drogue_CdA   = 0.60;      % m^2   drogue Cd*A                      [TAHMIN]
p.ana_CdA      = 9.00;      % m^2   ana parasut Cd*A                 [TAHMIN]
p.acilma_sure  = 0.60;      % s     sonlu acilma (sisme) suresi      [TAHMIN]

% ====================================================================
%  ORTAM
% ====================================================================
p.ruzgar       = 6.0;       % m/s   sabit yatay ruzgar (+x yonu)
p.rampa_acisi  = 3.0;       % derece rampanin dikeyden sapmasi
p.rampa_uzunlugu = 6.0;     % m     firlatma rampasi (ray) boyu
p.g            = 9.80665;   % m/s^2
p.rho0         = 1.225;     % kg/m^3 deniz seviyesi hava yogunlugu
p.olcek_yuk    = 8500;      % m     izotermal atmosfer olcek yuksekligi

% ====================================================================
%  SENSOR GURULTUSU
% ====================================================================
%  >>> BU BOLUM KALIBRE EDILMEDI <<<
%  Tasarim dokumani §6.3: bu degerler UYDURULMAZ, senin kartindan olculur.
%  Kart hareketsizken 5 dakikalik `ucusdebug` logu -> sigma, bias, drift
%  dogrudan cikar. O yapilana kadar esik eniyileme sonuclari GECICIDIR.
p.baro_sigma      = 0.8;    % m       barometre beyaz gurultusu       [TAHMIN]
p.baro_dinamik_k  = 0.0;    % -       statik delik dinamik basinc katsayisi.
                            %         0 = hata yok. Kotu delik yerlesimi
                            %         icin 0.02-0.10 arasi deneyebilirsin;
                            %         "sahte irtifa dususu" bu terimle dogar.
p.ivme_sigma      = 0.35;   % m/s^2   IMU lineer ivme gurultusu       [TAHMIN]
p.ivme_bias       = 0.10;   % m/s^2   IMU sabit bias                  [TAHMIN]
p.aci_sigma       = 0.30;   % derece  yonelim gurultusu               [TAHMIN]

% ====================================================================
%  SIMULASYON
% ====================================================================
p.dt_fizik     = 0.001;     % s   RK4 entegrasyon adimi (1 kHz)
p.dt_sensor    = 0.010;     % s   sensor + algoritma adimi (100 Hz)
p.t_bitis      = 400;       % s   guvenlik ust siniri
p.tohum        = 42;        % -   rastgele tohum (tekrarlanabilirlik icin)

% --- Turetilenler (elleme) ---
p.A            = pi * (p.cap/2)^2;
p.m_bos        = p.m_kalkis - p.m_yakit;
p.yanma_suresi = p.itki_egrisi(end,1);
end
