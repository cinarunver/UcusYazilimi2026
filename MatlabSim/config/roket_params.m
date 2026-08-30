function p = roket_params()
%ROKET_PARAMS  Simulasyonun TUM fiziksel girdileri tek yerde.
%
%   Burasi senin oynayacagin ilk dosya. Roketin gercek degerlerini buraya
%   yazdiginda simulasyon o rokete gore kosar.
%
%   KAYNAK: config/rocket.ork — OpenRocket 24.12, roket "Akdogan".
%   Asagidaki [ORK] etiketli degerler o dosyadan DOGRUDAN cikarilmistir.
%   [TAHMIN] etiketliler hala benim varsayimimdir; olctugunde guncelle.
%
%   OpenRocket'in kendi simulasyon sonucu (dogrulama referansi):
%       apogee 3703.3 m @ 27.81 s | maks hiz 276.8 m/s | maks ivme 94.6 m/s^2
%       rampa cikis hizi 31.0 m/s | yere carpma 13.2 m/s | ucus 310.9 s

% ====================================================================
%  ROKET — kutle ve geometri
% ====================================================================
p.m_kalkis     = 23.166;    % kg    kalkis kutlesi (yakit dahil)      [ORK]
p.m_yakit      =  4.103;    % kg    harcanan yakit kutlesi            [ORK]
p.cap          =  0.115;    % m     govde capi (aftradius 0.0575*2)   [ORK]
p.uzunluk      =  2.75;     % m     burun 0.32 + ust 1.40 + alt 1.03  [ORK]
p.Cd           =  0.500;    % -     govde surukleme katsayisi   [KALIBRE EDILDI]
                            %       OpenRocket apogee'sine (3703.3 m) gore
                            %       supuruldu; hucum acisina bagli alan
                            %       modeli eklendikten sonra yeniden ayarlandi.
p.CP_CG        =  0.27;     % m     basinc merkezi - agirlik merkezi     [ORK]
                            %       .ork: CP=1.626 m, CG=1.356 m (apogee civari).
                            %       Onceki 0.60 tahmini weathercocking'i IKI KAT
                            %       abartip apogee egimini 52 dereceye cikariyordu
                            %       (OpenRocket referansi 24.9 derece).
p.Cn_alpha     =  6.4;      % 1/rad normal kuvvet egimi                  [ORK]
                            %       .ork t=20s: CN=0.112 @ AoA=1.0 deg -> 6.4/rad
p.J            = 10.05;     % kg*m^2 boyuna atalet momenti               [ORK]
p.sonumleme    =  0.35;     % -     yunuslama sonumleme katsayisi     [TAHMIN]
p.aero_moment_min_hiz = 5;  % m/s   bu hizin altinda aerodinamik moment yok

% --- EGIM DUZELTME KATSAYISI (model sinirinin telafisi) --------------
%  2D yunuslama modeli apogee civarindaki egimi DUSUK tahmin ediyor:
%      OpenRocket referansi (6 m/s ruzgar)  t=27.8 s -> 24.7 derece
%      bu model, fiziksel Cn_alpha=6.4 ile  t=27.8 s -> 11.8 derece
%  Sebep: apogee civarinda dinamik basinc cok dusuk ve hucum acisi buyuk;
%  bu rejimi 2D dogrusal moment modeli yakalayamiyor. Cn_alpha'yi 35'e
%  cikarmak sayiyi tutturuyor ama fiziksel degil (.ork'tan gercek deger 6.4)
%  ve tirmanis egimini bozuyor.
%
%  ONEMLI: egimi DUSUK tahmin etmek, MAX_EGLIM guvenlik kapisinin marjini
%  OLDUGUNDAN GENIS gosterir — yani tehlikeli yonde yanilir. Bu yuzden
%  ALGORITMAYA GIDEN egim olcumu bu katsayiyla olceklenir; fizik (yorunge,
%  surukleme) olceklenmemis gercek aciyi kullanmaya devam eder.
%
%  >>> VARSAYILAN 1.0 — DUZELTME KAPALI <<<
%  Katsayi TEK bir kosulda (6 m/s ruzgar, nominal) kalibre edildi ve her
%  kosula uygulanmasi savunulamaz: dusuk ruzgarli kosumlarda bile gorulen
%  egimi 98 dereceye cikarip kapiyi haksiz yere kapatiyor. Olculen duyarlilik
%  (20 kosum, ortak tohum):
%       duzeltme 1.00 -> drogue kacirma  6/20
%       duzeltme 1.50 -> drogue kacirma 16/20
%       duzeltme 2.09 -> drogue kacirma 16/20
%  Yani MAX_EGLIM marjina dair her sonuc bu katsayiya bagli. Bu model o
%  soruyu CEVAPLAYAMAZ (bkz. BULGULAR.md, Bulgu 3). Katsayiyi duyarlilik
%  incelemesi icin degistirebilirsin; varsayilan olarak fizik bozulmaz.
p.egim_duzeltme = 1.0;
p.Cd_yan       =  1.20;     % -     yanal (capraz akis) surukleme kats. [TAHMIN]
                            %       Silindir capraz akisi ~1.2. Takla halinde
                            %       govde yandan gelir; etkin alan eksenel
                            %       alanin ~30 katidir, bu terim onu modeller.

% ====================================================================
%  MOTOR — TFM1850W (Teknofest26)
% ====================================================================
%  [zaman_s  itki_N]. OpenRocket ucus verisinden cikarildi.            [ORK]
%  Toplam impuls 7649 N*s, yanma suresi 6.74 s, maks itki 2411 N.
p.itki_egrisi = [ ...
    0.000      0.0
    0.070   1622.8
    0.140   2315.5
    0.210   2132.7
    0.280   2051.9
    0.350   2009.5
    0.419   2001.9
    0.719   2071.6
    1.068   2084.3
    1.418   1924.1
    1.768   1830.9
    2.117   1778.1
    2.467   1691.1
    2.816   1578.0
    3.165   1459.3
    3.504   1312.7
    3.815   1015.7
    4.140    884.2
    4.473    570.0
    4.823    428.8
    5.173    298.1
    5.522    159.9
    5.872     65.9
    6.222     24.1
    6.571      7.9
    6.741      0.0 ];

% ====================================================================
%  PARASUTLER
% ====================================================================
%  Cd*A degerleri OpenRocket'in ANA GOVDE ("Govdeler") dalindaki kararli
%  inis hizlarindan geri hesaplandi — capdan tahmin DEGIL, olculmus davranis:
%      drogue fazi : vz = -15.62 m/s, m = 13.93 kg, rho = 0.914  -> Cd*A = 1.226
%      ana fazi    : vz =  -6.36 m/s, m = 13.93 kg, rho = 1.092  -> Cd*A = 6.185
p.drogue_CdA   = 1.226;     % m^2   surukleme parasutu (cap 1.4 m)    [ORK]
p.ana_CdA      = 6.185;     % m^2   ana parasut (cap 2.8 m)           [ORK]
p.acilma_sure  = 0.60;      % s     sonlu acilma (sisme) suresi      [TAHMIN]

% Apogee'de govde ayrilir; UKB'yi tasiyan ana govde bu kutleyle iner.
% (Burun/gorev yuku bolumu 5.13 kg ile ayri iner — bu sim onu izlemez.)
p.m_ayrilma    = 13.93;     % kg    ayrilma sonrasi ana govde kutlesi [ORK]

% ====================================================================
%  ORTAM  — OpenRocket firlatma kosullari
% ====================================================================
p.ruzgar       = 6.0;       % m/s   ortalama yatay ruzgar             [ORK]
p.rampa_acisi  = 5.0;       % derece rampanin dikeyden sapmasi        [ORK]
p.rampa_uzunlugu = 6.0;     % m     firlatma rampasi (ray) boyu       [ORK]
p.kalkis_rakimi = 970.0;    % m     firlatma noktasi rakimi           [ORK]
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
p.dt_fizik     = 0.001;     % s   RK4 adimi — tirmanis/apogee (1 kHz)
p.dt_fizik_inis= 0.010;     % s   RK4 adimi — parasut altinda (100 Hz).
                            %     Inis dinamigi yavas; ucus suresinin %90'i
                            %     bu fazda gectigi icin kosumu ~5x hizlandirir.
p.dt_sensor    = 0.010;     % s   sensor + algoritma adimi (100 Hz)
p.t_bitis      = 400;       % s   guvenlik ust siniri
p.tohum        = 42;        % -   rastgele tohum (tekrarlanabilirlik icin)

% --- Turetilenler (elleme) ---
p.A            = pi * (p.cap/2)^2;       % eksenel (on) alan
p.A_yan        = p.uzunluk * p.cap;      % yanal (profil) alan
p.m_bos        = p.m_kalkis - p.m_yakit;
p.yanma_suresi = p.itki_egrisi(end,1);

% Kumulatif impuls: yakit tuketimini zamanla degil harcanan impulsla
% orantili modellemek icin (itki egrisi one yuklu).
tt = p.itki_egrisi(:,1);  ff = p.itki_egrisi(:,2);
kum = [0; cumsum(0.5*(ff(1:end-1)+ff(2:end)) .* diff(tt))];
p.impuls_toplam = kum(end);
p.impuls_kum    = @(t) interp1(tt, kum, min(max(t,tt(1)),tt(end)), 'linear');
end
