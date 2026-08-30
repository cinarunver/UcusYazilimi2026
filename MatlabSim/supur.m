function T = supur(alan, degerler, N)
%SUPUR  Tek bir esigi Monte Carlo altinda supurur (duyarlilik analizi).
%
%   T = SUPUR('apogee_irtifa_farki', [5 10 15 20 25], 100)
%
%   >>> ORTAK RASTGELE SAYILAR (CRN) <<<
%   Her esik degeri AYNI tohum kumesiyle kosulur. Bu olmadan Monte Carlo
%   gurultusu esikler arasi farki orter: 100 kosumda apogee gecikmesinin
%   standart hatasi, esigi 5 m degistirmenin etkisinden buyuk olabilir.
%   CRN ile fark dogrudan esikten gelir.

    if nargin < 3 || isempty(N), N = 60; end

    % --- Yol kurulumu (bkz. oyna.m'deki aciklama) ---
    buradan = fileparts(mfilename('fullpath'));
    addpath(buradan, fullfile(buradan, 'config'));

    tohumlar = 1:N;                       % <-- CRN: tum adaylarda AYNI

    n = numel(degerler);
    deger = degerler(:);
    drogue_kacirma = zeros(n,1); ana_kacirma = zeros(n,1);
    yukselirken = zeros(n,1);    hizli_inis = zeros(n,1);
    gecikme_ort = nan(n,1);      gecikme_p95 = nan(n,1);
    ana_hata = nan(n,1);         basari = zeros(n,1);
    ana_asiri = zeros(n,1);      kisit_ok = strings(n,1);

    fprintf('\n  %s supuruluyor (%d deger x %d kosum, CRN)\n\n', alan, n, N);
    for i = 1:n
        ayar = struct(alan, degerler(i));
        R = monte_carlo(ayar, N, tohumlar);

        drogue_kacirma(i) = R.drogue_kacirma;
        ana_kacirma(i)    = R.ana_kacirma;
        yukselirken(i)    = R.yukselirken;
        hizli_inis(i)     = R.hizli_inis;
        gecikme_ort(i)    = R.gecikme_ort;
        gecikme_p95(i)    = R.gecikme_p95;
        ana_hata(i)       = R.ana_hata;
        basari(i)         = 100*R.basari_orani;
        ana_asiri(i)      = R.ana_asiri_hiz;

        % Sert kisitlar (§8.2): ihlal -> aday REDDEDILIR, cezalandirilmaz.
        % ana_asiri_hiz de sert kisittir: ana parasutun balistik hizda
        % acilmasi "kurtarma" sayilamaz (bkz. Bulgu 7).
        if R.ana_kacirma > 0 || R.hizli_inis > 0 || R.yukselirken > 0 || R.ana_asiri_hiz > 0
            kisit_ok(i) = "RED";
        else
            kisit_ok(i) = "gecerli";
        end

        fprintf('    %-8g  drogue_kacirma=%3d  ana_asiri_hiz=%3d  gecikme=%5.2f/%5.2f s  ana_hata=%5.1f m  %s\n', ...
                degerler(i), R.drogue_kacirma, R.ana_asiri_hiz, R.gecikme_ort, ...
                R.gecikme_p95, R.ana_hata, kisit_ok(i));
    end

    T = table(deger, drogue_kacirma, ana_kacirma, ana_asiri, yukselirken, hizli_inis, ...
              gecikme_ort, gecikme_p95, ana_hata, basari, kisit_ok, ...
        'VariableNames', {alan,'DrogueKacirma','AnaKacirma','AnaAsiriHiz', ...
                          'YukselirkenAtes','HizliInis','GecikmeOrt','GecikmeP95', ...
                          'AnaHata_m','Basari_yuzde','Kisit'});
    fprintf('\n');
    disp(T);
end
