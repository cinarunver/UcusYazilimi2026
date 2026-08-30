function R = monte_carlo(ayar, N, tohumlar)
%MONTE_CARLO  Belirsizlik dagilimlari altinda N ucus kosar.
%
%   R = MONTE_CARLO(ayar, N)            N kosum, tohumlar 1..N
%   R = MONTE_CARLO(ayar, N, tohumlar)  belirli tohum kumesiyle
%
%   >>> ORTAK RASTGELE SAYILAR (CRN) <<<
%   `tohumlar` argumaninin varlik sebebi budur: farkli esik kumeleri AYNI
%   tohum kumesiyle kosulmalidir. Aksi halde Monte Carlo gurultusu esikler
%   arasi farki orter ve karsilastirma anlamsiz cikar. eniyile.m bu yuzden
%   her aday icin ayni `tohumlar` vektorunu gecirir.
%
%   Donen R: her kosum icin bir satir (tablo) + ozet istatistikler.

    if nargin < 1 || isempty(ayar), ayar = struct(); end
    if nargin < 2 || isempty(N),    N = 200;          end
    if nargin < 3 || isempty(tohumlar), tohumlar = 1:N; end
    N = numel(tohumlar);

    % --- Yol kurulumu (bkz. oyna.m'deki aciklama) ---
    buradan = fileparts(mfilename('fullpath'));
    addpath(buradan, fullfile(buradan, 'config'));
    p0 = roket_params();

    % ANA PARASUT ACILIS HIZI LIMITI [m/s].
    % Model her parasutun her acilista SAGLAM kaldigini varsayar; yirtilmayi
    % modellemez. Bu yuzden "indi mi?" tek basina basari olcusu DEGILDIR:
    % drogue kacip ana parasut balistik hizda acilan kosumlarda simulasyon
    % yumusak inis raporlar, gercek donanim raporlamaz. Nominal acilis
    % 15-18 m/s; limit oranin kendisini degil, guvenli tarafi isaretler.
    ANA_ACILIS_LIMIT = 30;

    apogee = zeros(N,1); drogue_z = zeros(N,1); gecikme = nan(N,1);
    ana_z  = nan(N,1);   inis = zeros(N,1);     surukl = zeros(N,1);
    tilt_at = nan(N,1);  vz_at = nan(N,1);      ana_hiz = nan(N,1);
    basarili = false(N,1);

    for i = 1:N
        p = ornekle(p0, tohumlar(i));
        s = sim_ucus(p, ayar);

        apogee(i) = s.apogee;  drogue_z(i) = s.drogue_z;
        gecikme(i)= s.apogee_gecikme;  ana_z(i) = s.ana_z;
        inis(i)   = s.inis_hizi;  surukl(i) = s.suruklenme;
        ana_hiz(i)= s.ana_hiz;

        if ~isnan(s.drogue_emir_t)
            j = find(s.t >= s.drogue_emir_t, 1);
            if ~isempty(j), tilt_at(i) = s.tilt(j); vz_at(i) = s.vz(j); end
        end
        % GERCEKCI basari: indi + inis hizi makul + ana parasut hayatta
        % kalabilecegi bir hizda acildi.
        basarili(i) = ~isnan(s.ana_t) && s.inis_hizi <= 10 && ...
                      s.ana_hiz <= ANA_ACILIS_LIMIT;
    end

    R.tablo = table(tohumlar(:), apogee, drogue_z, gecikme, ana_z, inis, ...
                    surukl, tilt_at, vz_at, ana_hiz, basarili, ...
        'VariableNames', {'Tohum','Apogee','DrogueZ','Gecikme','AnaZ','Inis', ...
                          'Suruklenme','TiltAtes','VzAtes','AnaHiz','Basarili'});

    % --- Sert kisit ihlalleri (tasarim dokumani §8.2) ---
    R.n              = N;
    R.drogue_kacirma = sum(isnan(drogue_z));
    R.ana_kacirma    = sum(isnan(ana_z));
    R.hizli_inis     = sum(inis > 10);
    R.yukselirken    = sum(vz_at > 5);
    R.ana_asiri_hiz  = sum(ana_hiz > ANA_ACILIS_LIMIT);
    R.ana_hiz_limit  = ANA_ACILIS_LIMIT;
    R.ana_hiz_p95    = prctile(ana_hiz(~isnan(ana_hiz)), 95);
    R.basari_orani   = mean(basarili);
    % Eski (yaniltici) olcu: yalnizca "indi mi". Karsilastirma icin duruyor —
    % aradaki fark, modelin parasut yirtilmasini gormemesinden gelir.
    R.inis_orani     = mean(~isnan(ana_z) & inis <= 10);

    % --- Amac fonksiyonu terimleri (§8.3) ---
    g = gecikme(~isnan(gecikme));
    R.gecikme_ort = mean(abs(g));
    R.gecikme_p95 = prctile(abs(g), 95);
    R.ana_hata    = mean(abs(ana_z(~isnan(ana_z)) - 550));
    R.suruklenme_ort = mean(surukl);
    R.apogee_ort  = mean(apogee);
end

% =====================================================================
function p = ornekle(p, tohum)
%ORNEKLE  Belirsiz parametreleri dagilimlarindan orneklendirir.
%
%   Hangi buyuklugun ne kadar belirsiz oldugu MUHENDISLIK KARARIDIR.
%   Asagidaki dagilimlar makul baslangiclardir; olcum yapildikca daralt.

    rs = RandStream('twister', 'Seed', tohum);

    % Motor performansi: toplam impuls +-5 %
    imp_carpan = 1 + 0.05*randn(rs);
    p.itki_egrisi(:,2) = p.itki_egrisi(:,2) * max(0.7, imp_carpan);

    % Kutle: +-2 %
    p.m_kalkis = p.m_kalkis * (1 + 0.02*randn(rs));
    p.m_bos    = p.m_kalkis - p.m_yakit;

    % Surukleme katsayisi: +-10 % (en belirsiz aerodinamik buyukluk)
    p.Cd = p.Cd * (1 + 0.10*randn(rs));

    % Ruzgar: 0-14 m/s, yon +-
    p.ruzgar = abs(7 + 4*randn(rs)) * sign(randn(rs));

    % Rampa acisi: 5 +- 2 derece
    p.rampa_acisi = 5 + 2*randn(rs);

    % Parasut Cd*A: +-8 %
    p.drogue_CdA = p.drogue_CdA * (1 + 0.08*randn(rs));
    p.ana_CdA    = p.ana_CdA    * (1 + 0.08*randn(rs));

    % Statik delik kalitesi: dinamik basinc katsayisi 0-0.08
    p.baro_dinamik_k = abs(0.03*randn(rs));

    % Sensor gurultusu — DIKKAT: bu dagilimlar KALIBRE EDILMEDI (§6.3)
    p.baro_sigma = abs(p.baro_sigma * (1 + 0.3*randn(rs)));
    p.ivme_sigma = abs(p.ivme_sigma * (1 + 0.3*randn(rs)));

    % Kumulatif impuls yeniden hesaplanmali (itki egrisi olceklendi)
    tt = p.itki_egrisi(:,1); ff = p.itki_egrisi(:,2);
    kum = [0; cumsum(0.5*(ff(1:end-1)+ff(2:end)) .* diff(tt))];
    p.impuls_toplam = kum(end);
    p.impuls_kum = @(t) interp1(tt, kum, min(max(t,tt(1)),tt(end)), 'linear');

    p.tohum = tohum;   % sim ici gurultu de tohuma bagli
end
