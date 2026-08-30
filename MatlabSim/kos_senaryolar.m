function T = kos_senaryolar(ayar, ciz_mi)
%KOS_SENARYOLAR  Dogrulama senaryosu kutuphanesini kosar ve tablo uretir.
%
%   T = KOS_SENARYOLAR()            varsayilan esiklerle
%   T = KOS_SENARYOLAR(ayar)        ozel esiklerle
%   T = KOS_SENARYOLAR(ayar, true)  ayrica her senaryonun grafigini kaydeder
%
%   Cikti tablosu dogrudan mühendislik tasarim raporuna girer.

    if nargin < 1, ayar = struct(); end
    if nargin < 2, ciz_mi = false; end

    % --- Yol kurulumu (bkz. oyna.m'deki aciklama) ---
    buradan = fileparts(mfilename('fullpath'));
    addpath(buradan, fullfile(buradan, 'config'));
    p0 = roket_params();
    S  = senaryolar();
    n  = numel(S);

    ad = strings(n,1); apogee = zeros(n,1); drogue_z = zeros(n,1);
    gecikme = zeros(n,1); ana_z = zeros(n,1); inis = zeros(n,1);
    surukl = zeros(n,1); sonuc = strings(n,1); not_ = strings(n,1);
    ana_hiz = nan(n,1);

    fprintf('\n');
    for i = 1:n
        p = uygula(p0, S(i).p_deg);
        s = sim_ucus(p, ayar, S(i).ariza);

        ad(i)      = S(i).ad;
        apogee(i)  = s.apogee;
        drogue_z(i)= s.drogue_z;
        gecikme(i) = s.apogee_gecikme;
        ana_z(i)   = s.ana_z;
        inis(i)    = s.inis_hizi;
        surukl(i)  = s.suruklenme;
        ana_hiz(i) = s.ana_hiz;

        [ok, aciklama] = degerlendir(s, S(i));
        sonuc(i) = ok;  not_(i) = aciklama;

        fprintf('  %-28s %s  %s\n', S(i).ad, pad(ok), aciklama);

        if ciz_mi
            ciz(s, sprintf('%d. %s', i, S(i).ad), ...
                fullfile(buradan,'cikti','senaryo',sprintf('%02d_%s.png', i, dosyaadi(S(i).ad))));
        end
    end

    T = table(ad, apogee, drogue_z, gecikme, ana_z, ana_hiz, inis, surukl, sonuc, not_, ...
        'VariableNames', {'Senaryo','Apogee_m','Drogue_m','Gecikme_s', ...
                          'Ana_m','AnaAcilisHiz_ms','Inis_ms','Suruklenme_m','Sonuc','Not'});

    fprintf('\n');
    disp(T(:, {'Senaryo','Apogee_m','Drogue_m','Gecikme_s','Ana_m','AnaAcilisHiz_ms','Inis_ms','Sonuc'}));

    fprintf('\n  GECTI: %d   INCELE: %d   KALDI: %d   (toplam %d)\n\n', ...
            sum(sonuc == "GECTI"), sum(sonuc == "INCELE"), sum(sonuc == "KALDI"), n);
end

% =====================================================================
function p = uygula(p, deg)
%UYGULA  Senaryo degisikliklerini parametrelere isler.
    if isempty(deg) || ~isstruct(deg), return; end
    f = fieldnames(deg);
    for i = 1:numel(f)
        if strcmp(f{i}, 'motor_kes')
            % Itki egrisini verilen anda kes (motor erken sonmesi)
            tkes = deg.motor_kes;
            e = p.itki_egrisi(p.itki_egrisi(:,1) < tkes, :);
            p.itki_egrisi = [e; tkes 0];
            p.yanma_suresi = tkes;
            tt = p.itki_egrisi(:,1); ff = p.itki_egrisi(:,2);
            kum = [0; cumsum(0.5*(ff(1:end-1)+ff(2:end)) .* diff(tt))];
            p.impuls_toplam = kum(end);
            p.impuls_kum = @(t) interp1(tt, kum, min(max(t,tt(1)),tt(end)), 'linear');
            % Yanmayan yakit kutlede kalir
            p.m_yakit = p.m_yakit * (kum(end) / 7649);
            p.m_bos   = p.m_kalkis - p.m_yakit;
        else
            p.(f{i}) = deg.(f{i});
        end
    end
end

% =====================================================================
function [ok, aciklama] = degerlendir(s, sen)
%DEGERLENDIR  Senaryonun sert kisitlarini kontrol eder.
%   Bir senaryonun "GECTI" olmasi kurtarmanin calistigi anlamina gelir;
%   "INCELE" beklenen ama dikkat isteyen bir sonuc; "KALDI" ise kurtarma
%   basarisizligi demektir.

    ihlal = strings(0);
    if isnan(s.ana_t),       ihlal(end+1) = "ana parasut acilmadi"; end
    if s.inis_hizi > 10,     ihlal(end+1) = sprintf("inis %.1f m/s", s.inis_hizi); end
    % Ana parasutun balistik hizda acilmasi kurtarma SAYILMAZ: model
    % yirtilmayi gormez, gercek donanim gorur (bkz. Bulgu 7).
    if ~isnan(s.ana_hiz) && s.ana_hiz > 30
        ihlal(end+1) = sprintf("ana parasut %.0f m/s'de acildi", s.ana_hiz);
    end

    % Yukselirken veya asiri egimde atesleme = guvenlik ihlali
    if ~isnan(s.drogue_emir_t)
        j = find(s.t >= s.drogue_emir_t, 1);
        if ~isempty(j)
            if s.vz(j) > 5,   ihlal(end+1) = sprintf("yukselirken atesleme (Vz=%+.0f)", s.vz(j)); end
        end
    end

    if ~isempty(ihlal)
        ok = "KALDI";
        aciklama = strjoin(ihlal, "; ");
        return;
    end

    % Kurtarma calisti. Simdi DIKKAT isteyen ama basarisizlik olmayan
    % durumlar: sert kisit degil, muhendislik uyarisi.
    uyari = strings(0);
    if ~isnan(s.ana_t) && abs(s.ana_z - 550) > 50
        uyari(end+1) = sprintf("ana parasut GERCEKTE %.0f m (hedef 550)", s.ana_z);
    end
    if ~isnan(s.apogee_gecikme) && abs(s.apogee_gecikme) > 5
        uyari(end+1) = sprintf("drogue %.1f s gec — acilis hizi yuksek", s.apogee_gecikme);
    end

    if ~isempty(uyari)
        ok = "INCELE";
        aciklama = strjoin(uyari, "; ");
    else
        ok = "GECTI";
        if isnan(s.drogue_emir_t)
            aciklama = "drogue emri yok — ana parasut yedek tetikle acildi";
        elseif s.apogee < 550
            aciklama = "apogee taban altinda; arama kapisi calisti";
        else
            aciklama = sprintf("gecikme %+.2f s", s.apogee_gecikme);
        end
    end
end

% =====================================================================
function s = pad(x)
    s = sprintf('%-7s', x);
end

function d = dosyaadi(ad)
    d = lower(ad);
    d = regexprep(d, '[^a-z0-9]+', '_');
    d = regexprep(d, '_+$', '');
end
