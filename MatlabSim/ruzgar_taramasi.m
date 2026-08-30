function T = ruzgar_taramasi(rampa_acilari, ruzgarlar)
%RUZGAR_TARAMASI  Rampa egim YONU x ruzgar YONU -> apogee egimi -> drogue karari.
%
%   T = RUZGAR_TARAMASI()                        varsayilan izgara
%   T = RUZGAR_TARAMASI([0 5 11], -12:6:12)      ozel izgara
%
%   NEDEN VAR (Bulgu 7):
%   Monte Carlo'da drogue kacirma orani %32.5 cikti. Bu oran RASTGELE
%   degildir; tek bir mekanizmadan gelir:
%
%     Weathercocking burnu ruzgara cevirir.
%       - Rampa RUZGARA KARSI egikse, bu donme yercekimi donusunu SONDURUR
%         -> apogee egimi kucuk -> drogue acilir.
%       - Rampa RUZGAR YONUNE egikse, ikisi TOPLANIR
%         -> apogee egimi buyur -> MAX_EGLIM kapisi ateslemeyi keser.
%
%   monte_carlo.m ruzgar yonunu `sign(randn)` ile yazi-tura sectigi icin
%   kosumlarin yarisi kotu tarafa dusuyor ve oran oradan cikiyor.
%
%   ISARET KURALI: negatif ruzgar = rampanin egik oldugu YONE esiyor.
%
%   GECERLILIK: Etkinin YONU kararlilik fiziginden gelir ve modele bagli
%   degildir; bu yuzden "rampayi ruzgara karsi eg" sonucu guvenilirdir.
%   75 derecelik kapinin TAM olarak nerede asildigi ise Bulgu 3 kapsaminda
%   belirsizdir — model apogee civari egimi DUSUK tahmin ediyor, dolayisiyla
%   asagidaki tablo iyimser taraftadir.

    if nargin < 1 || isempty(rampa_acilari), rampa_acilari = [0 3 5 8 11]; end
    if nargin < 2 || isempty(ruzgarlar),     ruzgarlar = [-18 -12 -6 0 6 12 18]; end

    % --- Yol kurulumu (bkz. oyna.m'deki aciklama) ---
    buradan = fileparts(mfilename('fullpath'));
    addpath(buradan, fullfile(buradan, 'config'));
    p0 = roket_params();

    nr = numel(rampa_acilari); nw = numel(ruzgarlar);
    rampa = []; ruzgar = []; egim_ap = []; drogue = []; ana_h = []; apo = [];

    fprintf('\n=== APOGEE EGIMI [derece] — satir: rampa acisi, sutun: ruzgar ===\n');
    fprintf('   (negatif ruzgar = rampanin egik oldugu YONE esiyor)\n\n');
    fprintf('%10s', 'rampa\ruz'); fprintf('%9d', ruzgarlar); fprintf('\n');

    for a = 1:nr
        fprintf('%10.0f', rampa_acilari(a));
        for w = 1:nw
            p = p0;  p.rampa_acisi = rampa_acilari(a);  p.ruzgar = ruzgarlar(w);
            s = sim_ucus(p);
            [~, ia] = max(s.z);

            rampa(end+1,1)   = rampa_acilari(a);   %#ok<AGROW>
            ruzgar(end+1,1)  = ruzgarlar(w);       %#ok<AGROW>
            egim_ap(end+1,1) = s.tilt(ia);         %#ok<AGROW>
            drogue(end+1,1)  = ~isnan(s.drogue_z); %#ok<AGROW>
            ana_h(end+1,1)   = s.ana_hiz;          %#ok<AGROW>
            apo(end+1,1)     = s.apogee;           %#ok<AGROW>

            im = ' ';
            if ~isnan(s.drogue_z), im = '*'; end   % * = drogue acildi
            fprintf('%8.1f%s', s.tilt(ia), im);
        end
        fprintf('\n');
    end
    fprintf('\n   * = drogue acildi (isaretsiz = MAX_EGLIM kapisi kesti)\n');

    T = table(rampa, ruzgar, apo, egim_ap, drogue, ana_h, ...
        'VariableNames', {'RampaAcisi_deg','Ruzgar_ms','Apogee_m', ...
                          'EgimApogee_deg','DrogueAcildi','AnaAcilisHiz_ms'});

    kacan = T(~T.DrogueAcildi, :);
    fprintf('\n   %d/%d izgara noktasinda drogue KACTI.\n', height(kacan), height(T));
    if ~isempty(kacan)
        fprintf('   Kacan noktalarda ana parasut acilis hizi: %.0f - %.0f m/s (limit 30)\n', ...
                min(kacan.AnaAcilisHiz_ms), max(kacan.AnaAcilisHiz_ms));
        fprintf('   Hepsinde ruzgar isareti rampa egimiyle AYNI yonde mi: %d\n', all(kacan.Ruzgar_ms < 0));
    end
    fprintf('\n');
end
