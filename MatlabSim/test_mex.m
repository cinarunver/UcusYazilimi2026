function test_mex()
%TEST_MEX  MEX koprusunun dogrulanmasi (F2 bitis kaniti).
%
%   Bu testler test/test_ucus_algo/test_algo.cpp icindeki native C++
%   senaryolarinin BIREBIR MATLAB karsiligidir. Ayni senaryo, ayni esikler,
%   ayni beklenen sonuc. Ikisi de ayni ucus_algoritmasi.h dosyasini derledigi
%   icin uyusmamalari koprude bir hata oldugu anlamina gelir.
%
%   Kullanim:  cd MatlabSim; test_mex

    fprintf('=== ucus_mex kopru dogrulamasi ===\n');
    gecen = 0; kalan = 0;

    % --- 0) Kopru ayakta mi, boyutlar tutarli mi ---
    b = ucus_mex('bilgi');
    [gecen, kalan] = onay(b.hal_bayt > 0, 'kopru yanit veriyor', gecen, kalan);

    hal = ucus_mex('sifirla');
    [gecen, kalan] = onay(numel(hal) == b.hal_bayt && isa(hal,'uint8'), ...
        'sifirla dogru boyutta opak hal donduruyor', gecen, kalan);

    % --- 1) Baslangic durumu HAZIR ---
    [~, c] = ucus_mex('adim', hal, ucus_girdi(0, [0 0 0], [0 0 0], [1 0 0 0], 1000000, true, true));
    [gecen, kalan] = onay(c.durum == 0, 'baslangic durumu HAZIR', gecen, kalan);

    % --- 2) Kalkis (Kalman zayiflatmasi nedeniyle tek adimda DEGIL) ---
    hal = ucus_mex('sifirla'); t = 1000000;
    [hal, c] = adim(hal, 0, 0, 0, t);
    [gecen, kalan] = onay(c.durum == 0, 'ilk adim sonrasi hala HAZIR', gecen, kalan);
    [hal, t] = kalkis_yap(hal, t);
    [~, c]   = adim(hal, 0, 0, 0, t);
    [gecen, kalan] = onay(c.durum == 1, 'kalkis tespit edildi (YUKSELIYOR)', gecen, kalan);

    % --- 3) REGRESYON: tek adimlik 25 m/s^2 kalkis URETMEZ ---
    hal = ucus_mex('sifirla'); t = 1000000;
    [hal, ~] = adim(hal, 0, 0, 0, t);   t = t + 10000;
    [~, c]   = adim(hal, 0, 25, 0, t);
    [gecen, kalan] = onay(c.durum == 0, ...
        'tek adimlik 25 m/s^2 kalkis uretmiyor (Kalman zayiflatmasi)', gecen, kalan);

    % --- 4) Apogee tam kosul: T && A && B && D ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 600, 0, 5, t);
    [hal, t, c] = oturt(hal, 580, 0, 5, t);
    [gecen, kalan] = onay(c.durum == 2, 'apogee yakalandi -> INIS_1', gecen, kalan);

    % --- 5) Egim kapisi 100 derecede atesleme engelliyor ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 600, 0, 100, t);
    [~, ~, c] = oturt(hal, 580, 0, 100, t);
    [gecen, kalan] = onay(c.durum == 1, ...
        'egim 100 derece: apogee sayilmadi (guvenlik kapisi)', gecen, kalan);

    % --- 6) Egim 45 derecede GECIYOR (esik 75) ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 600, 0, 45, t);
    [~, ~, c] = oturt(hal, 580, 0, 45, t);
    [gecen, kalan] = onay(c.durum == 2, 'egim 45 derece: apogee gecti', gecen, kalan);

    % --- 7) REGRESYON: rampada negatif irtifa ile atesleme yok ---
    [hal, t] = ucus_baslat();
    [~, ~, c] = oturt(hal, -20, 0, 0, t);
    [gecen, kalan] = onay(c.durum == 1, ...
        'rampada negatif irtifa: arama tabani (T) atesleme kesiyor', gecen, kalan);

    % --- 8) Arama tabani siniri: 550 gecilmeden apogee aranmaz ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 540, 0, 5, t);
    [~, ~, c] = oturt(hal, 500, 0, 5, t);
    [gecen, kalan] = onay(c.durum == 1, ...
        'taban altinda (540 m) apogee aranmadi', gecen, kalan);

    % --- 9) Ana parasut yedek tetigi: apogee kacti ama ana acildi ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 700, 0, 120, t);
    [~, ~, c] = oturt(hal, 400, 0, 120, t);
    [gecen, kalan] = onay(c.durum == 3 && bitand(c.durum_bitleri, 128) > 0, ...
        'apogee kacti -> yedek tetik ana parasutu acti', gecen, kalan);
    [gecen, kalan] = onay(bitand(c.durum_bitleri, 32) == 0, ...
        '  ... drogue emri VERILMEDI (bit5 sonuk)', gecen, kalan);

    % --- 10) Yedek tetik yukselirken atesleme yapmiyor ---
    [hal, t] = ucus_baslat();
    for irt = 300:10:600
        t = t + 10000;
        [hal, c] = adim(hal, irt, 0, 120, t);
    end
    [gecen, kalan] = onay(bitand(c.durum_bitleri, 128) == 0, ...
        'yukselirken yedek tetik ateslenmiyor', gecen, kalan);

    % --- 11) Funye emri KENAR: yalniz bir adim true ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 600, 0, 5, t);
    emir_sayisi = 0;
    for i = 1:60
        t = t + 10000;
        [hal, c] = adim(hal, 580, 0, 5, t);
        if c.funye1_emir, emir_sayisi = emir_sayisi + 1; end
    end
    [gecen, kalan] = onay(emir_sayisi == 1, ...
        'funye1 emri tam bir kez (kenar, seviye degil)', gecen, kalan);

    % --- 12) Gozlem biti emirden bagimsiz ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 700, 0, 120, t);
    [~, ~, c] = oturt(hal, 650, 0, 120, t);
    [gecen, kalan] = onay(bitand(c.durum_bitleri, 16) > 0 && bitand(c.durum_bitleri, 32) == 0, ...
        'alcalma gozlemi yandi ama drogue emri yanmadi', gecen, kalan);

    % --- 13) DURUMSUZLUK: ayni hal'den iki kez adim ayni sonucu vermeli ---
    [hal, t] = ucus_baslat();
    [hal, t] = oturt(hal, 600, 0, 5, t);
    g = ucus_girdi(580, [0 0 0], [5 0 0], [1 0 0 0], t + 10000, true, true);
    [halA, cA] = ucus_mex('adim', hal, g);
    [halB, cB] = ucus_mex('adim', hal, g);
    [gecen, kalan] = onay(isequal(halA, halB) && isequal(cA, cB), ...
        'MEX durumsuz: ayni hal + ayni girdi -> ayni cikti (parfor guvenli)', gecen, kalan);

    % --- 14) Esik supurulebiliyor (F4 onkosulu) ---
    ayar = struct('max_eglim', 30);        % kapiyi daralt
    hal2 = ucus_mex('sifirla', ayar);
    t2 = 1000000;
    [hal2, t2] = kalkis_yap(hal2, t2);
    [hal2, t2] = oturt(hal2, 600, 0, 45, t2);
    [~, ~, c2] = oturt(hal2, 580, 0, 45, t2);
    [gecen, kalan] = onay(c2.durum == 1, ...
        'ozel esik (max_eglim=30) etkili: 45 derece artik engelleniyor', gecen, kalan);

    % --- OZET ---
    fprintf('\n---------------------------------------------\n');
    fprintf('  GECEN: %d   KALAN: %d\n', gecen, kalan);
    fprintf('---------------------------------------------\n');
    if kalan > 0
        error('test_mex:basarisiz', '%d test basarisiz.', kalan);
    end
    fprintf('TUM TESTLER GECTI — kopru native C++ testleriyle uyumlu.\n');
end

% =====================================================================
function [g, k] = onay(kosul, ad, g, k)
    if kosul
        fprintf('  [GECTI] %s\n', ad);  g = g + 1;
    else
        fprintf('  [KALDI] %s\n', ad);  k = k + 1;
    end
end

function [hal, c] = adim(hal, irtifa, az, egim, t_us)
    g = ucus_girdi(irtifa, [0 0 az], [egim 0 0], [1 0 0 0], t_us, true, true);
    [hal, c] = ucus_mex('adim', hal, g);
end

% Kalman ivme filtresi basamak girdiyi tek adimda gecirmez; kalkis birkac
% ornekte olur. Native testteki kalkis_yap() ile ayni.
function [hal, t] = kalkis_yap(hal, t)
    for i = 1:10
        t = t + 10000;
        [hal, ~] = adim(hal, 0, 40, 0, t);
    end
end

function [hal, t] = ucus_baslat()
    hal = ucus_mex('sifirla');
    t = 1000000;
    [hal, t] = kalkis_yap(hal, t);
end

function [hal, t, c] = oturt(hal, irtifa, az, egim, t)
    for i = 1:40
        t = t + 10000;      % 100 Hz
        [hal, c] = adim(hal, irtifa, az, egim, t);
    end
end
