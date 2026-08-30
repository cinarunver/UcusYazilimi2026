%OYNA  Buradan basla. Calistir, grafige bak, asagidaki sayilari degistir, tekrar calistir.
%
%   Kullanim:  cd MatlabSim; oyna
%
%   Bu betik roketi ucurur ve GERCEK ucus algoritmasini (karta yuklenecek
%   ayni C++ kodu) o ucusun icinde kosturur. Algoritma gercek durumu ASLA
%   gormez — yalnizca gurultulu sensor okumasini gorur, tipki gercekte
%   oldugu gibi.

clear; clc;

% --- Yol kurulumu: nereden calistirilirsa calistirilsin dogru klasoru bulur ---
% VSCode dosyayi calistirirken MATLAB'in gecerli klasoru proje koku olabilir;
% bu yuzden yollar dosyanin KENDI konumundan turetiliyor.
buradan = fileparts(mfilename('fullpath'));
addpath(buradan, fullfile(buradan, 'config'));

p = roket_params();          % <-- roketin fizigi: config/roket_params.m

% =====================================================================
%  1) NOMINAL UCUS — mevcut esiklerle
% =====================================================================
s = sim_ucus(p);
ozet(s, 'NOMINAL — mevcut esikler');
ciz(s, 'Nominal ucus — mevcut esikler', fullfile(buradan,'cikti','1_nominal.png'));


% =====================================================================
%  2) OYNAMA ALANI — ESIKLERI DEGISTIR
% =====================================================================
%  Asagidaki alanlardan istedigini ac ve degistir. Yazmadigin her esik
%  firmware'deki varsayilan degerinde kalir.
%
%    kalkis_ivme_esigi    20      m/s^2   kalkis tespiti
%    apogee_irtifa_farki  10      m       apogee: max'tan bu kadar dusunce
%    min_dikey_hiz         3      m/s     apogee: hiz bunun altina inince
%    max_eglim            75      derece  atesleme izni (guvenlik kapisi)
%    apogee_min_irtifa   550      m       bu irtifa asilmadan apogee aranmaz
%    ayrilma2_mesafe     550      m       ana parasut irtifasi
%    kf_irtifa_p     [1.5 1.5 0.1]        irtifa Kalman'i (e_mea, e_est, q)

ayar = struct();
ayar.apogee_irtifa_farki = 25;        % <-- DENE: 10 yerine 25 m

s2 = sim_ucus(p, ayar);
ozet(s2, 'DENEME — apogee_irtifa_farki = 25 m');
ciz(s2, 'apogee\_irtifa\_farki = 25 m', fullfile(buradan,'cikti','2_deneme.png'));

fprintf('\n--- KARSILASTIRMA -------------------------------------\n');
fprintf('  %-28s %10s %10s\n', '', 'mevcut', 'deneme');
fprintf('  %-28s %10.2f %10.2f\n', 'apogee gecikmesi [s]', s.apogee_gecikme, s2.apogee_gecikme);
fprintf('  %-28s %10.1f %10.1f\n', 'drogue irtifasi [m]',  s.drogue_z,       s2.drogue_z);
fprintf('  %-28s %10.1f %10.1f\n', 'inis hizi [m/s]',      s.inis_hizi,      s2.inis_hizi);
fprintf('  %-28s %10.1f %10.1f\n', 'suruklenme [m]',       s.suruklenme,     s2.suruklenme);
fprintf('-------------------------------------------------------\n');


% =====================================================================
%  3) ARIZA SENARYOSU — kotu statik delik (sahte irtifa dususu)
% =====================================================================
%  Roket hizliyken statik deliklerden giren dinamik basinc barometreyi
%  yaniltir. README 4.3'te ucuncu onay kriterinin (egim kapisi) varlik
%  sebebi tam olarak budur. k = 0 -> hata yok, buyudukce kotulesir.

p_kotu = p;
p_kotu.baro_dinamik_k = 0.06;         % <-- DENE: 0, 0.02, 0.06, 0.12

s3 = sim_ucus(p_kotu);
ozet(s3, 'ARIZA — kotu statik delik (baro_dinamik_k = 0.06)');
ciz(s3, 'Ariza: dinamik basinc hatasi', fullfile(buradan,'cikti','3_ariza.png'));

fprintf('\nGrafikler kaydedildi: %s\n', fullfile(buradan,'cikti'));
