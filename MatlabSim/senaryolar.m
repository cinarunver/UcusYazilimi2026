function S = senaryolar()
%SENARYOLAR  Dogrulama senaryosu kutuphanesi (tasarim dokumani §7).
%
%   Her senaryo, algoritmanin BELIRLI bir kapisini veya yedegini sinar.
%   S(i).ad      : senaryo adi
%   S(i).sinar   : hangi mekanizmayi sinadigi
%   S(i).p_deg   : roket_params uzerine uygulanacak degisiklikler (struct)
%   S(i).ariza   : ariza enjeksiyonu (struct)
%   S(i).bekleti : bu senaryoda beklenen davranis (metin)

    i = 0;

    i=i+1; S(i).ad     = 'Nominal';
    S(i).sinar   = 'temel davranis';
    S(i).p_deg   = struct();
    S(i).ariza   = struct();
    S(i).bekleti = 'Drogue apogee civarinda, ana 550 m, inis < 10 m/s';

    i=i+1; S(i).ad     = 'Motor erken sonme';
    S(i).sinar   = 'APOGEE_MIN_IRTIFA arama tabani';
    S(i).p_deg   = struct('motor_kes', 1.4);   % itki 1.4 s'de kesilir
    S(i).ariza   = struct();
    S(i).bekleti = 'Apogee 550 m altinda kalirsa drogue ARANMAZ (taban kapisi)';

    i=i+1; S(i).ad     = 'Apogee civari kararlilik kaybi';
    S(i).sinar   = 'MAX_EGLIM guvenlik kapisi (irtifa korunurken)';
    S(i).p_deg   = struct();
    S(i).ariza   = struct('kararlilik_kaybi_t', 24, 'takla_t', 24, 'takla_omega', 1.5);
    S(i).bekleti = 'Irtifa yuksek kalir ama roket apogee-de yatar; kapi ateslemeyi kesmeli';

    i=i+1; S(i).ad     = 'Takla (tumbling)';
    S(i).sinar   = 'sahte apogee reddi';
    S(i).p_deg   = struct();
    S(i).ariza   = struct('kararlilik_kaybi_t', 15, 'takla_t', 15, 'takla_omega', 4.0);
    S(i).bekleti = 'Takla sirasinda egim buyuk; kapi ateslemeyi kesmeli';

    i=i+1; S(i).ad     = 'Yayvan tepe (dusuk Cd)';
    S(i).sinar   = 'APOGEE_IRTIFA_FARKI yeterliligi';
    S(i).p_deg   = struct('Cd', 0.28);
    S(i).ariza   = struct();
    S(i).bekleti = 'Tepe yayvan; 10 m dusus kriteri gec tetiklenebilir';

    i=i+1; S(i).ad     = 'Guclu ruzgar';
    S(i).sinar   = 'egim kapisinin nominal ucusu BLOKE ETMEMESI';
    S(i).p_deg   = struct('ruzgar', 18);
    S(i).ariza   = struct();
    S(i).bekleti = 'Weathercocking egimi buyutur; kapi yine de acilmali';

    i=i+1; S(i).ad     = 'Kotu statik delik';
    S(i).sinar   = 'sahte irtifa dususune dayaniklilik';
    S(i).p_deg   = struct('baro_dinamik_k', 0.12);
    S(i).ariza   = struct();
    S(i).bekleti = 'Dinamik basinc yukseliste sahte dusus gosterir; erken atesleme OLMAMALI';

    i=i+1; S(i).ad     = 'Apogee-de baro donmasi';
    S(i).sinar   = 'ana parasut yedek tetigi';
    S(i).p_deg   = struct();
    S(i).ariza   = struct('baro_don_t', 26);
    S(i).bekleti = 'Irtifa sabit kalir -> apogee kacar; yedek tetik de baroya bagli oldugu icin CALISMAZ';

    i=i+1; S(i).ad     = 'IMU doyma (4G)';
    S(i).sinar   = 'kalkis tespiti';
    S(i).p_deg   = struct();
    S(i).ariza   = struct('imu_doyma', 39.2, 'imu_eksen_hatasi', 8);
    S(i).bekleti = 'Ivme kirpilir ama 20 m/s^2 esigi hala asilmali';

    i=i+1; S(i).ad     = 'Drogue funyesi calismadi';
    S(i).sinar   = 'ana parasut yedek tetigi (son care)';
    S(i).p_deg   = struct();
    S(i).ariza   = struct('drogue_calismadi', true);
    S(i).bekleti = 'Drogue emri verilir ama acilmaz; ana parasut yine de acilmali';
end
