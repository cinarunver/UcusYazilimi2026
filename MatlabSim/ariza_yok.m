function a = ariza_yok()
%ARIZA_YOK  Ariza enjeksiyonunun notr (arizasiz) hali.
%
%   sim_ucus'un ucuncu argumani. Bir alani degistirip o arizayi devreye
%   alirsin; digerleri notr kalir.
%
%   a = ariza_yok();
%   a.baro_don_t = 26;              % 26. saniyede barometre donar
%   s = sim_ucus(p, [], a);

    a.baro_don_t        = NaN;   % s      bu andan sonra baro son degeri tekrarlar
    a.imu_doyma         = Inf;   % m/s^2  lineer ivme olcum araligi (BNO055 4G = 39.2)
    a.imu_eksen_hatasi  = 0;     % derece sensor ekseni ile govde ekseni arasindaki hata
    a.kararlilik_kaybi_t = NaN;  % s      bu andan sonra aerodinamik toparlanma
                                 %        momenti SIFIRLANIR (kanatcik kaybi /
                                 %        govde kirilmasi). Roket artik burnunu
                                 %        ruzgara cevirmez -> gercek takla.
    a.takla_t           = NaN;   % s      bu anda ani acisal hiz enjekte edilir
    a.takla_omega       = 0;     % rad/s  enjekte edilen acisal hiz
    a.drogue_calismadi  = false; % true   emir verilir ama drogue ACILMAZ (funye arizasi)
    a.imu_z_ters        = false; % true   BNO055 Z ekseni TERS isaretli monte/yapilandirilmis.
                                 %        ucus_algoritmasi.h icindeki
                                 %        "[TODO] BNO055 Z-ekseni yonu dogrulanmali!"
                                 %        notunun bedelini olcer.
    a.baro_ofset        = 0;     % m      referans basinc kaymasindan dogan SABIT irtifa
                                 %        yanlisligi. Kart padde beklerken hava
                                 %        basinci degisirse referans_basinc bayatlar;
                                 %        ~8.3 m/hPa. + ise irtifa oldugundan YUKSEK
                                 %        okunur.
end
