function s = sim_ucus(p, ayar)
%SIM_UCUS  Kapali dongu ucus simulasyonu — GERCEK ucus algoritmasiyla.
%
%   s = SIM_UCUS(p)         p = roket_params()
%   s = SIM_UCUS(p, ayar)   ayar = ucus algoritmasi esikleri (struct)
%
%   DONGU KAPALIDIR: algoritma drogue emri verdiginde roketin suruklemesi
%   degisir, dolayisiyla sonraki yorunge algoritmanin kararina baglidir.
%   Acik dongu (once yorunge uret, sonra algoritmayi uzerinde kostur)
%   kullanilamaz — gec apogee tespitinin inis hizina etkisi olculemezdi.
%
%   Zincir:   roket modeli -> sensor modeli -> ucus_mex -> emirler -> model
%                                              (GERCEK ucan kod)
%
%   Donen s yapisinda: t, z, vz, tilt (gercek) + olculen degerler +
%   algoritmanin durumu ve emir anlari.

    if nargin < 2, ayar = struct(); end
    rng(p.tohum);

    % --- Algoritmayi baslat (GERCEK kod, MEX koprusu uzerinden) ---
    hal = ucus_mex('sifirla', ayar);

    % --- Durum vektoru: [x z vx vz theta omega] ---
    th0 = p.rampa_acisi * pi/180;
    y   = [0; 0; 0; 0; th0; 0];

    % --- Parasut durumu ---
    drogue_t = NaN;   % acilma emri zamani
    ana_t    = NaN;

    % --- Kayit ---
    n    = ceil(p.t_bitis / p.dt_sensor) + 1;
    s.t          = nan(n,1);   s.z        = nan(n,1);
    s.vz         = nan(n,1);   s.tilt     = nan(n,1);
    s.x          = nan(n,1);   s.hiz      = nan(n,1);
    s.z_olculen  = nan(n,1);   s.vz_kest  = nan(n,1);
    s.tilt_kest  = nan(n,1);   s.durum    = nan(n,1);
    s.az_olculen = nan(n,1);   s.bitler   = nan(n,1);

    adim_orani = round(p.dt_sensor / p.dt_fizik);
    t = 0; k = 0; i = 0;

    while t < p.t_bitis
        % ---------- SENSOR + ALGORITMA (100 Hz) ----------
        if mod(k, adim_orani) == 0
            i = i + 1;
            [olcum, gercek] = sensor_oku(y, t, p, drogue_t, ana_t);

            girdi = ucus_girdi(olcum.irtifa, olcum.ivme, olcum.euler, ...
                               olcum.kuat, round(t*1e6), false, true);
            [hal, c] = ucus_mex('adim', hal, girdi);

            % --- EMIRLER: modele geri besleme (dongunun kapandigi yer) ---
            if c.funye1_emir && isnan(drogue_t), drogue_t = t; end
            if c.funye2_emir && isnan(ana_t),    ana_t    = t; end

            s.t(i)=t;  s.z(i)=y(2);  s.vz(i)=y(4);  s.x(i)=y(1);
            s.tilt(i)      = abs(y(5))*180/pi;
            s.hiz(i)       = hypot(y(3), y(4));
            s.z_olculen(i) = olcum.irtifa;
            s.az_olculen(i)= olcum.ivme(3);
            s.vz_kest(i)   = c.dikey_hiz;
            s.tilt_kest(i) = c.eglim_acisi;
            s.durum(i)     = c.durum;
            s.bitler(i)    = c.durum_bitleri;

            if c.durum == 4 && t > 1, break; end      % INDI
        end

        % ---------- FIZIK (1 kHz, RK4) ----------
        y = rk4(y, t, p.dt_fizik, p, drogue_t, ana_t);
        if y(2) < 0 && t > 1, y(2) = 0; break; end     % yere carpti
        t = t + p.dt_fizik; k = k + 1;
    end

    % --- Kayitlari kirp ---
    alan = {'t','z','vz','tilt','x','hiz','z_olculen','vz_kest', ...
            'tilt_kest','durum','az_olculen','bitler'};
    for a = alan, s.(a{1}) = s.(a{1})(1:i); end

    % --- Ozet buyuklukler ---
    [s.apogee, ai]  = max(s.z);
    s.apogee_t      = s.t(ai);
    s.drogue_t      = drogue_t;
    s.ana_t         = ana_t;
    s.drogue_z      = kesit(s, drogue_t);
    s.ana_z         = kesit(s, ana_t);
    s.apogee_gecikme= drogue_t - s.apogee_t;
    s.inis_hizi     = abs(s.vz(end));
    s.suruklenme    = abs(s.x(end));
    s.ayar          = ayar;
end

% =====================================================================
function z = kesit(s, tt)
%KESIT  Verilen andaki gercek irtifa (emir yoksa NaN).
    if isnan(tt), z = NaN; return; end
    [~, j] = min(abs(s.t - tt));  z = s.z(j);
end

% =====================================================================
function [o, g] = sensor_oku(y, t, p, drogue_t, ana_t)
%SENSOR_OKU  Gercek durumdan HAM sensor okumasi uretir.
%   Algoritmanin gordugu tek sey budur — gercek durumu asla gormez.

    z = y(2); vx = y(3); vz = y(4); th = y(5);
    g.z = z;  g.tilt = abs(th)*180/pi;

    % --- Barometre: beyaz gurultu + dinamik basinc hatasi ---
    % Dinamik basinc terimi statik delik yerlesiminden dogar ve hizla
    % artar; "sahte irtifa dususu" mekanizmasi budur (bkz. README 4.3).
    v2   = vx^2 + vz^2;
    rho  = p.rho0 * exp(-(max(z,0)+p.kalkis_rakimi)/p.olcek_yuk);
    dyn  = p.baro_dinamik_k * 0.5 * rho * v2 / (rho * p.g);   % [m] karsiligi
    o.irtifa = z - dyn + p.baro_sigma * randn();

    % --- IMU: BNO055 LINEARACCEL gibi, YERCEKIMI CIKARILMIS, govde ekseninde ---
    [F, ~] = kuvvetler(y, t, p, drogue_t, ana_t);
    m      = kutle(t, p, drogue_t);
    a_lin  = F.itki_ve_suruklenme / m;            % agirlik haric [x z]
    eks_z  = [sin(th); cos(th)];                  % govde uzun ekseni
    eks_x  = [cos(th); -sin(th)];
    o.ivme = [ dot(a_lin, eks_x) + p.ivme_bias + p.ivme_sigma*randn(), ...
               0, ...
               dot(a_lin, eks_z) + p.ivme_bias + p.ivme_sigma*randn() ];

    % --- Yonelim ---
    th_ol   = th + (p.aci_sigma*pi/180) * randn();
    o.euler = [th_ol*180/pi, 0, 0];                     % roll, pitch, yaw
    o.kuat  = [cos(th_ol/2), sin(th_ol/2), 0, 0];       % qw qx qy qz
    % Not: 1 - 2*(qx^2+qy^2) = cos(th) -> algoritmanin egim formulu dogru
    % aciyi verir. Gercek ucus yolu bu (kuaterniyon), Euler DEGIL.
end

% =====================================================================
function m = kutle(t, p, drogue_t)
%KUTLE  Anlik kutle.
%   Yakit, gecen ZAMANLA degil harcanan IMPULS'la orantili tukenir — gercek
%   itki egrisi one yuklu oldugu icin aradaki fark max-Q civarinda anlamlidir.
%   Apogee'de govde ayrilir: UKB'yi tasiyan ana govde daha hafif iner.
    if nargin < 3, drogue_t = NaN; end
    if t < p.yanma_suresi
        harcanan = p.impuls_kum(min(t, p.yanma_suresi)) / p.impuls_toplam;
        m = p.m_kalkis - p.m_yakit * harcanan;
    elseif ~isnan(drogue_t) && t >= drogue_t
        m = p.m_ayrilma;          % drogue ile birlikte ayrildi
    else
        m = p.m_bos;
    end
end

% =====================================================================
function [F, M] = kuvvetler(y, t, p, drogue_t, ana_t)
%KUVVETLER  Toplam kuvvet [x z] ve yunuslama momenti.

    z = y(2); vx = y(3); vz = y(4); th = y(5); om = y(6);
    m   = kutle(t, p, drogue_t);
    rho = p.rho0 * exp(-(max(z,0)+p.kalkis_rakimi)/p.olcek_yuk);

    eks_z = [sin(th); cos(th)];              % govde uzun ekseni (burun)

    % --- Itki ---
    T = interp1(p.itki_egrisi(:,1), p.itki_egrisi(:,2), t, 'linear', 0);
    F_itki = T * eks_z;

    % --- Bagil hiz (ruzgar dahil) ---
    v_bag = [vx - p.ruzgar; vz];
    V     = norm(v_bag);

    % --- Surukleme: govde + acilmis parasutler ---
    CdA = p.Cd * p.A;
    CdA = CdA + parasut_CdA(t, drogue_t, p.drogue_CdA, p);
    CdA = CdA + parasut_CdA(t, ana_t,    p.ana_CdA,    p);

    if V > 1e-6
        F_sur = -0.5 * rho * V^2 * CdA * (v_bag / V);
    else
        F_sur = [0; 0];
    end

    F.itki_ve_suruklenme = F_itki + F_sur;    % IMU'nun gordugu (agirlik haric)
    F.toplam = F.itki_ve_suruklenme + [0; -m * p.g];

    % --- Yunuslama momenti: aerodinamik toparlanma + sonumleme ---
    % Parasut altinda govde aerodinamigi anlamsizlasir, momenti sonlendir.
    % V esigi ONEMLI: dusuk hizda atan2 ile hesaplanan hucum acisi anlamsiz
    % buyur (ruzgar hiz vektorune hakim olur) ve roketi rampada devirir.
    if V > 30 && isnan(drogue_t)
        alpha = atan2(v_bag(1), v_bag(2)) - th;      % hucum acisi
        alpha = atan2(sin(alpha), cos(alpha));       % [-pi, pi]
        alpha = max(-0.35, min(0.35, alpha));        % dogrusal aralikta tut (~20 derece)
        % ISARET: alpha = (hiz vektoru acisi) - (govde acisi). Statik kararli
        % bir rokette (CP, CG'nin ARKASINDA) moment burnu hiz vektorune dogru
        % cevirir — yani alpha>0 iken th ARTMALI. Dolayisiyla terim ARTI'dir.
        % Eksi yazilirsa roket statik KARARSIZ olur ve rampadan cikar cikmaz
        % takla atar (bu hata bir kez yapildi, apogee 3000 m yerine 408 m cikti).
        M = +0.5*rho*V^2*p.A * p.Cn_alpha * alpha * p.CP_CG ...
            - p.sonumleme * om * V;
    else
        M = -2.0 * om;                               % sadece sonumleme
    end
end

% =====================================================================
function CdA = parasut_CdA(t, acilma_t, tam_CdA, p)
%PARASUT_CDA  Sonlu acilma (sisme) sureli parasut suruklemesi.
    if isnan(acilma_t) || t < acilma_t
        CdA = 0; return;
    end
    oran = min(1, (t - acilma_t) / p.acilma_sure);
    CdA  = tam_CdA * oran^2;      % sisme profili
end

% =====================================================================
function ydot = tureva(y, t, p, drogue_t, ana_t)
    [F, M] = kuvvetler(y, t, p, drogue_t, ana_t);
    m = kutle(t, p, drogue_t);

    % --- RAMPA FAZI ---
    % Roket rampayi terk edene kadar donemez ve yalniz ray boyunca hareket
    % eder. Bu faz modellenmezse roket daha birkac m/s hizdayken ruzgarin
    % urettigi sahte hucum acisiyla devrilir — gercekte rampa buna izin
    % vermez. Ayrica itki agirligi yenene kadar roket yerinde durur.
    if hypot(y(1), y(2)) < p.rampa_uzunlugu
        eks = [sin(y(5)); cos(y(5))];
        f_eksen = dot(F.toplam, eks);
        if f_eksen < 0 && hypot(y(3), y(4)) < 0.1
            ydot = zeros(6,1); return;      % itki agirligi yenmedi
        end
        a = (f_eksen / m) * eks;
        ydot = [ y(3); y(4); a(1); a(2); 0; 0 ];
        return;
    end

    J = m * 0.6;                        % basitlestirilmis atalet momenti
    ydot = [ y(3); y(4); F.toplam(1)/m; F.toplam(2)/m; y(6); M/J ];
end

% =====================================================================
function y2 = rk4(y, t, h, p, drogue_t, ana_t)
    k1 = tureva(y,          t,       p, drogue_t, ana_t);
    k2 = tureva(y + h/2*k1, t + h/2, p, drogue_t, ana_t);
    k3 = tureva(y + h/2*k2, t + h/2, p, drogue_t, ana_t);
    k4 = tureva(y + h*k3,   t + h,   p, drogue_t, ana_t);
    y2 = y + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end
