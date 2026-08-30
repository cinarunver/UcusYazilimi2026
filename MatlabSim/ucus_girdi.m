function g = ucus_girdi(irtifa, ivme, euler, kuat, t_us, sut_modu, filtrele)
%UCUS_GIRDI  ucus_mex('adim', ...) icin girdi struct'i kurar.
%
%   g = UCUS_GIRDI(irtifa, ivme, euler, kuat, t_us, sut_modu, filtrele)
%
%   irtifa   : skaler, BME280 ham irtifasi [m] (yer referansina gore)
%   ivme     : 1x3, BNO055 ham lineer ivme [m/s^2]
%   euler    : 1x3, [roll pitch yaw] [derece]
%   kuat     : 1x4, [qw qx qy qz]
%   t_us     : skaler, mikrosaniye zaman damgasi (uint32 araliginda)
%   sut_modu : true  -> egim EULER'den (SUT: gercek kuaterniyon yok)
%              false -> egim KUATERNIYON'dan (gercek ucus yolu)
%   filtrele : true  -> Kalman zinciri devrede (gercek ucus yolu / simulasyon)
%              false -> ham degerler dogrudan (SUT enjeksiyonunun bugunku hali)
%
%   Kisayol: eksik argumanlar gercek ucus yolunun varsayilanlariyla dolar
%   (dik yonelim, kuaterniyon kaynakli egim, filtre acik).

    if nargin < 3 || isempty(euler),    euler = [0 0 0];    end
    if nargin < 4 || isempty(kuat),     kuat  = [1 0 0 0];  end
    if nargin < 5 || isempty(t_us),     t_us  = 0;          end
    if nargin < 6 || isempty(sut_modu), sut_modu = false;   end
    if nargin < 7 || isempty(filtrele), filtrele = true;    end

    g = struct( ...
        'ham_irtifa', double(irtifa), ...
        'ham_ivme',   double(ivme(:)').*[1 1 1], ...
        'ham_euler',  double(euler(:)').*[1 1 1], ...
        'ham_kuat',   double(kuat(:)').*[1 1 1 1], ...
        't_us',       double(t_us), ...
        'sut_modu',   double(logical(sut_modu)), ...
        'filtrele',   double(logical(filtrele)));
end
