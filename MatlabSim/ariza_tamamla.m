function a = ariza_tamamla(a)
%ARIZA_TAMAMLA  Eksik ariza alanlarini notr degerleriyle doldurur.
%   Kullanicinin yalniz ilgilendigi alani yazabilmesi icin.

    v = ariza_yok();
    if isempty(a) || ~isstruct(a), a = v; return; end
    alanlar = fieldnames(v);
    for i = 1:numel(alanlar)
        if ~isfield(a, alanlar{i}) || isempty(a.(alanlar{i}))
            a.(alanlar{i}) = v.(alanlar{i});
        end
    end
end
