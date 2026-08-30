function ozet(s, ad)
%OZET  Bir kosumun sonucunu okunur sekilde yazar.

    if nargin < 2, ad = 'KOSUM'; end

    fprintf('\n== %s ==\n', ad);
    fprintf('  Gercek apogee        : %7.1f m   (t = %5.2f s)\n', s.apogee, s.apogee_t);

    if isnan(s.drogue_t)
        fprintf('  Drogue               :  ACILMADI  <<< KURTARMA BASARISIZ\n');
    else
        fprintf('  Drogue emri          : %7.1f m   (t = %5.2f s)\n', s.drogue_z, s.drogue_t);
        fprintf('  Apogee gecikmesi     : %+7.2f s  (%.1f m irtifa kaybi)\n', ...
                s.apogee_gecikme, s.apogee - s.drogue_z);
    end

    if isnan(s.ana_t)
        fprintf('  Ana parasut          :  ACILMADI  <<< KURTARMA BASARISIZ\n');
    else
        fprintf('  Ana parasut emri     : %7.1f m   (hedef 550 m, hata %+.1f m)\n', ...
                s.ana_z, s.ana_z - 550);
        fprintf('  Ana acilis hizi      : %7.1f m/s%s\n', s.ana_hiz, ...
                repmat('   <<< BALISTIK ACILIS', 1, s.ana_hiz > 30));
    end

    fprintf('  Inis hizi            : %7.1f m/s\n', s.inis_hizi);
    fprintf('  Yatay suruklenme     : %7.1f m\n',   s.suruklenme);

    % --- Sert kisit kontrolu (tasarim dokumani §8.2) ---
    ihlal = {};
    if isnan(s.drogue_t), ihlal{end+1} = 'drogue acilmadi'; end
    if isnan(s.ana_t),    ihlal{end+1} = 'ana parasut acilmadi'; end
    if s.inis_hizi > 10,  ihlal{end+1} = sprintf('inis hizi %.1f > 10 m/s', s.inis_hizi); end
    % Ana parasut balistik hizda acildiysa model "indi" der ama gercek
    % donanim yirtilir (bkz. Bulgu 7).
    if ~isnan(s.ana_hiz) && s.ana_hiz > 30
        ihlal{end+1} = sprintf('ana parasut %.0f m/s''de acildi (limit 30)', s.ana_hiz);
    end
    if ~isnan(s.drogue_t)
        j = find(s.t >= s.drogue_t, 1);
        if ~isempty(j)
            if s.vz(j) > 0
                ihlal{end+1} = sprintf('yukselirken atesleme (Vz = %+.1f m/s)', s.vz(j));
            end
            if s.tilt(j) > 45
                ihlal{end+1} = sprintf('egim %.0f > 45 derece iken atesleme', s.tilt(j));
            end
        end
    end

    if isempty(ihlal)
        fprintf('  Sert kisitlar        : HEPSI SAGLANDI\n');
    else
        fprintf('  Sert kisitlar        : IHLAL VAR\n');
        for i = 1:numel(ihlal), fprintf('      - %s\n', ihlal{i}); end
    end
end
