function ciz(s, baslik_metni, kayit_yolu)
%CIZ  Simulasyon sonucunu dort panelde cizer.
%
%   ciz(s)                        ekrana cizer
%   ciz(s, 'baslik')              baslikla
%   ciz(s, 'baslik', 'ucus.png')  ayrica dosyaya kaydeder

    if nargin < 2 || isempty(baslik_metni), baslik_metni = 'Ucus Simulasyonu'; end
    if nargin < 3, kayit_yolu = ''; end

    f = figure('Position', [80 80 1150 850], 'Color', 'w');
    tiledlayout(f, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---------------- 1) IRTIFA ----------------
    ax1 = nexttile; hold(ax1,'on'); grid(ax1,'on');
    h1 = plot(s.t, s.z_olculen, 'Color', [.75 .75 .78], 'LineWidth', .8);
    h2 = plot(s.t, s.z, 'k', 'LineWidth', 1.6);
    olay_cizgisi(ax1, s);
    h3 = plot(s.apogee_t, s.apogee, 'v', 'MarkerSize', 9, ...
              'MarkerFaceColor', [.85 .33 .1], 'MarkerEdgeColor', 'none');
    yline(ax1, 550, ':', 'ana parasut esigi 550 m', 'Color', [.4 .4 .4]);
    ylabel(ax1, 'irtifa [m]');
    legend([h1 h2 h3], {'olculen (baro)','gercek','gercek apogee'}, ...
           'Location','northeast','Box','off');
    title(ax1, baslik_metni, 'FontWeight','bold');

    % ---------------- 2) DIKEY HIZ ----------------
    ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on');
    g1 = plot(s.t, s.vz_kest, 'Color', [.75 .75 .78], 'LineWidth', .8);
    g2 = plot(s.t, s.vz, 'k', 'LineWidth', 1.4);
    olay_cizgisi(ax2, s);
    yline(ax2, 3, ':', 'MIN\_DIKEY\_HIZ = +3', 'Color', [.4 .4 .4]);
    yline(ax2, 0, '-', 'Color', [.8 .8 .8]);
    ylabel(ax2, 'V_z [m/s]');
    legend([g1 g2], {'algoritmanin kestirdigi','gercek'}, 'Location','southeast','Box','off');

    % ---------------- 3) EGIM ----------------
    ax3 = nexttile; hold(ax3,'on'); grid(ax3,'on');
    e1 = plot(s.t, s.tilt_kest, 'Color', [.75 .75 .78], 'LineWidth', .8);
    e2 = plot(s.t, s.tilt, 'k', 'LineWidth', 1.4);
    olay_cizgisi(ax3, s);
    yline(ax3, 75, ':', 'MAX\_EGLIM = 75', 'Color', [.85 .33 .1]);
    ylabel(ax3, 'egim [derece]');
    legend([e1 e2], {'algoritmanin gordugu','gercek'}, 'Location','northwest','Box','off');

    % ---------------- 4) DURUM MAKINESI ----------------
    ax4 = nexttile; hold(ax4,'on'); grid(ax4,'on');
    stairs(s.t, s.durum, 'k', 'LineWidth', 1.6);
    olay_cizgisi(ax4, s);
    ylim(ax4, [-0.4 4.4]);
    yticks(ax4, 0:4);
    yticklabels(ax4, {'HAZIR','YUKSELIYOR','INIS\_1','INIS\_2','INDI'});
    xlabel(ax4, 'zaman [s]');

    linkaxes([ax1 ax2 ax3 ax4], 'x');
    xlim(ax1, [0 s.t(end)]);

    if ~isempty(kayit_yolu)
        d = fileparts(kayit_yolu);
        if ~isempty(d) && ~isfolder(d), mkdir(d); end
        exportgraphics(f, kayit_yolu, 'Resolution', 130);
        fprintf('Grafik kaydedildi: %s\n', kayit_yolu);
    end
end

% =====================================================================
function olay_cizgisi(ax, s)
%OLAY_CIZGISI  Funye emir anlarini dikey cizgi olarak isaretler.
    if ~isnan(s.drogue_t)
        xline(ax, s.drogue_t, '-', 'drogue', 'Color', [.85 .33 .10], ...
              'LineWidth', 1.2, 'LabelVerticalAlignment','bottom');
    end
    if ~isnan(s.ana_t)
        xline(ax, s.ana_t, '-', 'ana', 'Color', [0 .45 .74], ...
              'LineWidth', 1.2, 'LabelVerticalAlignment','bottom');
    end
end
