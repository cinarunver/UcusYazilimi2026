function derle()
%DERLE  ucus_mex koprusunu derler.
%
%   MatlabSim/mex/ucus_mex.cpp dosyasini, UcusAlgoritmasi/ucus_algoritmasi.h
%   basligini include ederek MEX olarak derler. Cikti MatlabSim/ altina yazilir
%   ki MATLAB yolunda tek bir yer olsun.
%
%   Kullanim:  cd MatlabSim/mex; derle
%
%   Onkosul: Xcode Command Line Tools (mex -setup C++ ile dogrulanabilir).

    buradan   = fileparts(mfilename('fullpath'));      % .../MatlabSim/mex
    matlabsim = fileparts(buradan);                    % .../MatlabSim
    kok       = fileparts(matlabsim);                  % proje koku
    baslik    = fullfile(kok, 'UcusAlgoritmasi');
    kaynak    = fullfile(buradan, 'ucus_mex.cpp');

    if ~isfile(fullfile(baslik, 'ucus_algoritmasi.h'))
        error('derle:baslikYok', ...
              'ucus_algoritmasi.h bulunamadi: %s', baslik);
    end

    fprintf('Derleniyor: %s\n', kaynak);
    fprintf('  baslik yolu : %s\n', baslik);
    fprintf('  cikti       : %s\n', matlabsim);

    % MATLAB'in mex'i macOS'ta tam Xcode ister ve lisansi kabul edilmemisse
    % "Supported compiler not detected" ile durur. Command Line Tools tek
    % basina yeterlidir; o durumda clang++ ile dogrudan derleyip ayni MEX
    % ikilisini uretiyoruz. Once resmi yol denenir, olmazsa yedege dusulur.
    try
        mex('-R2018a', ...
            ['-I' baslik], ...
            'CXXFLAGS=$CXXFLAGS -std=c++17', ...
            '-outdir', matlabsim, ...
            kaynak);
    catch hata
        fprintf(2, '\n  mex basarisiz (%s)\n', hata.identifier);
        fprintf('  -> clang++ yedegine dusuluyor (Command Line Tools yeterli)\n\n');
        derle_clang(kaynak, baslik, matlabsim);
    end

    fprintf('TAMAM. Kontrol:\n');
    eski = cd(matlabsim);
    temizle = onCleanup(@() cd(eski));
    b = ucus_mex('bilgi');
    fprintf('  UcusHal   = %d bayt\n', b.hal_bayt);
    fprintf('  UcusAyar  = %d bayt\n', b.ayar_bayt);
    fprintf('  UcusGirdi = %d bayt\n', b.girdi_bayt);
    fprintf('  UcusCikti = %d bayt\n', b.cikti_bayt);
end

% =====================================================================
function derle_clang(kaynak, baslik, cikti_dizin)
%DERLE_CLANG  MEX'i clang++ ile dogrudan uretir (tam Xcode gerektirmez).
    mr = matlabroot;
    switch computer('arch')
        case 'maca64', libdizin = fullfile(mr,'bin','maca64'); uzanti = 'mexmaca64'; ark = 'arm64';
        case 'maci64', libdizin = fullfile(mr,'bin','maci64'); uzanti = 'mexmaci64'; ark = 'x86_64';
        otherwise
            error('derle:platform', ...
                  'clang++ yedegi yalniz macOS icin tanimli (bulunan: %s).', computer('arch'));
    end

    hedef = fullfile(cikti_dizin, ['ucus_mex.' uzanti]);
    komut = sprintf(['clang++ -O2 -std=c++17 -fno-common -arch %s -bundle ' ...
                     '-DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a ' ...
                     '-I"%s" -I"%s" -L"%s" -lmex -lmx -Wl,-rpath,"%s" ' ...
                     '-o "%s" "%s"'], ...
                    ark, fullfile(mr,'extern','include'), baslik, ...
                    libdizin, libdizin, hedef, kaynak);

    fprintf('  %s\n\n', komut);
    [durum, ciktisi] = system(komut);
    if durum ~= 0
        error('derle:clang', 'clang++ derlemesi basarisiz:\n%s', ciktisi);
    end
    if ~isempty(strtrim(ciktisi)), fprintf('%s\n', ciktisi); end
    clear ucus_mex          % eski MEX yuklu kaldiysa serbest birak
    rehash path
end
