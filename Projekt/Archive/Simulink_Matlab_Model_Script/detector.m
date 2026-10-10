function det = detector(p, calib)
%DETECTOR  Detektor FDI oparty na residuum Digital Twina.
%
%  DLACZEGO PREDYKTOR JEDNOKROKOWY, A NIE MODEL SWOBODNY
%  -----------------------------------------------------
%  Wersja 1 porownywala calkowicie niezalezny przebieg MATLAB-a z obiektem.
%  Przy niedopasowaniu parametrow taki "swobodny" model rozjezdza sie
%  w stanie ustalonym o wartosc proporcjonalna do punktu pracy
%  (dla 5% bledu Ra bylo to ok. 1.3 rad/s) - czyli WIECEJ niz typowy atak.
%  Slaby atak ginie w bledzie modelu, a atak o przeciwnym znaku wrecz
%  ZMNIEJSZA residuum i staje sie niewidoczny.
%
%  Tutaj Digital Twin przewiduje TYLKO jeden krok do przodu na podstawie
%  ostatniego pomiaru:
%
%      x_pred(k+1) = Ad_nom * y(k) + Bd_nom * u(k)
%      r(k+1)      = omega_meas(k+1) - omega_pred(k+1)
%
%  Residuum jest wtedy ograniczone i nie kumuluje sie w czasie.
%
%  DLACZEGO TEST DWUSTRONNY I NORMALIZOWANY
%  ----------------------------------------
%  Niedopasowanie parametrow daje DETERMINISTYCZNE obciazenie mu(k),
%  ktore kalibrujemy na przebiegach bez ataku. Testujemy
%
%      z(k) = | r(k) - mu(k) | / sigma(k)   >   k_sigma
%
%  czyli odchylenie od ZNANEGO obciazenia, a nie od zera. Test jest
%  dwustronny, wiec atak "w dol" tez zostanie wykryty.

    [Ad, Bd] = motor_model(p.nom, p.dt);

    det.Ad = Ad;
    det.Bd = Bd;
    det.p  = p;

    if nargin < 2 || isempty(calib)
        % Tryb kalibracji: bez normalizacji i bez alarmow
        det.mu        = zeros(1, p.N);
        det.sd        = ones(1, p.N);
        det.calibrated = false;
    else
        det.mu         = calib.mu;
        det.sd         = max(calib.sd, 1e-9);
        det.calibrated = true;
    end

    % Stan detektora
    det.r      = nan(1, p.N);   % residuum surowe [rad/s]
    det.z      = nan(1, p.N);   % residuum znormalizowane [sigma]
    det.alarm  = zeros(1, p.N); % flaga alarmu
    det.cusum  = zeros(1, p.N); % statystyka CUSUM
    det.count  = 0;             % licznik kolejnych przekroczen
    det.Sp     = 0;             % akumulator CUSUM (odchylenie w gore)
    det.Sm     = 0;             % akumulator CUSUM (odchylenie w dol)
    det.latched = false;        % alarm zatrzaskowy

    det.update = @update;

end

%% =============================================================
function det = update(det, k, y_prev, U_prev, y_now)
%UPDATE  Jeden krok detektora. Indeks k = numer residuum (1..N).
%
%  y_prev - pomiar [i; omega] w kroku k
%  U_prev - sterowanie zastosowane w kroku k
%  y_now  - pomiar [i; omega] w kroku k+1

    p = det.p;

    %% ---------------------------------------------------------
    %  PREDYKCJA DIGITAL TWINA (model nominalny)
    %  ---------------------------------------------------------

    x_pred = det.Ad * y_prev(:) + det.Bd * U_prev;

    %% ---------------------------------------------------------
    %  RESIDUUM (ze znakiem!)
    %  ---------------------------------------------------------

    r = y_now(2) - x_pred(2);

    det.r(k) = r;

    if ~det.calibrated
        return;   % w trybie kalibracji zbieramy tylko r
    end

    %% ---------------------------------------------------------
    %  NORMALIZACJA
    %  ---------------------------------------------------------

    z = (r - det.mu(k)) / det.sd(k);

    det.z(k) = z;

    %% ---------------------------------------------------------
    %  TEST PROGOWY + PERSYSTENCJA
    %  ---------------------------------------------------------
    %  Persystencja (N kolejnych przekroczen) tlumi pojedyncze
    %  odstajace probki szumu -> obniza false alarm rate.

    if abs(z) > p.det.k_sigma
        det.count = det.count + 1;
    else
        det.count = 0;
    end

    alarm_thr = det.count >= p.det.persist;

    %% ---------------------------------------------------------
    %  CUSUM (wykrywa wolne dryfy ponizej progu chwilowego)
    %  ---------------------------------------------------------

    alarm_cusum = false;

    %  UWAGA: CUSUM musi dzialac na residuum ZE ZNAKIEM, w wersji
    %  dwustronnej (S+ i S-). Wariant naiwny S = max(0, S + |z| - k)
    %  DRYFUJE nawet bez ataku, bo E[|z|] = sqrt(2/pi) ~ 0.8 > k = 0.5,
    %  co daje gwarantowany falszywy alarm po ok. 17 krokach.
    %  Dla wersji ze znakiem E[z - k] = -k < 0, wiec S+ wraca do zera.

    if p.det.use_cusum
        det.Sp = max(0, det.Sp + z - p.det.cusum_k);
        det.Sm = max(0, det.Sm - z - p.det.cusum_k);

        det.cusum(k) = max(det.Sp, det.Sm);

        alarm_cusum = det.cusum(k) > p.det.cusum_h;
    end

    %% ---------------------------------------------------------
    %  DECYZJA
    %  ---------------------------------------------------------

    if alarm_thr || alarm_cusum
        det.latched = true;
    end

    det.alarm(k) = double(alarm_thr || alarm_cusum);

end
