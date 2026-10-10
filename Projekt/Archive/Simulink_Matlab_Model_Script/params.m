function p = params()
%PARAMS  Konfiguracja projektu: Digital Twin + DP + detekcja ataku FDI.
%
%  KLUCZOWA ZASADA PROJEKTU
%  ------------------------
%  Istnieja DWA zestawy parametrow:
%
%    p.nom.*    - model NOMINALNY  (to, w co "wierzy" Digital Twin i DP)
%    p.plant.*  - obiekt RZECZYWISTY (Simulink = fizyczny naped)
%
%  p.plant = p.nom .* p.mismatch, czyli obiekt rzeczywisty ma inne
%  parametry niz model. To odwzorowuje bledy identyfikacji (temperatura
%  uzwojen, tarcie, moment bezwladnosci). Dzieki temu residuum w stanie
%  bez ataku NIE jest zerowe - i dobor progu detekcji staje sie
%  realnym problemem projektowym, a nie liczba wybrana "na oko".

    %% =========================================================
    %  MODEL NOMINALNY (Digital Twin + DP)
    %  =========================================================

    p.nom.J  = 0.01;        % Moment bezwladnosci [kg*m^2]
    p.nom.B  = 0.1;         % Tarcie lepkie [N*m*s/rad]
    p.nom.TL = 0.0;         % Moment obciazenia [N*m]

    p.nom.Ra = 1.5;         % Rezystancja twornika [Ohm]
    p.nom.La = 0.05;        % Indukcyjnosc twornika [H]

    p.nom.Kt = 0.5;         % Stala momentu [N*m/A]
    p.nom.Ke = 0.5;         % Stala SEM [V*s/rad]

    %% =========================================================
    %  NIEDOPASOWANIE MODEL <-> OBIEKT
    %  =========================================================
    %  Mnozniki bledu identyfikacji (1.02 = obiekt ma o 2% wiecej).
    %  Wartosci rzedu 1-3% odpowiadaja dobrze zidentyfikowanemu
    %  napedowi. Zwiekszenie ich podnosi podloge residuum i pogarsza
    %  wykrywalnosc slabych atakow - dobry material na analize wrazliwosci.

    p.mismatch.Ra = 1.02;   % +2%  (nagrzewanie uzwojen)
    p.mismatch.La = 0.99;   % -1%
    p.mismatch.Kt = 0.99;   % -1%
    p.mismatch.Ke = 1.01;   % +1%
    p.mismatch.J  = 1.015;  % +1.5%
    p.mismatch.B  = 1.03;   % +3%  (tarcie najgorzej identyfikowalne)

    %% =========================================================
    %  OBIEKT RZECZYWISTY
    %  =========================================================

    p.plant.Ra = p.nom.Ra * p.mismatch.Ra;
    p.plant.La = p.nom.La * p.mismatch.La;
    p.plant.Kt = p.nom.Kt * p.mismatch.Kt;
    p.plant.Ke = p.nom.Ke * p.mismatch.Ke;
    p.plant.J  = p.nom.J  * p.mismatch.J;
    p.plant.B  = p.nom.B  * p.mismatch.B;
    p.plant.TL = p.nom.TL;

    %% =========================================================
    %  PARAMETRY SYMULACJI
    %  =========================================================

    p.dt = 0.05;            % Krok sterowania (rate DP i detektora) [s]
    p.N  = 40;              % Liczba krokow sterowania

    p.omega_0 = 0.0;        % Predkosc poczatkowa [rad/s]
    p.i_0     = 0.0;        % Prad poczatkowy [A]

    % Calkowanie obiektu ciaglego: krok solvera << krok sterowania.
    % UWAGA: to naprawia oryginalny blad projektu - ode1 przy dt=0.05 s
    % dla wartosci wlasnych -20+-20j dawal |1+lambda*dt| = 1,
    % czyli niegasnace oscylacje numeryczne.
    p.plant.substeps = 50;                  % h = dt/50 = 1e-3 s
    p.plant.h        = p.dt / p.plant.substeps;
    p.plant.solver   = 'ode4';              % Runge-Kutta 4 w Simulinku

    %% =========================================================
    %  OGRANICZENIA NAPEDU
    %  =========================================================

    p.Umin = 0.0;           % Min. napiecie sterujace [V]
    p.Umax = 24.0;          % Max. napiecie sterujace [V]

    p.Imin = 0.0;           % Min. prad [A]
    p.Imax = 4.0;           % Max. prad [A]

    p.omega_max = 12.0;     % Max. predkosc uwzgledniana w DP [rad/s]

    %% =========================================================
    %  CEL STEROWANIA
    %  =========================================================

    p.omega_target = 10.0;  % Predkosc zadana [rad/s]

    %% =========================================================
    %  SIATKI DYNAMICZNEGO PROGRAMOWANIA
    %  =========================================================

    p.i_grid     = linspace(p.Imin, p.Imax, 21);
    p.omega_grid = linspace(0, p.omega_max, 61);
    p.U_grid     = linspace(p.Umin, p.Umax, 31);

    %% =========================================================
    %  FUNKCJA KOSZTU
    %  =========================================================

    p.beta            = 5;      % Waga uchybu predkosci
    p.lambda_E        = 1;      % Waga energii elektrycznej
    p.alpha_terminal  = 100;    % Waga koncowego uchybu predkosci

    p.dp_interp = true;         % Interpolacja biliniowa funkcji wartosci
                                % (false = najblizszy wezel, jak w wersji 1)

    %% =========================================================
    %  TOR POMIAROWY (SENSORY)
    %  =========================================================
    %  Bez szumu dobor progu jest trywialny - szum jest konieczny,
    %  zeby analiza FAR/ROC miala sens.

    p.sensor.sigma_w = 0.02;    % Szum czujnika predkosci [rad/s]
    p.sensor.sigma_i = 0.01;    % Szum czujnika pradu [A]
    p.sensor.seed    = 1;       % Ziarno generatora (powtarzalnosc)

    %% =========================================================
    %  ATAK FDI (wstrzykiwany w TOR POMIAROWY, nie w fizyke)
    %  =========================================================
    %  Atakujacy siedzi na kanale telemetrii: silnik dalej pracuje
    %  zgodnie z fizyka, falszowane jest to, co widzi regulator i detektor.

    p.attack.type       = 'bias';   % 'none'|'bias'|'ramp'|'scaling'|'replay'
    p.attack.start      = 1.0;      % Poczatek ataku [s]
    p.attack.bias       = 0.5;      % 'bias'    : offset [rad/s]
    p.attack.slope      = 2.0;      % 'ramp'    : narastanie [rad/s^2]
    p.attack.gain       = 1.05;     % 'scaling' : mnoznik [-]
    p.attack.replay_lag = 10;       % 'replay'  : opoznienie [kroki]
    p.attack.on_omega   = true;     % Atak na czujnik predkosci
    p.attack.on_current = false;    % Atak na czujnik pradu

    %% =========================================================
    %  DETEKTOR
    %  =========================================================
    %  Prog NIE jest zgadywany. Jest kalibrowany na przebiegach
    %  bez ataku: mu(k), sigma(k) -> prog = k_sigma * sigma.

    p.det.k_sigma  = 5.0;   % Prog w jednostkach sigma (test dwustronny)
    p.det.persist  = 2;     % Ile kolejnych przekroczen = alarm
    p.det.n_calib  = 300;   % Liczba przebiegow kalibracyjnych Monte Carlo
    p.det.use_cusum = true; % Dodatkowo CUSUM (wykrywa wolne dryfy)
    p.det.cusum_k  = 0.5;   % Martwa strefa CUSUM [sigma]
    p.det.cusum_h  = 5.0;   % Prog CUSUM [sigma]

    %% =========================================================
    %  URUCHAMIANIE
    %  =========================================================

    p.use_simulink = true;              % false = obiekt liczony w MATLAB
    p.model_name   = 'system_model';

    %% =========================================================
    %  ZGODNOSC WSTECZNA
    %  =========================================================
    %  Stare skrypty odwoluja sie do p.Ra, p.J itd. - wskazuja na model
    %  NOMINALNY (bo tego uzywa DP i Digital Twin).

    p.Ra = p.nom.Ra;  p.La = p.nom.La;
    p.Kt = p.nom.Kt;  p.Ke = p.nom.Ke;
    p.J  = p.nom.J;   p.B  = p.nom.B;   p.TL = p.nom.TL;

    p.attack_start       = p.attack.start;
    p.residual_threshold = NaN;   % Wyznaczany przez calibrate_detector()

end
