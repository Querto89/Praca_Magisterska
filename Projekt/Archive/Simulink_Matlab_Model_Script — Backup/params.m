function p = params()

    %% =========================================================
    %  PARAMETRY SILNIKA DC
    %  =========================================================

    % Parametry mechaniczne
    p.J  = 0.01;        % Moment bezwladnosci [kg*m^2]
    p.B  = 0.1;         % Wspolczynnik tarcia lepkiego [N*m*s/rad]
    p.TL = 0.0;         % Moment obciazenia [N*m]

    % Parametry elektryczne
    p.Ra = 1.5;         % Rezystancja twornika [Ohm]
    p.La = 0.05;        % Indukcyjnosc twornika [H]

    % Stale elektromechaniczne
    p.Kt = 0.5;         % Stala momentu [N*m/A]
    p.Ke = 0.5;         % Stala SEM [V*s/rad]

    %% =========================================================
    %  PARAMETRY SYMULACJI
    %  =========================================================

    p.dt = 0.05;        % Krok czasowy [s]
    p.N  = 40;          % Liczba krokow

    p.omega_0 = 0.0;    % Predkosc poczatkowa [rad/s]
    p.i_0     = 0.0;    % Prad poczatkowy [A]

    %% =========================================================
    %  OGRANICZENIA NAPEDU
    %  =========================================================

    p.Umin = 0.0;       % Minimalne napiecie sterujace [V]
    p.Umax = 24.0;      % Maksymalne napiecie sterujace [V]

    p.Imin = 0.0;       % Minimalny prad [A]
    p.Imax = 4.0;       % Maksymalny prad [A]

    p.omega_max = 12.0; % Maksymalna predkosc uwzgledniana w DP [rad/s]

    %% =========================================================
    %  CEL STEROWANIA
    %  =========================================================

    p.omega_target = 10.0;  % Predkosc zadana [rad/s]

    %% =========================================================
    %  SIATKI DYNAMICZNEGO PROGRAMOWANIA
    %  =========================================================

    p.i_grid = linspace(p.Imin, p.Imax, 21);
    p.omega_grid = linspace(0, p.omega_max, 61);

    p.U_grid = linspace(p.Umin, p.Umax, 31);

    %% =========================================================
    %  FUNKCJA KOSZTU
    %  =========================================================

    % Waga uchybu predkosci
    p.beta = 5;

    % Waga energii elektrycznej
    p.lambda_E = 1;

    % Waga koncowego uchybu predkosci
    p.alpha_terminal = 100;

    %% =========================================================
    %  PARAMETRY FDI
    %  =========================================================

    p.attack_start = 1.0;       % Początek ataku [s]
    p.attack_mag   = 0.0;       % Amplituda falszowania [rad/s]

    p.residual_threshold = 1.0; % Prog detekcji [rad/s]

end