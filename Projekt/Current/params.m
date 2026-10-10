function p = params()

%% ============================================================
%  PARAMETRY SILNIKA PRĄDU STAŁEGO
%  ============================================================

p.motor.Ra = 1.5;       % rezystancja uzwojenia [Ohm]
p.motor.La = 0.05;      % indukcyjność uzwojenia [H]

p.motor.Kt = 0.5;       % stała momentu [N*m/A]
p.motor.Ke = 0.5;       % stała SEM [V*s/rad]

p.motor.J  = 0.01;      % moment bezwładności [kg*m^2]
p.motor.B  = 0.1;       % współczynnik tarcia lepkiego [N*m*s/rad]

p.motor.TL = 0.0;       % moment obciążenia [N*m]


%% ============================================================
%  WARUNKI POCZĄTKOWE
%  ============================================================

p.initial.i_a    = 0.0;     % początkowy prąd twornika [A]
p.initial.omega  = 0.0;     % początkowa prędkość [rad/s]


%% ============================================================
%  PARAMETRY SYMULACJI
%  ============================================================

p.sim.dt   = 0.05;      % krok czasowy [s]
p.sim.Tsim = 2.0;       % czas symulacji [s]

p.sim.N = round(p.sim.Tsim / p.sim.dt);

p.sim.energy_substeps = 10;

%% ============================================================
%  OGRANICZENIA NAPIECIA
%  ============================================================

p.limits.Umin = 0.0;    % minimalne napięcie twornika [V]
p.limits.Umax = 24.0;   % maksymalne napięcie twornika [V]


%% ============================================================
%  OGRANICZENIA PRĄDU
%  ============================================================

p.limits.Imin = 0.0;    % minimalny prąd [A]
p.limits.Imax = 4.0;    % maksymalny prąd [A]


%% ============================================================
%  OGRANICZENIA PREDKOSCI
%  ============================================================

p.limits.omega_min = 0.0;
p.limits.omega_max = 12.0;


%% ============================================================
%  WARTOSC ZADANA
%  ============================================================

p.reference.omega = 10.0;    % prędkość zadana [rad/s]


%% ============================================================
%  PARAMETRY DYNAMICZNEGO PROGRAMOWANIA
%  ============================================================

% Siatka prądu
p.dp.i_grid = linspace( ...
    p.limits.Imin, ...
    p.limits.Imax, ...
    21);

% Siatka prędkości
p.dp.omega_grid = linspace( ...
    p.limits.omega_min, ...
    p.limits.omega_max, ...
    61);

% Siatka napięcia sterującego
p.dp.U_grid = linspace( ...
    p.limits.Umin, ...
    p.limits.Umax, ...
    31);

% Waga błędu prędkości
p.dp.lambda_speed = 5.0;

% Waga zużycia energii
p.dp.lambda_energy = 1.0;

% Waga błędu końcowego
p.dp.lambda_terminal = 100.0;


%% ============================================================
%  PARAMETRY REGULATORA KASKADOWEGO PI
%  ============================================================

% Regulator PI pradu
p.control.current.Kp = 7.5;
p.control.current.Ki = 225.0;

% Anti-windup regulatora pradu
p.control.current.Kb = ...
    p.control.current.Ki / p.control.current.Kp;

% Ograniczenie wartosci zadanej pradu
p.control.current.Iref_min = 0.0;
p.control.current.Iref_max = 4.0;

% Regulator PI predkosci
p.control.speed.Kp = 0.6;
p.control.speed.Ki = 6.0;

% Anti-windup regulatora predkosci
p.control.speed.Kb = ...
    p.control.speed.Ki / p.control.speed.Kp;

%% ============================================================
%  REGULATOR ZE SPRZEZENIEM OD STANU I CALKA UCHYBU
%  ============================================================

% Bieguny projektowe ukladu rozszerzonego
p.control.state_feedback.poles = [-30 -35 -40];

% Wzmocnienia wyznaczone metoda rozmieszczania biegunow
p.control.state_feedback.K_current  = 3.25;
p.control.state_feedback.K_speed    = 2.20;
p.control.state_feedback.K_integral = 42.0;

% Warunek poczatkowy integratora uchybu predkosci
p.control.state_feedback.xi0 = 0.0;

% Anti-windup: na razie wylaczony
p.control.state_feedback.Kaw = 0.05;

p.control.state_feedback.Kaw = 0.05;
p.control.state_feedback.Umax_test = 5.0;

%% ============================================================
%  PARAMETRY ATAKU FDI
%  ============================================================

p.fdi.enabled = false;

p.fdi.start_time = 1.0;

p.fdi.type = 'bias';

p.fdi.magnitude = 0.0;


%% ============================================================
%  PARAMETRY DETEKCJI ATAKU
%  ============================================================

p.detection.threshold = 1.0;     % próg residuum [rad/s]

%% PARAMETRY ATAKU FDI

p.fdi.enabled = false;

p.fdi.start_time = 1.0;

p.fdi.type = 'bias';

p.fdi.magnitude = 0.0;

% Czas narastania ataku ramp [s]
p.fdi.ramp_duration = 0.5;

% Czas trwania ataku pulse [s]
p.fdi.pulse_duration = 0.3;

end