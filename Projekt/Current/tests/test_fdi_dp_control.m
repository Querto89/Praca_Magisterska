
%% ============================================================
% TEST FDI - ATAK NA STEROWANIE DP
%
% Obiekt Simulink: napiecie po ataku
% Digital Twin:    oryginalne napiecie DP
%% ============================================================

clear;
clc;
close all;

startup_project;

p = params();

% Na poczatek testujemy BIAS.
% Pozniej zmienimy na 'ramp' oraz 'pulse'.
p.fdi.enabled = true;
p.fdi.type = 'bias';
p.fdi.start_time = 1.0;
p.fdi.magnitude = 2.0;

p.fdi.ramp_duration = 0.5;
p.fdi.pulse_duration = 0.3;

fprintf('\n========================================\n');
fprintf('OPTYMALIZACJA DP\n');
fprintf('========================================\n');

%% 1. OPTYMALNE STEROWANIE

dp = dp_solver(p);

u_dp = dp.u_opt(:);

% Czas odpowiada poczatkom przedzialow sterowania.
t_u = (0:p.sim.N-1)' * p.sim.dt;

assert(numel(u_dp) == p.sim.N, ...
    'Nieprawidlowa liczba probek sterowania DP.');

%% 2. GENEROWANIE ATAKU

attack = fdi_control_attack(t_u, u_dp, p);

fprintf('\n========================================\n');
fprintf('ATAK NA STEROWANIE\n');
fprintf('========================================\n');

fprintf('Typ: %s\n', p.fdi.type);
fprintf('Poczatek: %.3f s\n', p.fdi.start_time);
fprintf('Amplituda: %.3f V\n', p.fdi.magnitude);

%% 3. SYMULACJA OBIEKTU I BLIZNIAKA

fprintf('\nUruchamianie obiektu bez ataku...\n');

plant_nominal = run_simulink(p, u_dp);

fprintf('Uruchamianie obiektu po ataku...\n');

plant_attack = run_simulink(p, attack.u_actual);

fprintf('Uruchamianie cyfrowego blizniaka...\n');

twin = run_matlab(p, u_dp);

%% 4. WSPOLNA OS CZASU

% Wyniki Simulink moga miec inna siatke czasu.
% Do analizy uzywamy probek Digital Twin.

t = twin.t(:);

omega_twin = twin.omega(:);

omega_nominal = interp1( ...
    plant_nominal.t(:), ...
    plant_nominal.omega(:), ...
    t, 'linear', 'extrap');

omega_attack = interp1( ...
    plant_attack.t(:), ...
    plant_attack.omega(:), ...
    t, 'linear', 'extrap');

%% 5. RESIDUUM I ALARM

% Przed analiza FDI sprawdzamy zgodnosc modeli
% w scenariuszu bez ataku.

model_error = abs(omega_nominal - omega_twin);

% W tym scenariuszu pomiar predkosci nie jest falszowany.
% Zmienia sie rzeczywista dynamika obiektu.

residual = abs(omega_attack - omega_twin);

threshold = p.detection.threshold;

alarm = residual > threshold;

%% 6. CZAS DETEKCJI

idx_detection = find( ...
    alarm & t >= p.fdi.start_time, ...
    1, 'first');

if isempty(idx_detection)

    first_detection = NaN;
    detection_delay = NaN;

else

    first_detection = t(idx_detection);

    detection_delay = ...
        first_detection - p.fdi.start_time;

end

%% 7. WSKAZNIKI

before_attack = t < p.fdi.start_time;
after_attack = t >= p.fdi.start_time;

max_model_error = max(model_error);

max_residual = max(residual);

max_residual_before = ...
    max(residual(before_attack));

max_residual_after = ...
    max(residual(after_attack));

false_alarms = nnz( ...
    alarm & before_attack);

E_nominal = plant_nominal.E(end);
E_attack = plant_attack.E(end);

delta_E = E_attack - E_nominal;

%% 8. WYNIKI

fprintf('\n========================================\n');
fprintf('ZGODNOSC MODELI BEZ ATAKU\n');
fprintf('========================================\n');

fprintf('Maksymalny blad: %.9f rad/s\n', ...
    max_model_error);

fprintf('\n========================================\n');
fprintf('WYNIKI ATAKU NA STEROWANIE DP\n');
fprintf('========================================\n');

fprintf('Maksymalne residuum: %.6f rad/s\n', ...
    max_residual);

fprintf('Residuum przed atakiem: %.6f rad/s\n', ...
    max_residual_before);

fprintf('Residuum po ataku: %.6f rad/s\n', ...
    max_residual_after);

fprintf('Prog detekcji: %.6f rad/s\n', ...
    threshold);

if isnan(first_detection)

    fprintf('Atak nie zostal wykryty.\n');

else

    fprintf('Pierwsza detekcja: %.6f s\n', ...
        first_detection);

    fprintf('Opoznienie detekcji: %.6f s\n', ...
        detection_delay);

end

fprintf('Falszywe alarmy: %d\n', ...
    false_alarms);

fprintf('\nEnergia bez ataku: %.6f J\n', ...
    E_nominal);

fprintf('Energia po ataku: %.6f J\n', ...
    E_attack);

fprintf('Zmiana energii: %+.6f J\n', ...
    delta_E);

fprintf('Predkosc koncowa bez ataku: %.6f rad/s\n', ...
    omega_nominal(end));

fprintf('Predkosc koncowa po ataku: %.6f rad/s\n', ...
    omega_attack(end));

%% 9. WYKRESY

figure('Name', 'FDI - atak na sterowanie DP');

subplot(3,1,1);

stairs(t_u, u_dp, ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Sterowanie DP');

hold on;

stairs(t_u, attack.u_actual, ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Napiecie po ataku');

xline(p.fdi.start_time, ':', ...
    'Poczatek ataku');

grid on;
ylabel('u [V]');
title('Manipulacja sygnalem sterujacym');
legend('Location', 'best');

subplot(3,1,2);

plot(t, omega_nominal, ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Obiekt bez ataku');

hold on;

plot(t, omega_attack, ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Obiekt po ataku');

plot(t, omega_twin, '--', ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Digital Twin');

yline(p.reference.omega, ':', ...
    'Predkosc zadana');

xline(p.fdi.start_time, ':', ...
    'Poczatek ataku');

grid on;
ylabel('\omega [rad/s]');
title('Predkosc obiektu i cyfrowego blizniaka');
legend('Location', 'best');

subplot(3,1,3);

plot(t, residual, ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Residuum');

hold on;

yline(threshold, '--', ...
    'Prog detekcji');

xline(p.fdi.start_time, ':', ...
    'Poczatek ataku');

stairs(t, double(alarm), ':', ...
    'LineWidth', 1.1, ...
    'DisplayName', 'Alarm (0/1)');

grid on;
xlabel('Czas [s]');
ylabel('Residuum [rad/s]');
title('Detekcja ataku na sterowanie');
legend('Location', 'best');

fprintf('\nTEST FDI CONTROL DP ZAKONCZONY.\n');