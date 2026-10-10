
%% ============================================================
%  TEST FDI - NAPED OPTYMALIZOWANY METODA DP
%
%  Sterowanie: dp_solver.m
%  Obiekt: run_simulink.m
%  Digital Twin: run_matlab.m
%  Atak: fdi_attack.m
%  Detektor: fdi_detector.m
%
%  Atak dotyczy wylacznie pomiaru predkosci.
%  Sekwencja sterowania DP pozostaje niezmieniona.
%% ============================================================

clear;
clc;
close all;

startup_project;

%% 1. PARAMETRY

p = params();

p.fdi.enabled = true;
p.fdi.start_time = 1.0;
p.fdi.type = 'bias';
p.fdi.magnitude = 2.0;

p.detection.threshold = 1.0;

%% 2. OPTYMALIZACJA DP

fprintf('\n========================================\n');
fprintf('OBLICZANIE OPTYMALNEGO STEROWANIA DP\n');
fprintf('========================================\n');

tic;
dp = dp_solver(p);
dp_time = toc;

u_opt = dp.u_opt(:);

assert(numel(u_opt) == p.sim.N, ...
    'Nieprawidlowa liczba probek sterowania DP.');

fprintf('Czas obliczen DP: %.3f s\n', dp_time);
fprintf('Maksymalne napiecie DP: %.6f V\n', ...
    max(u_opt));

%% 3. OBIEKT I CYFROWY BLIZNIAK

fprintf('\nUruchamianie obiektu Simulink...\n');
plant = run_simulink(p, u_opt);

fprintf('Uruchamianie cyfrowego blizniaka...\n');
dt_model = run_matlab(p, u_opt);

%% 4. WSPOLNA OS CZASU

% Detektor pracuje w chwilach probkowania DT.
t = dt_model.t(:);

omega_dt = dt_model.omega(:);

% Probki predkosci obiektu w tych samych chwilach.
omega_plant = interp1( ...
    plant.t(:), ...
    plant.omega(:), ...
    t, ...
    'linear');

assert(all(isfinite(omega_plant)), ...
    'Nieprawidlowe probki predkosci obiektu.');

%% 5. ZGODNOSC MODELI BEZ ATAKU

model_error = abs(omega_plant - omega_dt);

fprintf('\n========================================\n');
fprintf('ZGODNOSC OBIEKTU I DIGITAL TWIN\n');
fprintf('========================================\n');

fprintf('Maksymalny blad: %.9f rad/s\n', ...
    max(model_error));

fprintf('Sredni blad: %.9f rad/s\n', ...
    mean(model_error));

%% 6. ATAK FDI

attack = fdi_attack(t, omega_plant, p);

%% 7. DETEKCJA

detection = fdi_detector( ...
    t, ...
    attack.omega_measured, ...
    omega_dt, ...
    p);

%% 8. WERYFIKACJA

before_attack = t < p.fdi.start_time;
after_attack = t >= p.fdi.start_time;

assert(any(before_attack) && any(after_attack), ...
    'Chwila ataku musi lezec wewnatrz symulacji.');

assert(~any(detection.alarm(before_attack)), ...
    'Falszywy alarm przed atakiem.');

assert(any(detection.alarm(after_attack)), ...
    'Nie wykryto ataku FDI.');

assert(detection.first_detection_time >= ...
    p.fdi.start_time, ...
    'Detekcja wystapila przed atakiem.');

detection_delay = ...
    detection.first_detection_time - p.fdi.start_time;

%% 9. WYNIKI

fprintf('\n========================================\n');
fprintf('WYNIKI FDI DLA STEROWANIA DP\n');
fprintf('========================================\n');

fprintf('Poczatek ataku: %.6f s\n', ...
    p.fdi.start_time);

fprintf('Amplituda ataku: %.6f rad/s\n', ...
    p.fdi.magnitude);

fprintf('Prog detekcji: %.6f rad/s\n', ...
    p.detection.threshold);

fprintf('Pierwsza detekcja: %.6f s\n', ...
    detection.first_detection_time);

fprintf('Opoznienie detekcji: %.6f s\n', ...
    detection_delay);

fprintf('Maksymalne residuum przed atakiem: %.9f rad/s\n', ...
    max(detection.residual(before_attack)));

fprintf('Srednie residuum po ataku: %.6f rad/s\n', ...
    mean(detection.residual(after_attack)));

fprintf('\nPredkosc koncowa obiektu: %.6f rad/s\n', ...
    omega_plant(end));

fprintf('Energia obiektu: %.6f J\n', ...
    plant.E(end));

fprintf('\nTEST FDI DP: OK\n');

%% 10. WYKRESY

figure( ...
    'Name', 'FDI dla sterowania DP');

subplot(3,1,1);

plot(t, omega_plant, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Obiekt Simulink');

hold on;

plot(t, omega_dt, '--', ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Digital Twin');

plot(t, attack.omega_measured, ':', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Pomiar po ataku');

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
ylabel('\omega [rad/s]');
title('Sterowanie DP - predkosc i atak FDI');
legend('Location', 'best');

subplot(3,1,2);

plot(t, detection.residual, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Residuum');

hold on;

yline(p.detection.threshold, 'r--', ...
    'Prog detekcji');

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
ylabel('r [rad/s]');
title('Residuum detektora');
legend('Location', 'best');

subplot(3,1,3);

stairs(t, double(detection.alarm), ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Alarm');

hold on;

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('Alarm');
ylim([-0.1 1.1]);
yticks([0 1]);
title('Decyzja detektora');

%% 11. ZACHOWANIE WYNIKOW W WORKSPACE

fdi_dp_result.p = p;
fdi_dp_result.dp = dp;
fdi_dp_result.plant = plant;
fdi_dp_result.digital_twin = dt_model;
fdi_dp_result.attack = attack;
fdi_dp_result.detection = detection;
fdi_dp_result.detection_delay = detection_delay;
fdi_dp_result.dp_computation_time = dp_time;

fprintf('\nTest zakonczony.\n');