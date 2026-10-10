
%% ============================================================
%  TEST INTEGRACYJNY FDI Z CYFROWYM BLIZNIAKIEM
%
%  Obiekt: run_simulink.m
%  Digital Twin: run_matlab.m
%  Atak: fdi_attack.m
%  Detektor: fdi_detector.m
%
%  Test w ukladzie otwartym.
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

%% 2. WSPOLNY SYGNAL STERUJACY

% Stale napiecie 8 V przez cala symulacje.
% Obie funkcje otrzymuja identyczne sterowanie.

u = 8.0 * ones(p.sim.N, 1);

%% 3. NIEZALEZNE SYMULACJE

fprintf('Uruchamianie modelu Simulink...\n');
plant = run_simulink(p, u);

fprintf('Uruchamianie cyfrowego blizniaka...\n');
dt_model = run_matlab(p, u);

%% 4. WSPOLNA OS CZASU - CHWILE PROBKOWANIA

% Detektor pracuje z okresem probkowania p.sim.dt.
t = dt_model.t(:);

% Predykcja cyfrowego blizniaka bez interpolacji.
omega_dt = dt_model.omega(:);

% Odczyt predkosci obiektu Simulink
% w chwilach probkowania cyfrowego blizniaka.
omega_plant = interp1( ...
    plant.t(:), ...
    plant.omega(:), ...
    t, ...
    'linear');

assert(all(isfinite(omega_plant)), ...
    'Nie udalo sie wyznaczyc probek obiektu.');

%% 5. WALIDACJA MODELI PRZED ATAKIEM

model_error = abs(omega_plant - omega_dt);

fprintf('\n========================================\n');
fprintf('ZGODNOSC MODELU I CYFROWEGO BLIZNIAKA\n');
fprintf('========================================\n');

fprintf('Maksymalna roznica: %.9f rad/s\n', ...
    max(model_error));

fprintf('Srednia roznica: %.9f rad/s\n', ...
    mean(model_error));

%% 6. GENEROWANIE ATAKU

attack = fdi_attack(t, omega_plant, p);

%% 7. DETEKCJA

detection = fdi_detector( ...
    t, ...
    attack.omega_measured, ...
    omega_dt, ...
    p);

%% 8. WERYFIKACJA

before_attack = t < p.fdi.start_time;
after_attack  = t >= p.fdi.start_time;

% Bez falszywych alarmow przed atakiem.
assert(~any(detection.alarm(before_attack)), ...
    'Wykryto falszywy alarm przed atakiem.');

% Atak powinien zostac wykryty.
assert(any(detection.alarm(after_attack)), ...
    'Nie wykryto ataku FDI.');

% Pierwszy alarm nie moze wystapic przed atakiem.
assert(detection.first_detection_time >= ...
       p.fdi.start_time, ...
       'Nieprawidlowa chwila detekcji.');

detection_delay = ...
    detection.first_detection_time - p.fdi.start_time;

fprintf('\n========================================\n');
fprintf('WYNIKI TESTU FDI\n');
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

fprintf('\nTEST FDI INTEGRATION: OK\n');

%% 9. WYKRESY

figure( ...
    'Name', 'FDI - test z cyfrowym blizniakiem');

subplot(3,1,1);

plot(t, omega_plant, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Predkosc rzeczywista');

hold on;

plot(t, omega_dt, '--', ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Digital Twin');

plot(t, attack.omega_measured, ':', ...
    'LineWidth', 1.7, ...
    'DisplayName', 'Pomiar po ataku');

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
ylabel('\omega [rad/s]');
title('Obiekt, cyfrowy blizniak i falszywy pomiar');
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
    'LineWidth', 1.5);

hold on;

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('Alarm');
ylim([-0.1 1.1]);
yticks([0 1]);
title('Decyzja detektora');

%% 10. KONIEC

fprintf('\nTest integracyjny zakonczony.\n');