
%% ============================================================
%  TEST TRZECH RODZAJOW ATAKOW FDI
%% ============================================================

clear;
clc;
close all;

startup_project;

p = params();

p.fdi.enabled = true;
p.fdi.start_time = 1.0;
p.fdi.magnitude = 2.0;
p.fdi.ramp_duration = 0.5;
p.fdi.pulse_duration = 0.3;

%% 1. SYGNAL TESTOWY

t = (0:0.01:2)';
omega = 10 * (1 - exp(-8*t));

%% 2. ATAK BIAS

p.fdi.type = 'bias';

bias = fdi_attack(t, omega, p);

%% 3. ATAK RAMP

p.fdi.type = 'ramp';

ramp_attack = fdi_attack(t, omega, p);

%% 4. ATAK PULSE

p.fdi.type = 'pulse';

pulse = fdi_attack(t, omega, p);

%% 5. TESTY POPRAWNOSCI

tol = 1e-10;

% BIAS: stale przesuniecie od 1 s
assert(all(abs(bias.a(t < 1)) < tol), ...
    'BIAS: atak wystapil przed 1 s.');

assert(all(abs(bias.a(t >= 1) - 2) < tol), ...
    'BIAS: nieprawidlowa amplituda.');

% RAMP: 0 w chwili 1 s, 2 w chwili 1.5 s
idx_start = find(abs(t - 1.0) < tol, 1);
idx_end = find(abs(t - 1.5) < tol, 1);

assert(abs(ramp_attack.a(idx_start)) < tol, ...
    'RAMP: nieprawidlowa amplituda poczatkowa.');

assert(abs(ramp_attack.a(idx_end) - 2) < tol, ...
    'RAMP: nieprawidlowa amplituda koncowa.');

assert(all(abs(ramp_attack.a(t >= 1.5) - 2) < tol), ...
    'RAMP: amplituda po narastaniu jest nieprawidlowa.');

% PULSE: atak w przedziale [1.0, 1.3)
pulse_window = t >= 1.0 & t < 1.3;

assert(all(abs(pulse.a(pulse_window) - 2) < tol), ...
    'PULSE: nieprawidlowa amplituda impulsu.');

assert(all(abs(pulse.a(~pulse_window)) < tol), ...
    'PULSE: atak wystepuje poza zadanym przedzialem.');

fprintf('\n========================================\n');
fprintf('TEST RODZAJOW ATAKOW FDI\n');
fprintf('========================================\n');
fprintf('BIAS:  OK\n');
fprintf('RAMP:  OK\n');
fprintf('PULSE: OK\n');
fprintf('Wszystkie testy zakonczone pomyslnie.\n');

%% 6. WYKRESY

figure('Name', 'Rodzaje atakow FDI');

subplot(2,1,1);

plot(t, omega, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Predkosc rzeczywista');

hold on;

plot(t, bias.omega_measured, '--', ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Bias');

plot(t, ramp_attack.omega_measured, '-.', ...
    'LineWidth', 1.3, ...
    'DisplayName', 'Ramp');

plot(t, pulse.omega_measured, ':', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Pulse');

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('\omega [rad/s]');
title('Pomiar predkosci podczas roznych atakow');
legend('Location', 'best');

subplot(2,1,2);

plot(t, bias.a, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Bias');

hold on;

plot(t, ramp_attack.a, '--', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Ramp');

plot(t, pulse.a, ':', ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Pulse');

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('a(t) [rad/s]');
title('Wstrzykiwane sygnaly FDI');
legend('Location', 'best');