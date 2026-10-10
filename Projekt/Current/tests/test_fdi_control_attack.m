
%% ============================================================
%  TEST GENERATORA ATAKOW NA STEROWANIE
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

% N probek odpowiada N przedzialom sterowania.
t = (0:p.sim.N-1)' * p.sim.dt;

u = 8 * ones(p.sim.N, 1);

types = {'bias', 'ramp', 'pulse'};

results = struct([]);

%% 2. TESTY

for k = 1:numel(types)

    p_case = p;
    p_case.fdi.type = types{k};

    attack = fdi_control_attack(t, u, p_case);

    results(k).type = types{k};
    results(k).attack = attack;

    assert(all(attack.u_actual >= p.limits.Umin), ...
        'Przekroczono dolny limit napiecia.');

    assert(all(attack.u_actual <= p.limits.Umax), ...
        'Przekroczono gorny limit napiecia.');

    assert(all(attack.a(t < p.fdi.start_time) == 0), ...
        'Atak rozpoczal sie zbyt wczesnie.');

end

%% 3. SPRAWDZENIE BIAS

bias = results(1).attack;

assert(all(abs(bias.u_actual(t >= 1) - 10) < 1e-10), ...
    'Nieprawidlowa amplituda BIAS.');

%% 4. SPRAWDZENIE RAMP

ramp_attack = results(2).attack;

idx_start = find(abs(t - 1.0) < 1e-10, 1);
idx_end = find(abs(t - 1.5) < 1e-10, 1);

assert(abs(ramp_attack.a(idx_start)) < 1e-10, ...
    'RAMP: nieprawidlowy poczatek.');

assert(abs(ramp_attack.a(idx_end) - 2) < 1e-10, ...
    'RAMP: nieprawidlowa amplituda koncowa.');

%% 5. SPRAWDZENIE PULSE

pulse = results(3).attack;

pulse_window = t >= 1.0 & t < 1.3;

assert(all(abs(pulse.a(pulse_window) - 2) < 1e-10), ...
    'PULSE: nieprawidlowa amplituda.');

assert(all(abs(pulse.a(~pulse_window)) < 1e-10), ...
    'PULSE: atak poza zadanym przedzialem.');

%% 6. WYKRESY

figure('Name', 'FDI - ataki na sterowanie');

hold on;

stairs(t, u, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Sterowanie bez ataku');

for k = 1:numel(types)

    stairs(t, results(k).attack.u_actual, ...
        'LineWidth', 1.5, ...
        'DisplayName', upper(types{k}));

end

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('Napiecie [V]');
title('Ataki na sygnal sterujacy');
legend('Location', 'best');

fprintf('\n========================================\n');
fprintf('TEST FDI CONTROL ATTACK\n');
fprintf('========================================\n');

fprintf('BIAS: OK\n');
fprintf('RAMP: OK\n');
fprintf('PULSE: OK\n');

fprintf('\nWszystkie testy zakonczone pomyslnie.\n');