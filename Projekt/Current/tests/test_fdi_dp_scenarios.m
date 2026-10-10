
%% ============================================================
%  TEST FDI DP - POROWNANIE TRZECH RODZAJOW ATAKOW
%
%  Bias, ramp, pulse na tej samej trajektorii DP.
%  DP, obiekt i Digital Twin sa uruchamiane tylko raz.
%% ============================================================

clear;
clc;
close all;

startup_project;

%% 1. PARAMETRY

p = params();

p.fdi.enabled = true;
p.fdi.start_time = 1.0;
p.fdi.magnitude = 2.0;
p.fdi.ramp_duration = 0.5;
p.fdi.pulse_duration = 0.3;

p.detection.threshold = 1.0;

attack_types = {'bias', 'ramp', 'pulse'};
n_scenarios = numel(attack_types);

%% 2. OPTYMALIZACJA DP

fprintf('\n========================================\n');
fprintf('OPTYMALIZACJA DP\n');
fprintf('========================================\n');

tic;
dp = dp_solver(p);
dp_time = toc;

u_opt = dp.u_opt(:);

assert(numel(u_opt) == p.sim.N, ...
    'Nieprawidlowa liczba probek sterowania DP.');

fprintf('Czas obliczen DP: %.3f s\n', dp_time);

%% 3. OBIEKT SIMULINK I DIGITAL TWIN

fprintf('\nUruchamianie obiektu Simulink...\n');
plant = run_simulink(p, u_opt);

fprintf('Uruchamianie cyfrowego blizniaka...\n');
dt_model = run_matlab(p, u_opt);

% Wspolna dyskretna os czasu
t = dt_model.t(:);

omega_dt = dt_model.omega(:);

omega_plant = interp1( ...
    plant.t(:), ...
    plant.omega(:), ...
    t, ...
    'linear');

assert(all(isfinite(omega_plant)), ...
    'Nieprawidlowe probki obiektu Simulink.');

%% 4. WERYFIKACJA ZGODNOSCI MODELI

model_error = abs(omega_plant - omega_dt);

fprintf('\n========================================\n');
fprintf('ZGODNOSC MODELI\n');
fprintf('========================================\n');

fprintf('Maksymalny blad: %.9f rad/s\n', ...
    max(model_error));

fprintf('Sredni blad: %.9f rad/s\n', ...
    mean(model_error));

fprintf('Energia obiektu: %.6f J\n', ...
    plant.E(end));

fprintf('Predkosc koncowa: %.6f rad/s\n', ...
    omega_plant(end));

%% 5. PRZYGOTOWANIE WYNIKOW

scenarios = struct([]);

first_detection = NaN(n_scenarios, 1);
detection_delay = NaN(n_scenarios, 1);
max_residual = NaN(n_scenarios, 1);
mean_residual = NaN(n_scenarios, 1);
alarm_samples = zeros(n_scenarios, 1);
false_alarms = zeros(n_scenarios, 1);

%% 6. PETLA SCENARIUSZY FDI

for k = 1:n_scenarios

    p_case = p;
    p_case.fdi.type = attack_types{k};

    fprintf('\n----------------------------------------\n');
    fprintf('ATAK: %s\n', upper(attack_types{k}));
    fprintf('----------------------------------------\n');

    % Generowanie falszywego pomiaru
    attack = fdi_attack( ...
        t, ...
        omega_plant, ...
        p_case);

    % Detekcja na podstawie predykcji DT
    detection = fdi_detector( ...
        t, ...
        attack.omega_measured, ...
        omega_dt, ...
        p_case);

    % Przedzial przed rozpoczeciem ataku
    before_attack = t < p_case.fdi.start_time;

    % Pierwsza probka z aktywnym alarmem
    first_idx = find(detection.alarm, 1, 'first');

    if ~isempty(first_idx)

        first_detection(k) = t(first_idx);

        detection_delay(k) = ...
            first_detection(k) - p_case.fdi.start_time;

    end

    max_residual(k) = max(detection.residual);
    mean_residual(k) = mean(detection.residual);

    alarm_samples(k) = nnz(detection.alarm);

    false_alarms(k) = ...
        nnz(detection.alarm(before_attack));

    % Zapis pelnych przebiegow
    scenarios(k).type = attack_types{k};
    scenarios(k).attack = attack;
    scenarios(k).detection = detection;
    scenarios(k).first_detection = first_detection(k);
    scenarios(k).detection_delay = detection_delay(k);

    fprintf('Maksymalne residuum: %.6f rad/s\n', ...
        max_residual(k));

    if isnan(first_detection(k))

        fprintf('Atak nie zostal wykryty.\n');

    else

        fprintf('Pierwsza detekcja: %.6f s\n', ...
            first_detection(k));

        fprintf('Opoznienie: %.6f s\n', ...
            detection_delay(k));

    end

    fprintf('Liczba probek z alarmem: %d\n', ...
        alarm_samples(k));

    fprintf('Falszywe alarmy przed atakiem: %d\n', ...
        false_alarms(k));

end

%% 7. TABELA POROWNAWCZA

results_table = table( ...
    string(attack_types(:)), ...
    first_detection, ...
    detection_delay, ...
    max_residual, ...
    mean_residual, ...
    alarm_samples, ...
    false_alarms, ...
    'VariableNames', { ...
        'AttackType', ...
        'FirstDetection_s', ...
        'DetectionDelay_s', ...
        'MaxResidual_rad_s', ...
        'MeanResidual_rad_s', ...
        'AlarmSamples', ...
        'FalseAlarms'});

fprintf('\n========================================\n');
fprintf('POROWNANIE SCENARIUSZY FDI\n');
fprintf('========================================\n');

disp(results_table);

%% 8. WERYFIKACJA

assert(all(false_alarms == 0), ...
    'Wystapily falszywe alarmy przed atakiem.');

assert(all(~isnan(first_detection)), ...
    'Nie wszystkie ataki zostaly wykryte.');

assert(all(first_detection >= p.fdi.start_time), ...
    'Detekcja wystapila przed poczatkiem ataku.');

fprintf('\nTEST FDI DP SCENARIOS: OK\n');

%% 9. WYKRES - POMIARY

figure('Name', 'FDI DP - porownanie pomiarow');

plot(t, omega_plant, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Predkosc rzeczywista');

hold on;

plot(t, omega_dt, '--', ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Digital Twin');

for k = 1:n_scenarios

    plot(t, ...
        scenarios(k).attack.omega_measured, ...
        'LineWidth', 1.2, ...
        'DisplayName', upper(scenarios(k).type));

end

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('\omega [rad/s]');
title('Sterowanie DP - porownanie atakow FDI');
legend('Location', 'best');

%% 10. WYKRES - RESIDUA

figure('Name', 'FDI DP - residua');

hold on;

for k = 1:n_scenarios

    plot(t, ...
        scenarios(k).detection.residual, ...
        'LineWidth', 1.5, ...
        'DisplayName', upper(scenarios(k).type));

end

yline(p.detection.threshold, '--', ...
    'Prog detekcji');

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('Residuum [rad/s]');
title('Porownanie residuow detektora');
legend('Location', 'best');

%% 11. WYKRES - DECYZJE DETEKTORA

figure('Name', 'FDI DP - alarmy');

hold on;

for k = 1:n_scenarios

    stairs(t, ...
        double(scenarios(k).detection.alarm), ...
        'LineWidth', 1.5, ...
        'DisplayName', upper(scenarios(k).type));

end

xline(p.fdi.start_time, ':', 'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('Alarm');
ylim([-0.1 1.1]);
yticks([0 1]);
title('Decyzje detektora FDI');
legend('Location', 'best');

%% 12. ZAPIS WYNIKOW

fdi_scenarios_result.p = p;
fdi_scenarios_result.dp = dp;
fdi_scenarios_result.dp_time = dp_time;
fdi_scenarios_result.plant = plant;
fdi_scenarios_result.digital_twin = dt_model;
fdi_scenarios_result.scenarios = scenarios;
fdi_scenarios_result.results_table = results_table;

fprintf('\nTest zakonczony.\n');