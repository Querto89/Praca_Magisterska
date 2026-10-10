
%% ============================================================
% POROWNANIE ATAKOW NA SYGNAL STERUJACY DP
%% ============================================================

% Zakladamy, ze wczesniej wykonano:
% p = params();
% dp = dp_solver(p);
% u_dp = dp.u_opt(:);

p.fdi.enabled = true;
p.fdi.start_time = 1.0;
p.fdi.magnitude = 2.0;
p.fdi.ramp_duration = 0.5;
p.fdi.pulse_duration = 0.3;

types = {'bias', 'ramp', 'pulse'};

t_u = (0:p.sim.N-1)' * p.sim.dt;

%% 1. SYMULACJA REFERENCYJNA

fprintf('\nSymulacja obiektu bez ataku...\n');

nominal = run_simulink(p, u_dp);

fprintf('Symulacja Digital Twin...\n');

twin = run_matlab(p, u_dp);

t = twin.t(:);
omega_twin = twin.omega(:);

omega_nominal = interp1( ...
    nominal.t(:), nominal.omega(:), ...
    t, 'linear', 'extrap');

fprintf('Maksymalny blad modeli: %.9f rad/s\n', ...
    max(abs(omega_nominal - omega_twin)));

%% 2. KOLEJNE SCENARIUSZE

results = struct([]);

for k = 1:numel(types)

    p_case = p;
    p_case.fdi.type = types{k};

    fprintf('\n========================================\n');
    fprintf('ATAK: %s\n', upper(types{k}));
    fprintf('========================================\n');

    attack = fdi_control_attack(t_u, u_dp, p_case);

    plant = run_simulink(p_case, attack.u_actual);

    omega = interp1( ...
        plant.t(:), plant.omega(:), ...
        t, 'linear', 'extrap');

    residual = abs(omega - omega_twin);

    alarm = residual > p_case.detection.threshold;

    idx = find( ...
        alarm & t >= p_case.fdi.start_time, ...
        1, 'first');

    if isempty(idx)
        detection_time = NaN;
        delay = NaN;
    else
        detection_time = t(idx);
        delay = detection_time - p_case.fdi.start_time;
    end

    before = t < p_case.fdi.start_time;

    results(k).type = types{k};
    results(k).attack = attack;
    results(k).plant = plant;
    results(k).omega = omega;
    results(k).residual = residual;
    results(k).alarm = alarm;

    results(k).detection_time = detection_time;
    results(k).delay = delay;
    results(k).max_residual = max(residual);
    results(k).false_alarms = nnz(alarm & before);

    results(k).energy = plant.E(end);
    results(k).delta_energy = ...
        plant.E(end) - nominal.E(end);

    results(k).omega_final = omega(end);
    results(k).omega_max = max(omega);

    fprintf('Maks. residuum: %.6f rad/s\n', ...
        results(k).max_residual);

    if isnan(detection_time)
        fprintf('Brak detekcji.\n');
    else
        fprintf('Detekcja: %.6f s\n', detection_time);
        fprintf('Opoznienie: %.6f s\n', delay);
    end

    fprintf('Energia: %.6f J\n', ...
        results(k).energy);

    fprintf('Zmiana energii: %+.6f J\n', ...
        results(k).delta_energy);

    fprintf('Predkosc koncowa: %.6f rad/s\n', ...
        results(k).omega_final);

    fprintf('Predkosc maksymalna: %.6f rad/s\n', ...
        results(k).omega_max);

    results(k).i_max = max(plant.i_a);

    results(k).current_limit_exceeded = ...
        results(k).i_max > p.limits.Imax + 1e-6;
    
    results(k).speed_limit_exceeded = ...
        results(k).omega_max > p.limits.omega_max + 1e-6;
    
    fprintf('Maksymalny prad: %.6f A\n', ...
        results(k).i_max);
    
    fprintf('Przekroczono limit pradu: %d\n', ...
        results(k).current_limit_exceeded);
    
    fprintf('Przekroczono limit predkosci: %d\n', ...
        results(k).speed_limit_exceeded);

    fprintf('Falszywe alarmy: %d\n', ...
        results(k).false_alarms);

end

%% 3. TABELA POROWNAWCZA

AttackType = string({results.type})';

DetectionTime_s = [results.detection_time]';
DetectionDelay_s = [results.delay]';
MaxResidual_rad_s = [results.max_residual]';

Energy_J = [results.energy]';
DeltaEnergy_J = [results.delta_energy]';

OmegaFinal_rad_s = [results.omega_final]';
OmegaMax_rad_s = [results.omega_max]';

FalseAlarms = [results.false_alarms]';

summary = table( ...
    AttackType, ...
    DetectionTime_s, ...
    DetectionDelay_s, ...
    MaxResidual_rad_s, ...
    Energy_J, ...
    DeltaEnergy_J, ...
    OmegaFinal_rad_s, ...
    OmegaMax_rad_s, ...
    FalseAlarms);

disp(summary);

%% 4. POROWNANIE PREDKOSCI

figure('Name', 'FDI DP - ataki na sterowanie');

subplot(2,1,1);

plot(t, omega_twin, '--', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Digital Twin');

hold on;

for k = 1:numel(types)

    plot(t, results(k).omega, ...
        'LineWidth', 1.3, ...
        'DisplayName', upper(types{k}));

end

yline(p.reference.omega, ':', ...
    'Predkosc zadana');

yline(p.limits.omega_max, ':', ...
    'Ograniczenie predkosci');

xline(p.fdi.start_time, ':', ...
    'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('\omega [rad/s]');
title('Predkosc podczas atakow na sterowanie DP');
legend('Location', 'best');

%% 5. POROWNANIE RESIDUOW

subplot(2,1,2);

hold on;

for k = 1:numel(types)

    plot(t, results(k).residual, ...
        'LineWidth', 1.4, ...
        'DisplayName', upper(types{k}));

end

yline(p.detection.threshold, '--', ...
    'Prog detekcji');

xline(p.fdi.start_time, ':', ...
    'Poczatek ataku');

grid on;
xlabel('Czas [s]');
ylabel('Residuum [rad/s]');
title('Porownanie residuow');
legend('Location', 'best');

fprintf('\nTEST FDI CONTROL SCENARIOS ZAKONCZONY.\n');