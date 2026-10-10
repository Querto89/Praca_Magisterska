%% EXPERIMENT_01_VALIDATION
% Walidacja cyfrowego blizniaka (MATLAB) wzgledem niezaleznego
% modelu obiektu DC w Simulinku dla identycznego sterowania DP.
%
% Uruchom z dowolnego katalogu:
%   run('.../experiments/experiment_01_validation.m')
%
% Wymagane pliki projektu:
%   params.m, run_matlab.m, run_simulink.m, system_model.slx,
%   DP/dp_solver.m, calculate_metrics.m

%% 1. Sciezki projektu
% Plik eksperymentu znajduje sie w katalogu experiments/.
experiment_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(experiment_dir);

addpath(project_root);
addpath(fullfile(project_root, 'DP'));
addpath(fullfile(project_root, 'classic_methods'));
addpath(fullfile(project_root, 'FDI'));
addpath(fullfile(project_root, 'tests'));

fprintf('\n========================================\n');
fprintf('EKSPERYMENT 01: WALIDACJA DIGITAL TWIN\n');
fprintf('========================================\n');

%% 2. Wspolne parametry i optymalne sterowanie
p = params();

fprintf('\nWyznaczanie sterowania DP...\n');
tic;
dp = dp_solver(p);
dp_time_s = toc;

u_dp = dp.u_opt(:);
assert(numel(u_dp) == p.sim.N, ...
    'DP zwrocilo nieprawidlowa liczbe probek sterowania.');

%% 3. Dwie niezalezne symulacje z tym samym wektorem u
fprintf('\nSymulacja Digital Twin (MATLAB)...\n');
dt_result = run_matlab(p, u_dp);

fprintf('Symulacja obiektu (Simulink)...\n');
sim_result = run_simulink(p, u_dp);

%% 4. Wspolna os czasu
% Porownujemy stany w chwilach dyskretnych t_k cyfrowego blizniaka.
% Simulink stosuje zmienny krok, wiec jego sygnaly interpolujemy
% na chwile t_k. Nie porownujemy interpolowanego DT na gestszej
% osi Simulinka, co wprowadzaloby sztuczny blad interpolacji.
t = dt_result.t(:);

omega_sim = interp1(sim_result.t(:), sim_result.omega(:), ...
    t, 'linear');
i_sim = interp1(sim_result.t(:), sim_result.i_a(:), ...
    t, 'linear');
E_sim = interp1(sim_result.t(:), sim_result.E(:), ...
    t, 'linear');

assert(all(isfinite([omega_sim; i_sim; E_sim])), ...
    'Nie udalo sie wyznaczyc wynikow na wspolnej osi czasu.');

%% 5. Wskazniki zgodnosci
omega_error = dt_result.omega(:) - omega_sim;
i_error = dt_result.i_a(:) - i_sim;
E_error = dt_result.E(:) - E_sim;

validation.max_omega_error_rad_s = max(abs(omega_error));
validation.mean_omega_error_rad_s = mean(abs(omega_error));
validation.rmse_omega_rad_s = sqrt(mean(omega_error.^2));
validation.max_current_error_A = max(abs(i_error));
validation.mean_current_error_A = mean(abs(i_error));
validation.energy_DT_J = dt_result.E(end);
validation.energy_Simulink_J = sim_result.E(end);
validation.energy_difference_J = abs(validation.energy_DT_J ...
    - validation.energy_Simulink_J);
validation.energy_difference_percent = 100 * ...
    validation.energy_difference_J / max(abs(validation.energy_Simulink_J), eps);
validation.omega_final_DT_rad_s = dt_result.omega(end);
validation.omega_final_Simulink_rad_s = sim_result.omega(end);
validation.dp_time_s = dp_time_s;

% Te same wskazniki jakosci liczone osobno dla obu symulacji.
metrics_DT = calculate_metrics(p, dt_result);
metrics_Simulink = calculate_metrics(p, sim_result);

%% 6. Kryteria zaliczenia walidacji
% Tolerancje odnosza sie do zgodnosci dwoch implementacji,
% a nie do bledu regulacji wzgledem predkosci zadanej.
tol_omega = 1e-3;   % rad/s
tol_current = 1e-3; % A
tol_energy = 0.05;  % J

validation.speed_OK = validation.max_omega_error_rad_s < tol_omega;
validation.current_OK = validation.max_current_error_A < tol_current;
validation.energy_OK = validation.energy_difference_J < tol_energy;
validation.passed = validation.speed_OK && ...
    validation.current_OK && validation.energy_OK;

%% 7. Wyniki liczbowe
fprintf('\n========================================\n');
fprintf('ZGODNOSC MODELI\n');
fprintf('========================================\n');
fprintf('Maks. blad predkosci:  %.9f rad/s\n', ...
    validation.max_omega_error_rad_s);
fprintf('Sredni blad predkosci: %.9f rad/s\n', ...
    validation.mean_omega_error_rad_s);
fprintf('RMSE predkosci:        %.9f rad/s\n', ...
    validation.rmse_omega_rad_s);
fprintf('Maks. blad pradu:      %.9f A\n', ...
    validation.max_current_error_A);
fprintf('Energia Digital Twin:  %.9f J\n', ...
    validation.energy_DT_J);
fprintf('Energia Simulink:      %.9f J\n', ...
    validation.energy_Simulink_J);
fprintf('Roznica energii:       %.9f J (%.5f%%)\n', ...
    validation.energy_difference_J, ...
    validation.energy_difference_percent);
fprintf('Predkosc koncowa DT:   %.9f rad/s\n', ...
    validation.omega_final_DT_rad_s);
fprintf('Predkosc koncowa SIM:  %.9f rad/s\n', ...
    validation.omega_final_Simulink_rad_s);

fprintf('\n========================================\n');
fprintf('KRYTERIA WALIDACJI\n');
fprintf('========================================\n');
fprintf('Predkosc (< %.4g rad/s): %d\n', tol_omega, validation.speed_OK);
fprintf('Prad (< %.4g A):       %d\n', tol_current, validation.current_OK);
fprintf('Energia (< %.4g J):    %d\n', tol_energy, validation.energy_OK);

%% 8. Wykresy: bez narzucania kolorow tla ani motywu MATLAB-a
figure('Name', 'Eksperyment 01 - walidacja Digital Twin');

subplot(3,1,1);
plot(t, dt_result.omega, '-', t, omega_sim, '--', 'LineWidth', 1.2);
grid on;
ylabel('\omega [rad/s]');
title('Predkosc katowa');
legend('Digital Twin', 'Simulink', 'Location', 'best');

subplot(3,1,2);
plot(t, dt_result.i_a, '-', t, i_sim, '--', 'LineWidth', 1.2);
grid on;
ylabel('i_a [A]');
title('Prad twornika');
legend('Digital Twin', 'Simulink', 'Location', 'best');

subplot(3,1,3);
plot(t, dt_result.E, '-', t, E_sim, '--', 'LineWidth', 1.2);
grid on;
xlabel('Czas [s]');
ylabel('E [J]');
title('Energia elektryczna');
legend('Digital Twin', 'Simulink', 'Location', 'best');

figure('Name', 'Eksperyment 01 - bledy zgodnosci');
subplot(2,1,1);
plot(t, omega_error, 'LineWidth', 1.2);
grid on;
ylabel('e_\omega [rad/s]');
title('Roznica predkosci: Digital Twin - Simulink');

subplot(2,1,2);
plot(t, i_error, 'LineWidth', 1.2);
grid on;
xlabel('Czas [s]');
ylabel('e_i [A]');
title('Roznica pradu: Digital Twin - Simulink');

%% 9. Zapis wynikow eksperymentu
results_dir = fullfile(project_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

save(fullfile(results_dir, 'experiment_01_validation.mat'), ...
    'p', 'u_dp', 'dt_result', 'sim_result', ...
    'metrics_DT', 'metrics_Simulink', 'validation', ...
    't', 'omega_error', 'i_error', 'E_error');

if validation.passed
    fprintf('\nEXPERIMENT_01_VALIDATION: OK\n');
else
    warning('EXPERIMENT_01_VALIDATION: przekroczono tolerancje.');
end
fprintf('Wyniki zapisano w: %s\n', results_dir);
