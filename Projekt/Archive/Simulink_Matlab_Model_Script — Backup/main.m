clear;
clc;
close all;

%% =========================================================
%  PARAMETRY
%  =========================================================

p = params();

%% =========================================================
%  OPTYMALIZACJA DP
%  =========================================================

fprintf('\n========================================\n');
fprintf('Dynamic Programming\n');
fprintf('========================================\n\n');

Uopt = dp_solver(p);

%% =========================================================
%  DIGITAL TWIN - MATLAB
%  =========================================================

fprintf('\n========================================\n');
fprintf('Digital Twin - MATLAB\n');
fprintf('========================================\n\n');

[t_dt, omega_ref, i_ref, T_ref, U_ref, E_ref] = ...
    run_matlab(p, Uopt);

%% =========================================================
%  OBIEKT - SIMULINK
%  =========================================================

fprintf('\n========================================\n');
fprintf('Obiekt fizyczny - Simulink\n');
fprintf('========================================\n\n');

[w_sim, I_sim, T_sim, E_sim, U_sim, ...
    t_sim, attack_detected] = ...
    run_simulink(p, Uopt);

%% =========================================================
%  DOPASOWANIE WYMIAROW
%  =========================================================

omega_sim = w_sim.dp(:);

omega_ref_col = omega_ref(:);

N_compare = min( ...
    length(omega_sim), ...
    length(omega_ref_col));

omega_sim = omega_sim(1:N_compare);
omega_ref_col = omega_ref_col(1:N_compare);

t_compare = t_dt(1:N_compare);

%% =========================================================
%  RESIDUUM
%  =========================================================

residual = ...
    abs(omega_sim - omega_ref_col);

%% =========================================================
%  WERYFIKACJA ZGODNOSCI
%  =========================================================

max_error = max(residual);

rmse_error = ...
    sqrt(mean(residual.^2));

fprintf('\n========================================\n');
fprintf('WERYFIKACJA MATLAB <-> SIMULINK\n');
fprintf('========================================\n');

fprintf('Maksymalny blad predkosci = %.6e rad/s\n', ...
    max_error);

fprintf('RMSE = %.6e rad/s\n', ...
    rmse_error);

%% =========================================================
%  WIZUALIZACJA
%  =========================================================

plots_security( ...
    p, ...
    omega_sim', ...
    omega_ref_col', ...
    attack_detected(:)');