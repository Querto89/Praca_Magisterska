clear;
clc;
close all;

%% ============================================================
%  ANALIZA WPLYWU ROZDZIELCZOSCI U_grid
%  ============================================================

p_base = params();

% Liczba punktow siatki sterowania
U_grid_sizes = [16 31 61];

n_cases = length(U_grid_sizes);

% Wyniki
E       = zeros(n_cases,1);
IAE     = zeros(n_cases,1);
ISE     = zeros(n_cases,1);
overshoot = zeros(n_cases,1);
settling  = zeros(n_cases,1);
u_max     = zeros(n_cases,1);
i_max     = zeros(n_cases,1);
calc_time = zeros(n_cases,1);

% Zachowanie przebiegow
results = cell(n_cases,1);

omega_ref = p_base.reference.omega;

%% ============================================================
%  PETLA EKSPERYMENTALNA
%  ============================================================

for n = 1:n_cases

    fprintf('\n\n');
    fprintf('========================================\n');
    fprintf('TEST U_grid: %d punktow\n', U_grid_sizes(n));
    fprintf('========================================\n');

    % Kopia parametrow bazowych
    p = p_base;

    % Zmieniamy TYLKO rozdzielczosc sterowania
    p.dp.U_grid = linspace( ...
        p.limits.Umin, ...
        p.limits.Umax, ...
        U_grid_sizes(n));

    %% DP

    tic;

    dp = dp_solver(p);

    calc_time(n) = toc;

    %% Walidacja sterowania na modelu Simulink

    sim_dp = run_simulink(p, dp.u_opt);

    results{n} = sim_dp;

    %% Energia

    E(n) = sim_dp.E(end);

    %% Uchyb predkosci

    error = omega_ref - sim_dp.omega;

    IAE(n) = trapz( ...
        sim_dp.t, ...
        abs(error));

    ISE(n) = trapz( ...
        sim_dp.t, ...
        error.^2);

    %% Przeregulowanie

    overshoot(n) = max(0, ...
        (max(sim_dp.omega) - omega_ref) ...
        / omega_ref * 100);

    %% Czas regulacji +/- 2 %

    tol = 0.02 * omega_ref;

    idx = find(abs(error) > tol);

    if isempty(idx)

        settling(n) = 0;

    elseif idx(end) < length(sim_dp.t)

        settling(n) = sim_dp.t(idx(end)+1);

    else

        settling(n) = NaN;

    end

    %% Maksymalne wartosci

    u_max(n) = max(sim_dp.u);
    i_max(n) = max(sim_dp.i_a);

end

%% ============================================================
%  TABELA WYNIKOW
%  ============================================================

ResultsTable = table( ...
    U_grid_sizes(:), ...
    24 ./ (U_grid_sizes(:)-1), ...
    E, ...
    IAE, ...
    ISE, ...
    overshoot, ...
    settling, ...
    u_max, ...
    i_max, ...
    calc_time, ...
    'VariableNames', { ...
    'N_U', ...
    'Delta_U', ...
    'Energy_J', ...
    'IAE', ...
    'ISE', ...
    'Overshoot_percent', ...
    'SettlingTime_s', ...
    'UmaxUsed_V', ...
    'Imax_A', ...
    'CalculationTime_s'});

fprintf('\n\n');
fprintf('========================================\n');
fprintf('ANALIZA U_grid\n');
fprintf('========================================\n');

disp(ResultsTable);

figure;

hold on;
grid on;

for n = 1:n_cases

    plot( ...
        results{n}.t, ...
        results{n}.omega, ...
        'LineWidth', 1.2);

end

yline(omega_ref, ':', ...
    '\omega_{ref}');

xlabel('t [s]');
ylabel('\omega [rad/s]');

legend( ...
    'N_U = 16', ...
    'N_U = 31', ...
    'N_U = 61', ...
    '\omega_{ref}', ...
    'Location', 'best');

title('Wplyw rozdzielczosci U_{grid} na predkosc');

figure;

hold on;
grid on;

for n = 1:n_cases

    stairs( ...
        results{n}.t, ...
        results{n}.u, ...
        'LineWidth', 1.2);

end

xlabel('t [s]');
ylabel('u_a [V]');

legend( ...
    'N_U = 16', ...
    'N_U = 31', ...
    'N_U = 61', ...
    'Location', 'best');

title('Wplyw rozdzielczosci U_{grid} na sterowanie');