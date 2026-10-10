
%% TEST DIAGNOSTYCZNY: zgodnosc obiektu i Digital Twin
clc;
clear;
close all;

fprintf('\n========================================\n');
fprintf('TEST FDI: WALIDACJA BAZOWA\n');
fprintf('========================================\n');

%% 1. Sciezki projektu

thisFile = mfilename('fullpath');
experimentsDir = fileparts(thisFile);
projectDir = fileparts(experimentsDir);

addpath(projectDir);
addpath(fullfile(projectDir, 'classic_methods'));

%% 2. Parametry i sterowanie DP

p = params();

fprintf('\nObliczanie sterowania DP...\n');
dp = dp_solver(p);

u = dp.u_opt;

fprintf('\nLiczba probek sterowania: %d\n', numel(u));
fprintf('Minimalne napiecie: %.6f V\n', min(u));
fprintf('Maksymalne napiecie: %.6f V\n', max(u));

%% 3. Niezalezne symulacje

fprintf('\nSymulacja Digital Twin (MATLAB)...\n');
dt = run_matlab(p, u);

fprintf('Symulacja obiektu (Simulink)...\n');
plant = run_simulink(p, u);

%% 4. Diagnostyka struktur

fprintf('\n========================================\n');
fprintf('POLA WYNIKOW SYMULACJI\n');
fprintf('========================================\n');

fprintf('\nDigital Twin:\n');
disp(fieldnames(dt));

fprintf('\nObiekt Simulink:\n');
disp(fieldnames(plant));

%% 5. Zapis wynikow

diagnosticDir = fullfile(projectDir, 'results', ...
    'experiment_06_diagnostic');

if ~exist(diagnosticDir, 'dir')
    mkdir(diagnosticDir);
end

save(fullfile(diagnosticDir, 'baseline_raw.mat'), ...
    'p', 'u', 'dt', 'plant');

fprintf('\nWyniki zapisane w:\n%s\n', diagnosticDir);
fprintf('\nTEST DIAGNOSTYCZNY: zakonczony.\n');
