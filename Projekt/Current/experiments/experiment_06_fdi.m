
%% EXPERIMENT 06 - detekcja FDI w pomiarze predkosci
% Test otwartej petli: obiekt i Digital Twin dostaja to samo sterowanie DP.
% Atak modyfikuje WYLACZNIE pomiar predkosci, nie stan obiektu ani sterowanie.
% Wszystkie residua sa liczone na siatce czasu Digital Twin (dt.t).
clear; clc; close all;

%% Sciezki projektu
thisFile = mfilename('fullpath');
if isempty(thisFile)
    error('Zapisz ten skrypt jako experiment_06_fdi.m przed uruchomieniem.');
end

experimentDir = fileparts(thisFile);
projectDir = fileparts(experimentDir);

if exist(fullfile(projectDir,'params.m'),'file') ~= 2
    projectDir = experimentDir;
end

addpath(projectDir);

resultDir = fullfile(projectDir,'results','experiment_06');
if ~exist(resultDir,'dir')
    mkdir(resultDir);
end

%% Parametry eksperymentu
p = params();

p.fdi.enabled = true;
p.fdi.start_time = 1.0;
p.fdi.magnitude = 2.0;       % rad/s
p.fdi.ramp_duration = 0.5;   % s: od 0 do 2 rad/s w 0.5 s

p.detection.threshold = 1.0; % rad/s

fprintf('\n========================================\n');
fprintf('EKSPERYMENT 06: DETEKCJA FDI\n');
fprintf('========================================\n');
fprintf('Start ataku: %.3f s | prog: %.3f rad/s\n', ...
    p.fdi.start_time, p.detection.threshold);
fprintf('Siatka detektora: dt = %.4f s\n', p.sim.dt);

%% Jeden sygnal DP dla obu modeli
fprintf('\nObliczanie sterowania DP...\n');

dp = dp_solver(p);
u = dp.u_opt(:);

fprintf('\nSymulacja Digital Twin (MATLAB)...\n');
dt = run_matlab(p,u);

fprintf('Symulacja niezaleznego obiektu (Simulink)...\n');
plant = run_simulink(p,u);

%% Porownanie na wspolnej siatce czasowej Digital Twin
% Interpolujemy sygnal Simulinka do chwil probkowania DT.
% Nie interpolujemy 41 probek DT na tysiace probek Simulinka.

t = dt.t(:);
wTwin = dt.omega(:);

tPlant = plant.t(:);
wPlant = plant.omega(:);

assert(numel(t) == numel(wTwin), ...
    'Niezgodne rozmiary dt.t i dt.omega.');

assert(numel(tPlant) == numel(wPlant), ...
    'Niezgodne rozmiary plant.t i plant.omega.');

assert(all(diff(t) > 0) && all(diff(tPlant) > 0), ...
    'Osie czasu musza byc rosnace.');

assert(t(1) >= tPlant(1)-1e-10 && ...
       t(end) <= tPlant(end)+1e-10, ...
    'Zakres czasu Simulinka nie pokrywa osi Digital Twin.');

wTrue = interp1(tPlant,wPlant,t,'linear');

assert(all(isfinite(wTrue)), ...
    'Interpolacja Simulinka zwrocila NaN/Inf.');

%% Walidacja bazowa bez ataku
rBase = wTrue - wTwin;

baseMax = max(abs(rBase));
baseRMSE = sqrt(mean(rBase.^2));

fprintf('\nWALIDACJA BAZOWA\n');
fprintf('Liczba probek DT: %d | Simulink: %d\n', ...
    numel(t),numel(tPlant));

fprintf('Max |rBase|: %.9f rad/s\n',baseMax);
fprintf('RMSE:        %.9f rad/s\n',baseRMSE);

if baseMax >= p.detection.threshold
    warning(['Blad bazowy przekracza prog detekcji. ' ...
        'Sprawdz synchronizacje, wejscie ZOH i zgodnosc modeli.']);
end

%% Scenariusze FDI
% NONE: brak ataku
% BIAS: skokowy blad pomiaru +2 rad/s od t=1 s
% RAMP: liniowy wzrost bledu od 0 do +2 rad/s w 0.5 s

scenarioNames = {'none','bias','ramp'};
n = numel(scenarioNames);

results = struct([]);

summary = table('Size',[n 10], ...
    'VariableTypes', ...
    {'string','double','double','double','logical', ...
     'double','double','logical','double','double'}, ...
    'VariableNames', ...
    {'Scenario', ...
     'AttackStart_s', ...
     'AttackAmplitude_rad_s', ...
     'Threshold_rad_s', ...
     'Detected', ...
     'FirstAlarm_s', ...
     'DetectionDelay_s', ...
     'FalseAlarmBeforeAttack', ...
     'MaxResidual_rad_s', ...
     'RMSEResidual_rad_s'});

for k = 1:n

    name = scenarioNames{k};

    attack = zeros(size(t));
    active = t >= p.fdi.start_time;

    switch name

        case 'none'
            attackStart = NaN;
            amplitude = 0;

        case 'bias'
            attack(active) = p.fdi.magnitude;

            attackStart = p.fdi.start_time;
            amplitude = p.fdi.magnitude;

        case 'ramp'
            attack(active) = p.fdi.magnitude * min( ...
                (t(active)-p.fdi.start_time) ...
                /p.fdi.ramp_duration,1);

            attackStart = p.fdi.start_time;
            amplitude = p.fdi.magnitude;

    end

    %% Pomiar zaatakowany i residuum
    wMeasured = wTrue + attack;

    residual = wMeasured - wTwin;

    alarm = abs(residual) > p.detection.threshold;

    %% Pierwszy alarm i opoznienie detekcji
    firstAlarmIdx = find(alarm,1,'first');

    if isempty(firstAlarmIdx)

        detected = false;
        firstAlarm = NaN;
        delay = NaN;

    else

        detected = true;
        firstAlarm = t(firstAlarmIdx);

        if isnan(attackStart)
            delay = NaN;
        else
            delay = firstAlarm - attackStart;
        end

    end

    %% Falszywe alarmy
    if isnan(attackStart)
        falseBefore = any(alarm);
    else
        falseBefore = any(alarm(t < attackStart));
    end

    %% Zachowanie pelnych przebiegow
    results(k).name = name;
    results(k).t = t;

    results(k).omega_plant = wTrue;
    results(k).omega_twin = wTwin;
    results(k).omega_measured = wMeasured;

    results(k).attack = attack;
    results(k).residual = residual;
    results(k).alarm = alarm;

    results(k).first_alarm_s = firstAlarm;
    results(k).detection_delay_s = delay;

    %% Tabela zbiorcza
    summary.Scenario(k) = string(name);
    summary.AttackStart_s(k) = attackStart;
    summary.AttackAmplitude_rad_s(k) = amplitude;

    summary.Threshold_rad_s(k) = p.detection.threshold;

    summary.Detected(k) = detected;
    summary.FirstAlarm_s(k) = firstAlarm;
    summary.DetectionDelay_s(k) = delay;

    summary.FalseAlarmBeforeAttack(k) = falseBefore;

    summary.MaxResidual_rad_s(k) = max(abs(residual));
    summary.RMSEResidual_rad_s(k) = sqrt(mean(residual.^2));

    fprintf(['%-5s | max |r|=%.6f | wykryto=%d ' ...
             '| alarm=%.3f s | opoznienie=%.3f s\n'], ...
        upper(name), ...
        summary.MaxResidual_rad_s(k), ...
        detected,firstAlarm,delay);

    %% Wykresy: kazdy scenariusz osobno
    fig = figure( ...
        'Name',['FDI: ' upper(name)], ...
        'Position',[100 100 1200 850]);

    tiledlayout(3,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    % Predkosc obiektu, Digital Twin i pomiar
    nexttile;

    plot(t,wTrue,'k-','LineWidth',1.2);
    hold on;

    plot(t,wTwin,'b--','LineWidth',1.2);
    plot(t,wMeasured,'r-','LineWidth',1.2);

    ylabel('\omega [rad/s]');
    title(['FDI: ' upper(name)]);

    legend( ...
        'Obiekt', ...
        'Digital Twin', ...
        'Pomiar (potencjalnie FDI)', ...
        'Location','best');

    grid on;
    xlim([t(1) t(end)]);

    % Sygnal ataku
    nexttile;

    stairs(t,attack,'m-','LineWidth',1.3);

    ylabel('a(t) [rad/s]');

    grid on;
    xlim([t(1) t(end)]);

    % Residuum i prog detekcji
    nexttile;

    plot(t,residual,'b-','LineWidth',1.3);
    hold on;

    yline(p.detection.threshold,'r--','Prog +');
    yline(-p.detection.threshold,'r--','Prog -');

    if ~isnan(attackStart)
        xline(attackStart,'k:','Start FDI');
    end

    if detected
        xline(firstAlarm,'g-','Alarm');
    end

    ylabel('r(t) [rad/s]');
    xlabel('Czas [s]');

    grid on;
    xlim([t(1) t(end)]);

    %% Zapis wykresu
    exportgraphics( ...
        fig, ...
        fullfile(resultDir,['fdi_' name '.png']), ...
        'Resolution',180);

end

%% Tabela i zapis wynikow
fprintf('\n========================================\n');
fprintf('PODSUMOWANIE DETEKCJI\n');
fprintf('========================================\n');

disp(summary);

writetable( ...
    summary, ...
    fullfile(resultDir,'fdi_summary.csv'));

save( ...
    fullfile(resultDir,'fdi_results.mat'), ...
    'p','dp','dt','plant','u', ...
    't','wTrue','wTwin','rBase', ...
    'baseMax','baseRMSE', ...
    'results','summary');

fprintf('\nEXPERIMENT_06: zakonczony. Wyniki: %s\n',resultDir);

fprintf(['UWAGA: to detekcja ataku na pomiar przy wspolnym ' ...
    'sterowaniu DP; nie jest to test wplywu FDI ' ...
    'na zamknieta petle.\n']);
