clear; clc; close all;

%% =========================================================================
%  DIGITAL TWIN NAPEDU DC: OPTYMALIZACJA DP + DETEKCJA ATAKU FDI
%  =========================================================================
%
%  ARCHITEKTURA
%  ------------
%    OBIEKT (Simulink, parametry PRAWDZIWE p.plant)
%        -> czujniki (szum) -> ATAK FDI -> pomiar y
%              |                              |
%              |                              +-> DIGITAL TWIN (p.nom)
%              |                                     -> residuum -> ALARM
%              +-> regulator DP (Uopt) -> u -> OBIEKT
%
%  Model nominalny (Digital Twin, DP) i obiekt rzeczywisty maja CELOWO
%  rozne parametry - to odwzorowuje bledy identyfikacji i sprawia, ze
%  dobor progu detekcji jest realnym problemem projektowym.
%
%  KOLEJNOSC:
%    1. Optymalizacja DP
%    2. Weryfikacja numeryczna obiektu (Simulink vs MATLAB)
%    3. Kalibracja detektora na przebiegach bez ataku
%    4. Scenariusz nominalny (DP vs baseline)
%    5. Scenariusz z atakiem FDI
%    6. Kampania atakow (opcjonalnie)
%  =========================================================================

RUN_CAMPAIGN = true;    % pelna kampania Monte Carlo (dluzsza)

p = params();

%% =========================================================================
%  1. OPTYMALIZACJA DP
%  =========================================================================

fprintf('\n=========================================================\n');
fprintf(' 1. PROGRAMOWANIE DYNAMICZNE\n');
fprintf('=========================================================\n');

tic;
[Uopt, Vdp] = dp_solver(p);
fprintf('  Czas obliczen DP: %.2f s\n', toc);
fprintf('  Rozmiar polityki Uopt: %d x %d x %d\n', size(Uopt));

%% =========================================================================
%  2. WERYFIKACJA NUMERYCZNA OBIEKTU
%  =========================================================================

fprintf('\n=========================================================\n');
fprintf(' 2. WERYFIKACJA OBIEKTU: SIMULINK vs MATLAB\n');
fprintf('=========================================================\n');

if p.use_simulink
    try
        ver_res = verify_plant(p);
    catch ME
        warning('main:simulinkFailed', ...
            'Simulink niedostepny lub blad modelu (%s). Przechodze na obiekt MATLAB.', ...
            ME.message);
        p.use_simulink = false;
    end
else
    fprintf('  Pominieto (p.use_simulink = false).\n');
end

%% =========================================================================
%  3. KALIBRACJA DETEKTORA (PRZEBIEGI BEZ ATAKU)
%  =========================================================================

fprintf('\n=========================================================\n');
fprintf(' 3. KALIBRACJA DETEKTORA\n');
fprintf('=========================================================\n');

[calib, calib_info] = calibrate_detector(p, Uopt);

p.residual_threshold = calib_info.thr_rad_s;   % zgodnosc wsteczna

%% =========================================================================
%  4. SCENARIUSZ NOMINALNY (BEZ ATAKU)
%  =========================================================================

fprintf('\n=========================================================\n');
fprintf(' 4. SCENARIUSZ NOMINALNY (BEZ ATAKU)\n');
fprintf('=========================================================\n');

if p.use_simulink
    plant = plant_simulink(p);
else
    plant = plant_matlab(p);
end
fprintf('  Obiekt: %s\n', plant.name);

attack_none      = p.attack;
attack_none.type = 'none';

log_nom = simulate_system(p, Uopt, plant, attack_none, calib);

% Baseline: stale napiecie dajace ten sam punkt pracy w stanie ustalonym
U_base   = p.omega_target * (p.nom.Ra*p.nom.B + p.nom.Kt*p.nom.Ke) / p.nom.Kt;
U_base   = min(max(U_base, p.Umin), p.Umax);
Uopt_base = repmat(U_base, size(Uopt));

log_base = simulate_system(p, Uopt_base, plant, attack_none, calib);

fprintf('  DP       : E = %7.2f J | omega_end = %5.2f | J_total = %8.2f\n', ...
    log_nom.E_total, log_nom.w_final, log_nom.J_total);
fprintf('  Baseline : E = %7.2f J | omega_end = %5.2f | J_total = %8.2f  (u = %.2f V)\n', ...
    log_base.E_total, log_base.w_final, log_base.J_total, U_base);
fprintf('  Zysk DP wzgledem baseline: %.1f%% kosztu calkowitego\n', ...
    100*(log_base.J_total - log_nom.J_total)/max(1e-9, log_base.J_total));

if log_nom.detected
    fprintf('  UWAGA: FALSZYWY ALARM w scenariuszu bez ataku (t = %.2f s).\n', ...
        log_nom.t_detect);
else
    fprintf('  Brak falszywych alarmow.\n');
end

plots_nominal(p, log_nom, log_base);

%% =========================================================================
%  5. SCENARIUSZ Z ATAKIEM FDI
%  =========================================================================

fprintf('\n=========================================================\n');
fprintf(' 5. SCENARIUSZ Z ATAKIEM FDI\n');
fprintf('=========================================================\n');

fprintf('  Typ ataku : %s\n', p.attack.type);
fprintf('  Start     : %.2f s\n', p.attack.start);

log_att = simulate_system(p, Uopt, plant, p.attack, calib);

if log_att.detected
    fprintf('  DETEKCJA: tak, t = %.2f s (opoznienie %.2f s = %d krokow)\n', ...
        log_att.t_detect, log_att.delay_detect, ...
        round(log_att.delay_detect / p.dt));
else
    fprintf('  DETEKCJA: NIE - atak pozostal niewykryty (stealthy).\n');
end

fprintf('  Energia   : %.2f J (bez ataku: %.2f J, zmiana %+.1f%%)\n', ...
    log_att.E_total, log_nom.E_total, ...
    100*(log_att.E_total - log_nom.E_total)/max(1e-9, log_nom.E_total));

fprintf('  Predkosc koncowa: %.2f rad/s (bez ataku: %.2f rad/s)\n', ...
    log_att.w_final, log_nom.w_final);

plots_security(p, log_att, calib);

if p.use_simulink
    plant.close();
end

%% =========================================================================
%  6. KAMPANIA ATAKOW
%  =========================================================================

if RUN_CAMPAIGN

    fprintf('\n=========================================================\n');
    fprintf(' 6. KAMPANIA ATAKOW (MONTE CARLO)\n');
    fprintf('=========================================================\n');

    Tres = attack_campaign(p, Uopt, calib, 30);

    fprintf('\n  Tabela wynikow:\n\n');
    disp(Tres);

    writetable(Tres, 'wyniki_kampanii.csv');
    fprintf('  Zapisano: wyniki_kampanii.csv\n');

end

fprintf('\n=========================================================\n');
fprintf(' KONIEC\n');
fprintf('=========================================================\n');
