clear; clc; close all;

%% =========================================================================
%  TESTY JEDNOSTKOWE / SANITY CHECKS
%  =========================================================================
%
%  NAPRAWIONY BLAD Z WERSJI 1
%  --------------------------
%  Poprzedni test zakladal:
%      expected_size = [length(p.omega_grid), p.N];
%  czyli polityke 2-wymiarowa. Tymczasem dp_solver zwraca tablice
%  3-wymiarowa (Ni x Nw x N), bo stan jest dwuwymiarowy: x = [i_a; omega].
%  Asercja zawsze by sie wywalila. Ponizej poprawna weryfikacja.

p = params();

fprintf('=========================================================\n');
fprintf(' TESTY\n');
fprintf('=========================================================\n\n');

%% -------------------------------------------------------------------------
%  TEST 1: parametry obiektu roznia sie od nominalnych
%  -------------------------------------------------------------------------

check('Obiekt ma inne parametry niz model nominalny', ...
    p.plant.Ra ~= p.nom.Ra && p.plant.J ~= p.nom.J);

%% -------------------------------------------------------------------------
%  TEST 2: stabilnosc numeryczna kroku calkowania
%  -------------------------------------------------------------------------
%  Sprawdza wlasnie ten blad, ktory psul oryginalny projekt:
%  jawny Euler przy dt = 0.05 s dla lambda = -20+-20j daje |1+lambda*dt| = 1.

[~, ~, A_nom] = motor_model(p.nom, p.dt);
lam = eig(A_nom);

euler_ctrl_rate = max(abs(1 + lam * p.dt));
euler_fine_rate = max(abs(1 + lam * p.plant.h));

fprintf('  Wartosci wlasne obiektu: %s\n', mat2str(round(lam, 3)));
fprintf('  |1+lambda*dt| przy dt=%.3f s : %.4f  (oryginalny blad)\n', ...
    p.dt, euler_ctrl_rate);
fprintf('  |1+lambda*h|  przy h =%.4f s: %.4f  (po poprawce)\n\n', ...
    p.plant.h, euler_fine_rate);

check('Krok calkowania obiektu jest numerycznie stabilny', ...
    euler_fine_rate < 0.999);

%% -------------------------------------------------------------------------
%  TEST 3: DP zwraca polityke o poprawnych wymiarach
%  -------------------------------------------------------------------------

fprintf('  Obliczanie DP...\n');
tic; [Uopt, V] = dp_solver(p); t_dp = toc;
fprintf('  Czas DP: %.2f s\n\n', t_dp);

expected = [numel(p.i_grid), numel(p.omega_grid), p.N];

check('Wymiary Uopt = [Ni x Nw x N]', isequal(size(Uopt), expected));
check('Wymiary V    = [Ni x Nw x N+1]', ...
    isequal(size(V), [numel(p.i_grid), numel(p.omega_grid), p.N + 1]));

%% -------------------------------------------------------------------------
%  TEST 4: sterowanie miesci sie w ograniczeniach
%  -------------------------------------------------------------------------

check('Uopt w zakresie [Umin, Umax]', ...
    all(Uopt(:) >= p.Umin - 1e-9) && all(Uopt(:) <= p.Umax + 1e-9));

%% -------------------------------------------------------------------------
%  TEST 5: obiekt MATLAB osiaga sensowny stan ustalony
%  -------------------------------------------------------------------------
%  Analitycznie: omega_ss = Kt*U / (Ra*B + Kt*Ke)

plant = plant_matlab(p);
U_test = 8.0;
x = plant.init();
for k = 1:200
    x = plant.step(x, U_test);
end

omega_ss_teor = p.plant.Kt * U_test / ...
    (p.plant.Ra * p.plant.B + p.plant.Kt * p.plant.Ke);

err_ss = abs(x(2) - omega_ss_teor);

fprintf('  Stan ustalony: symulacja %.4f | teoria %.4f rad/s\n\n', ...
    x(2), omega_ss_teor);

check('Stan ustalony zgodny z rozwiazaniem analitycznym', err_ss < 1e-6);

%% -------------------------------------------------------------------------
%  TEST 6: petla zamknieta bez ataku dochodzi do celu
%  -------------------------------------------------------------------------

attack_none = p.attack; attack_none.type = 'none';
log_nom = simulate_system(p, Uopt, plant, attack_none, []);

fprintf('  Predkosc koncowa DP: %.3f rad/s (cel %.1f)\n\n', ...
    log_nom.w_final, p.omega_target);

check('DP dochodzi do predkosci zadanej (+-10%)', ...
    abs(log_nom.w_final - p.omega_target) < 0.1 * p.omega_target);

check('Prad nie przekracza ograniczenia', ...
    max(log_nom.i_true) <= p.Imax * 1.05);

%% -------------------------------------------------------------------------
%  TEST 7: kalibracja daje niski false alarm rate
%  -------------------------------------------------------------------------

p_fast = p; p_fast.det.n_calib = 100;
[calib, info] = calibrate_detector(p_fast, Uopt, plant);

check('FAR na poziomie przebiegu < 5%', info.far_run < 0.05);

%% -------------------------------------------------------------------------
%  TEST 8: silny atak jest wykrywany
%  -------------------------------------------------------------------------

att = p.attack; att.type = 'bias'; att.bias = 1.0;
log_att = simulate_system(p, Uopt, plant, att, calib);

check('Atak bias 1.0 rad/s zostaje wykryty', log_att.detected);

%% -------------------------------------------------------------------------
%  TEST 9: brak ataku -> brak alarmu
%  -------------------------------------------------------------------------

log_clean = simulate_system(p, Uopt, plant, attack_none, calib);

check('Brak alarmu bez ataku', ~log_clean.detected);

%% =========================================================================
%  PODSUMOWANIE
%  =========================================================================

res_counts = check('__summary__', true);

fprintf('\n=========================================================\n');
fprintf(' WYNIK: %d zaliczonych, %d niezaliczonych\n', ...
    res_counts(1), res_counts(2));
fprintf('=========================================================\n');

%% -------------------------------------------------------------------------
function out = check(name, cond)
%CHECK  Pojedynczy test + licznik wynikow (persistent).

    persistent np nf

    if isempty(np), np = 0; nf = 0; end

    if strcmp(name, '__summary__')
        out = [np, nf];
        np = []; nf = [];      % reset na kolejne uruchomienie
        return;
    end

    if cond
        fprintf('  [ OK ]   %s\n', name);
        np = np + 1;
    else
        fprintf('  [FAIL]   %s\n', name);
        nf = nf + 1;
    end

    out = logical(cond);

end
