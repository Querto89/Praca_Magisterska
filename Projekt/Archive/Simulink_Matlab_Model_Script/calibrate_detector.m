function [calib, info] = calibrate_detector(p, Uopt, plant)
%CALIBRATE_DETECTOR  Wyznacza podloge residuum na przebiegach BEZ ATAKU.
%
%  METODA
%  ------
%  Uruchamiamy n_calib przebiegow bez ataku, z roznymi realizacjami szumu,
%  i zbieramy residuum r(k). Dla kazdego kroku k liczymy:
%
%      mu(k)    - obciazenie wynikajace z NIEDOPASOWANIA PARAMETROW
%                 (deterministyczne, powtarzalne)
%      sigma(k) - rozrzut wynikajacy z SZUMU CZUJNIKOW
%
%  Prog detekcji to k_sigma * sigma(k) wokol mu(k). Dzieki temu prog jest
%  UZASADNIONY statystycznie, a nie wybrany arbitralnie - w pracy mozna
%  to powiazac z zalozonym poziomem falszywych alarmow.
%
%  UWAGA METODOLOGICZNA
%  --------------------
%  Kalibracja MUSI byc wykonana na danych bez ataku. Jesli atakujacy
%  jest obecny podczas kalibracji, detektor "nauczy sie" ataku jako normy
%  (to ciekawy watek na dyskusje - atak na faze uczenia Digital Twina).

    if nargin < 3 || isempty(plant)
        plant = plant_matlab(p);   % kalibracja zawsze szybkim obiektem
    end

    M = p.det.n_calib;

    attack_none      = p.attack;
    attack_none.type = 'none';

    R = zeros(M, p.N);

    fprintf('  Kalibracja detektora: %d przebiegow bez ataku...\n', M);

    for m = 1:M
        log_m = simulate_system(p, Uopt, plant, attack_none, [], 1000 + m);
        R(m, :) = log_m.r;
    end

    calib.mu = mean(R, 1);
    calib.sd = std(R, 0, 1);

    % Zabezpieczenie przed sigma = 0 (np. gdy szum wylaczony)
    floor_sd = max(1e-6, 0.01 * max(abs(calib.mu)));
    calib.sd = max(calib.sd, floor_sd);

    %% =========================================================
    %  EMPIRYCZNY FALSE ALARM RATE
    %  =========================================================

    Z = abs((R - calib.mu) ./ calib.sd);

    far_sample = mean(Z(:) > p.det.k_sigma);

    % FAR na poziomie przebiegu - PELNA logika detektora
    % (prog + persystencja + dwustronny CUSUM)
    Zs = (R - calib.mu) ./ calib.sd;    % residuum znormalizowane ZE ZNAKIEM

    fa_runs = 0;
    for m = 1:M
        c = 0; Sp = 0; Sm = 0; hit = false;
        for k = 1:p.N
            z = Zs(m, k);

            if abs(z) > p.det.k_sigma, c = c + 1; else, c = 0; end

            if p.det.use_cusum
                Sp = max(0, Sp + z - p.det.cusum_k);
                Sm = max(0, Sm - z - p.det.cusum_k);
            end

            if c >= p.det.persist || max(Sp, Sm) > p.det.cusum_h
                hit = true; break;
            end
        end
        fa_runs = fa_runs + hit;
    end
    far_run = fa_runs / M;

    info.R            = R;
    info.bias_mean    = mean(calib.mu);
    info.bias_max     = max(abs(calib.mu));
    info.sigma_mean   = mean(calib.sd);
    info.far_sample   = far_sample;
    info.far_run      = far_run;
    info.thr_rad_s    = p.det.k_sigma * mean(calib.sd);

    fprintf('    Obciazenie od niedopasowania : mean %.4f | max %.4f rad/s\n', ...
        info.bias_mean, info.bias_max);
    fprintf('    Odchylenie od szumu          : %.4f rad/s\n', info.sigma_mean);
    fprintf('    Prog (%.1f sigma)             : %.4f rad/s\n', ...
        p.det.k_sigma, info.thr_rad_s);
    fprintf('    FAR (probka)                 : %.2e\n', far_sample);
    fprintf('    FAR (przebieg)               : %.2e\n', far_run);

end
