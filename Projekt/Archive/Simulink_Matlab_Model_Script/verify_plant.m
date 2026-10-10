function res = verify_plant(p, U_seq)
%VERIFY_PLANT  Weryfikacja numeryczna: Simulink vs MATLAB dla TEGO SAMEGO obiektu.
%
%  CO TO WLASCIWIE WERYFIKUJE
%  --------------------------
%  To NIE jest porownanie "obiekt vs Digital Twin" (tam roznica jest
%  zamierzona - inne parametry). Tutaj oba modele maja parametry
%  PRAWDZIWE (p.plant) i te same wymuszenia, wiec powinny dac
%  praktycznie identyczny wynik.
%
%  Niezgodnosc oznacza blad implementacji (zly solver, zle warunki
%  poczatkowe, zle przekazane parametry) - a nie zjawisko fizyczne.
%  W pracy to jest rozdzial "walidacja modelu symulacyjnego".

    if nargin < 2 || isempty(U_seq)
        % Wymuszenie pobudzajace obie dynamiki (elektryczna i mechaniczna)
        U_seq = p.Umax * [ones(1, round(p.N/2)), 0.4*ones(1, p.N - round(p.N/2))];
    end

    U_seq = U_seq(:)';

    %% =========================================================
    %  OBIEKT 1: MATLAB
    %  =========================================================

    pm = plant_matlab(p);
    x  = pm.init();
    Xm = zeros(2, p.N + 1); Xm(:, 1) = x;
    for k = 1:p.N
        x = pm.step(x, U_seq(k));
        Xm(:, k + 1) = x;
    end

    %% =========================================================
    %  OBIEKT 2: SIMULINK
    %  =========================================================

    ps = plant_simulink(p);
    x  = ps.init();
    Xs = zeros(2, p.N + 1); Xs(:, 1) = x;
    for k = 1:p.N
        x = ps.step(x, U_seq(k));
        Xs(:, k + 1) = x;
    end
    ps.close();

    %% =========================================================
    %  POROWNANIE
    %  =========================================================

    err_w = abs(Xm(2, :) - Xs(2, :));
    err_i = abs(Xm(1, :) - Xs(1, :));

    res.max_err_omega  = max(err_w);
    res.rmse_omega     = sqrt(mean(err_w.^2));
    res.max_err_i      = max(err_i);
    res.rel_err_omega  = res.max_err_omega / max(1e-9, max(abs(Xm(2, :))));
    res.Xm = Xm;
    res.Xs = Xs;
    res.U  = U_seq;

    fprintf('  Max blad predkosci  : %.3e rad/s\n', res.max_err_omega);
    fprintf('  RMSE predkosci      : %.3e rad/s\n', res.rmse_omega);
    fprintf('  Max blad pradu      : %.3e A\n',     res.max_err_i);
    fprintf('  Blad wzgledny       : %.3e\n',       res.rel_err_omega);

    if res.rel_err_omega < 1e-3
        fprintf('  WYNIK: modele zgodne - implementacja poprawna.\n');
    else
        fprintf(['  WYNIK: ROZBIEZNOSC. Sprawdz solver, krok calkowania ' ...
                 'i warunki poczatkowe integratorow.\n']);
    end

end
