function [t_dt, omega_ref, i_ref, T_ref, U_ref, E_ref] = ...
    run_matlab(p, Uopt)

    %% =========================================================
    %  CZAS
    %  =========================================================

    t_dt = (0:p.N)' * p.dt;

    %% =========================================================
    %  INICJALIZACJA
    %  =========================================================

    omega_ref = zeros(p.N + 1, 1);
    i_ref     = zeros(p.N + 1, 1);

    T_ref = zeros(p.N + 1, 1);
    U_ref = zeros(p.N, 1);

    E_ref = zeros(p.N + 1, 1);

    omega_ref(1) = p.omega_0;
    i_ref(1)     = p.i_0;

    %% =========================================================
    %  MODEL W PRZESTRZENI STANOW
    %  =========================================================

    A = [ -p.Ra/p.La,   -p.Ke/p.La;
           p.Kt/p.J,    -p.B/p.J ];

    Bu = [1/p.La;
          0];

    Bd = A \ ((expm(A*p.dt) - eye(2)) * Bu);

    Ad = expm(A*p.dt);

    %% =========================================================
    %  SYMULACJA DIGITAL TWIN
    %  =========================================================

    for k = 1:p.N

        %% -----------------------------------------
        %  Aktualny stan
        %  -----------------------------------------

        i_now = i_ref(k);
        omega_now = omega_ref(k);

        %% -----------------------------------------
        %  Wyszukanie najblizszego punktu siatki DP
        %  -----------------------------------------

        [~, ii] = min(abs(p.i_grid - i_now));
        [~, iw] = min(abs(p.omega_grid - omega_now));

        %% -----------------------------------------
        %  Optymalne napiecie
        %  -----------------------------------------

        U = Uopt(ii, iw, k);

        U = max(p.Umin, min(U, p.Umax));

        U_ref(k) = U;

        %% -----------------------------------------
        %  Aktualizacja modelu
        %  -----------------------------------------

        x_now = [i_now; omega_now];

        x_next = Ad*x_now + Bd*U;

        i_ref(k+1) = x_next(1);
        omega_ref(k+1) = x_next(2);

        %% -----------------------------------------
        %  Moment elektromagnetyczny
        %  -----------------------------------------

        T_ref(k) = p.Kt * i_ref(k);

        %% -----------------------------------------
        %  Energia elektryczna
        %  -----------------------------------------

        P_el = U * i_now;

        E_ref(k+1) = ...
            E_ref(k) + P_el * p.dt;

    end

    T_ref(end) = p.Kt * i_ref(end);

end