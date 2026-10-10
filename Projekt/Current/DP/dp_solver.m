function dp = dp_solver(p)
% DP_SOLVER
% Programowanie dynamiczne dla napedu pradu stalego.
%
% Stan:
%       x = [i_a; omega]
%
% Sterowanie:
%       u = u_a
%
% Funkcja celu:
%
%   J = sum_k [
%           lambda_energy * E_k
%         + lambda_speed * (omega_k - omega_ref)^2 * dt
%       ]
%       + lambda_terminal * (omega_N - omega_ref)^2
%
% gdzie energia elektryczna w kroku:
%
%   E_k = integral( u_k * i_a(t) dt )
%
% Przy zalozeniu ZOH napiecie u_k jest stale w pojedynczym
% kroku sterowania, natomiast prad i_a(t) zmienia sie zgodnie
% z dynamika silnika.
%
% Wyjscie:
%
%   dp.Uopt       - optymalna polityka sterowania na siatce
%   dp.V          - funkcja wartosci
%
%   dp.u_opt      - optymalne sterowanie dla stanu poczatkowego
%   dp.i_opt      - optymalna trajektoria pradu
%   dp.omega_opt  - optymalna trajektoria predkosci
%
%   dp.E_step_opt - energia w kolejnych krokach
%   dp.E_opt      - energia skumulowana
%
%   dp.t          - wektor czasu
%
%   dp.i_grid
%   dp.omega_grid
%   dp.U_grid


%% ============================================================
%  PARAMETRY SILNIKA
%  ============================================================

Ra = p.motor.Ra;
La = p.motor.La;

Kt = p.motor.Kt;
Ke = p.motor.Ke;

J  = p.motor.J;
B  = p.motor.B;

TL = p.motor.TL;


%% ============================================================
%  PARAMETRY SYMULACJI
%  ============================================================

dt = p.sim.dt;
N  = p.sim.N;

energy_substeps = p.sim.energy_substeps;


%% ============================================================
%  WARTOSC ZADANA
%  ============================================================

omega_ref = p.reference.omega;


%% ============================================================
%  WAGI FUNKCJI CELU
%  ============================================================

lambda_E = p.dp.lambda_energy;
lambda_w = p.dp.lambda_speed;
lambda_N = p.dp.lambda_terminal;


%% ============================================================
%  SIATKI PROGRAMOWANIA DYNAMICZNEGO
%  ============================================================

i_grid     = p.dp.i_grid;
omega_grid = p.dp.omega_grid;
U_grid     = p.dp.U_grid;

Ni = length(i_grid);
Nw = length(omega_grid);
Nu = length(U_grid);


%% ============================================================
%  MODEL CIAGLY SILNIKA DC
%
%  x = [i_a; omega]
%
%  dx/dt = A*x + Bu*u + BT*TL
%  ============================================================

A = [ -Ra/La,  -Ke/La;
       Kt/J,    -B/J ];

Bu = [1/La;
      0];

BT = [0;
     -1/J];


%% ============================================================
%  DOKLADNA DYSKRETYZACJA ZOH
%
%  x(k+1) =
%
%       Ad*x(k)
%       + Bdu*u(k)
%       + BdT*TL
%  ============================================================

B_all = [Bu BT];

M = [A, B_all;
     zeros(2,4)];

Md = expm(M * dt);

Ad  = Md(1:2,1:2);
Bdu = Md(1:2,3);
BdT = Md(1:2,4);


%% ============================================================
%  MACIERZE DLA PODKROKOW ENERGII
%
%  Podkroki sa wykorzystywane do:
%
%  1. wyznaczania przebiegu i_a(t),
%  2. calkowania energii elektrycznej,
%  3. sprawdzania ograniczenia pradu wewnatrz calego kroku.
%  ============================================================

tau = linspace(0, dt, energy_substeps + 1);

Ad_tau  = zeros(2,2,energy_substeps + 1);
Bdu_tau = zeros(2,energy_substeps + 1);
BdT_tau = zeros(2,energy_substeps + 1);

for m = 1:(energy_substeps + 1)

    M_tau = expm(M * tau(m));

    Ad_tau(:,:,m) = M_tau(1:2,1:2);

    Bdu_tau(:,m) = M_tau(1:2,3);

    BdT_tau(:,m) = M_tau(1:2,4);

end


%% ============================================================
%  INICJALIZACJA FUNKCJI WARTOSCI
%
%  V(ii,iw,k)
%
%  minimalny skumulowany koszt od kroku k do konca.
%
%
%  Uopt(ii,iw,k)
%
%  optymalna decyzja sterujaca dla wezla siatki:
%
%       i_a     = i_grid(ii)
%       omega   = omega_grid(iw)
%  ============================================================

V = inf(Ni, Nw, N+1);

Uopt = NaN(Ni, Nw, N);


%% ============================================================
%  KOSZT KONCOWY
%
%  Phi(x_N) =
%
%       lambda_terminal *
%       (omega_N - omega_ref)^2
%  ============================================================

for ii = 1:Ni

    for iw = 1:Nw

        omega = omega_grid(iw);

        V(ii,iw,N+1) = ...
            lambda_N * ...
            (omega - omega_ref)^2;

    end

end


%% ============================================================
%  PROGRAMOWANIE DYNAMICZNE WSTECZ
%  ============================================================

fprintf('\n');
fprintf('========================================\n');
fprintf('PROGRAMOWANIE DYNAMICZNE\n');
fprintf('========================================\n');

fprintf('Liczba stanow:       %d\n', Ni*Nw);
fprintf('Liczba sterowan:     %d\n', Nu);
fprintf('Liczba krokow:       %d\n', N);
fprintf('Podkroki energii:    %d\n\n', energy_substeps);


for k = N:-1:1

    fprintf('DP: krok %d / %d\n', k, N);

    V_next = V(:,:,k+1);


    %% --------------------------------------------------------
    %  PRZEJSCIE PO SIATCE STANOW
    %  --------------------------------------------------------

    for ii = 1:Ni

        i_a = i_grid(ii);


        for iw = 1:Nw

            omega = omega_grid(iw);

            x = [i_a;
                 omega];

            best_cost = inf;
            best_U    = NaN;


            %% ------------------------------------------------
            %  SPRAWDZENIE WSZYSTKICH DECYZJI
            %  ------------------------------------------------

            for iu = 1:Nu

                U = U_grid(iu);


                %% ============================================
                %  STAN NA KONCU KROKU
                %  ============================================

                x_next = ...
                    Ad*x ...
                    + Bdu*U ...
                    + BdT*TL;

                i_next     = x_next(1);
                omega_next = x_next(2);


                %% ============================================
                %  OGRANICZENIA NA KONCU KROKU
                %  ============================================

                if i_next < p.limits.Imin || ...
                   i_next > p.limits.Imax

                    continue;

                end


                if omega_next < p.limits.omega_min || ...
                   omega_next > p.limits.omega_max

                    continue;

                end


                %% ============================================
                %  PRZEBIEG PRADU WEWNATRZ KROKU
                %  ============================================

                i_internal = zeros( ...
                    energy_substeps + 1, 1);


                for m = 1:(energy_substeps + 1)

                    x_tau = ...
                        Ad_tau(:,:,m)*x ...
                        + Bdu_tau(:,m)*U ...
                        + BdT_tau(:,m)*TL;

                    i_internal(m) = x_tau(1);

                end


                %% ============================================
                %  OGRANICZENIE PRADU W CALYM KROKU
                %  ============================================

                if any(i_internal < p.limits.Imin) || ...
                   any(i_internal > p.limits.Imax)

                    continue;

                end


                %% ============================================
                %  ENERGIA ELEKTRYCZNA
                %
                %  E_k =
                %
                %       integral u_k*i_a(t) dt
                %
                %  Poniewaz u_k = const:
                %
                %  E_k =
                %
                %       u_k * integral i_a(t) dt
                %  ============================================

                E_step = ...
                    U * trapz(tau, i_internal);


                %% ============================================
                %  KOSZT BIEZACY
                %  ============================================

                speed_error = ...
                    omega - omega_ref;


                stage_cost = ...
                    lambda_E * E_step ...
                    + lambda_w * ...
                    speed_error^2 * dt;


                %% ============================================
                %  KOSZT PRZYSZLY
                %
                %  Stan x_next zwykle nie znajduje sie
                %  dokladnie w wezle siatki.
                %
                %  Dlatego V(k+1) jest wyznaczane za pomoca
                %  interpolacji dwuliniowej.
                %  ============================================

                future_cost = interp2( ...
                    omega_grid, ...
                    i_grid, ...
                    V_next, ...
                    omega_next, ...
                    i_next, ...
                    'linear');


                if isnan(future_cost) || ...
                   isinf(future_cost)

                    continue;

                end


                %% ============================================
                %  ROWNANIE BELLMANA
                %
                %  V_k(x_k) =
                %
                %    min_u [
                %
                %       L(x_k,u_k)
                %       + V_(k+1)(x_(k+1))
                %
                %    ]
                %  ============================================

                total_cost = ...
                    stage_cost ...
                    + future_cost;


                %% ============================================
                %  WYBOR NAJLEPSZEGO STEROWANIA
                %  ============================================

                if total_cost < best_cost

                    best_cost = total_cost;

                    best_U = U;

                end

            end


            %% ================================================
            %  ZAPIS FUNKCJI WARTOSCI I POLITYKI
            %  ================================================

            V(ii,iw,k) = best_cost;

            Uopt(ii,iw,k) = best_U;

        end

    end

end


%% ============================================================
%  KONTROLA POPRAWNOSCI POLITYKI
%  ============================================================

number_nan_U = sum(isnan(Uopt(:)));

number_inf_V = sum(isinf(V(:)));


if number_nan_U > 0

    warning( ...
        ['DP: znaleziono %d stanow bez ', ...
         'dopuszczalnego sterowania.'], ...
        number_nan_U);

end


if number_inf_V > 0

    warning( ...
        ['DP: znaleziono %d nieskonczonych ', ...
         'wartosci funkcji kosztu.'], ...
        number_inf_V);

end


%% ============================================================
%  ODTWORZENIE OPTYMALNEJ TRAJEKTORII
%
%  Start:
%
%       x_0 = [i_a(0); omega(0)]
%
%
%  WAZNE:
%
%  Nie interpolujemy polityki Uopt.
%
%  Dla rzeczywistego stanu x(k) ponownie sprawdzamy wszystkie
%  wartosci nalezace do U_grid.
%
%  Wykonane sterowanie pozostaje zatem elementem dyskretnego
%  zbioru decyzji wykorzystanego podczas optymalizacji.
%  ============================================================

x_opt = zeros(2,N+1);

u_opt = zeros(N,1);

E_step_opt = zeros(N,1);


x_opt(:,1) = ...
    [p.initial.i_a;
     p.initial.omega];


for k = 1:N

    x = x_opt(:,k);

    omega = x(2);

    best_cost = inf;

    best_U = NaN;

    best_E = NaN;

    best_x_next = [NaN;
                   NaN];


    %% --------------------------------------------------------
    %  SPRAWDZENIE WSZYSTKICH DOPUSZCZALNYCH DECYZJI
    %  --------------------------------------------------------

    for iu = 1:Nu

        U = U_grid(iu);


        %% ====================================================
        %  STAN NA KONCU KROKU
        %  ====================================================

        x_next = ...
            Ad*x ...
            + Bdu*U ...
            + BdT*TL;

        i_next     = x_next(1);
        omega_next = x_next(2);


        %% ====================================================
        %  OGRANICZENIA STANU
        %  ====================================================

        if i_next < p.limits.Imin || ...
           i_next > p.limits.Imax

            continue;

        end


        if omega_next < p.limits.omega_min || ...
           omega_next > p.limits.omega_max

            continue;

        end


        %% ====================================================
        %  PRZEBIEG PRADU WEWNATRZ KROKU
        %  ====================================================

        i_internal = zeros( ...
            energy_substeps + 1, 1);


        for m = 1:(energy_substeps + 1)

            x_tau = ...
                Ad_tau(:,:,m)*x ...
                + Bdu_tau(:,m)*U ...
                + BdT_tau(:,m)*TL;

            i_internal(m) = x_tau(1);

        end


        %% ====================================================
        %  OGRANICZENIE PRADU W CALYM KROKU
        %  ====================================================

        if any(i_internal < p.limits.Imin) || ...
           any(i_internal > p.limits.Imax)

            continue;

        end


        %% ====================================================
        %  ENERGIA KROKU
        %  ====================================================

        E_step = ...
            U * trapz(tau, i_internal);


        %% ====================================================
        %  KOSZT BIEZACY
        %  ====================================================

        speed_error = ...
            omega - omega_ref;


        stage_cost = ...
            lambda_E * E_step ...
            + lambda_w * ...
            speed_error^2 * dt;


        %% ====================================================
        %  KOSZT PRZYSZLY
        %  ====================================================

        future_cost = interp2( ...
            omega_grid, ...
            i_grid, ...
            V(:,:,k+1), ...
            omega_next, ...
            i_next, ...
            'linear');


        if isnan(future_cost) || ...
           isinf(future_cost)

            continue;

        end


        %% ====================================================
        %  ROWNANIE BELLMANA
        %  ====================================================

        total_cost = ...
            stage_cost ...
            + future_cost;


        %% ====================================================
        %  WYBOR NAJLEPSZEJ DECYZJI
        %  ====================================================

        if total_cost < best_cost

            best_cost = total_cost;

            best_U = U;

            best_E = E_step;

            best_x_next = x_next;

        end

    end


    %% ========================================================
    %  KONTROLA OSIAGALNOSCI
    %  ========================================================

    if isnan(best_U)

        error( ...
            ['DP: brak dopuszczalnego sterowania ', ...
             'podczas odtwarzania trajektorii ', ...
             'w kroku %d.'], ...
            k);

    end


    %% ========================================================
    %  ZAPIS OPTYMALNEGO KROKU
    %  ========================================================

    u_opt(k) = best_U;

    E_step_opt(k) = best_E;

    x_opt(:,k+1) = best_x_next;

end


%% ============================================================
%  STRUKTURA WYNIKOWA
%  ============================================================

% Pelna polityka DP
dp.Uopt = Uopt;

% Funkcja wartosci
dp.V = V;


% Siatki DP
dp.i_grid = i_grid;

dp.omega_grid = omega_grid;

dp.U_grid = U_grid;


% Model dyskretny
dp.Ad = Ad;

dp.Bdu = Bdu;

dp.BdT = BdT;


% Parametry obliczania energii
dp.energy_substeps = energy_substeps;


% Optymalna trajektoria dla zadanego x0
dp.u_opt = u_opt;

dp.i_opt = x_opt(1,:)';

dp.omega_opt = x_opt(2,:)';


% Energia
dp.E_step_opt = E_step_opt;

dp.E_opt = [0;
            cumsum(E_step_opt)];

% Wartosc funkcji celu dla odtworzonej trajektorii
J_stage = 0;

for k = 1:N

    speed_error = ...
        dp.omega_opt(k) - omega_ref;

    J_stage = J_stage ...
        + lambda_E * dp.E_step_opt(k) ...
        + lambda_w * speed_error^2 * dt;

end

J_terminal = ...
    lambda_N * ...
    (dp.omega_opt(end) - omega_ref)^2;

dp.J_opt = J_stage + J_terminal;

dp.J_stage = J_stage;
dp.J_terminal = J_terminal;

% Czas
dp.t = (0:N)' * dt;


%% ============================================================
%  INFORMACJE KONCOWE
%  ============================================================

fprintf('\n');
fprintf('Programowanie dynamiczne zakonczone.\n');

fprintf( ...
    'Rozmiar polityki Uopt: %d x %d x %d\n', ...
    size(Uopt,1), ...
    size(Uopt,2), ...
    size(Uopt,3));

fprintf( ...
    'Stany bez sterowania:  %d\n', ...
    number_nan_U);

fprintf( ...
    'Wartosci Inf w V:      %d\n', ...
    number_inf_V);

fprintf( ...
    'Energia trajektorii DP: %.6f J\n', ...
    dp.E_opt(end));

fprintf( ...
    'Predkosc koncowa DP:    %.6f rad/s\n', ...
    dp.omega_opt(end));

fprintf( ...
    'Maksymalny prad DP:     %.6f A\n', ...
    max(dp.i_opt));

fprintf( ...
    'Minimalne napiecie DP:  %.6f V\n', ...
    min(dp.u_opt));

fprintf( ...
    'Maksymalne napiecie DP: %.6f V\n', ...
    max(dp.u_opt));

fprintf( ...
    'Wartosc funkcji celu J: %.6f\n', ...
    dp.J_opt);

fprintf( ...
    '  koszt etapowy:        %.6f\n', ...
    dp.J_stage);

fprintf( ...
    '  koszt terminalny:      %.6f\n', ...
    dp.J_terminal);
end