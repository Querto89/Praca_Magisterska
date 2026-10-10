function Uopt = dp_solver(p)

    %% =========================================================
    %  MODEL CIAGLY W PRZESTRZENI STANOW
    %  x = [i_a; omega]
    %  =========================================================

    A = [ -p.Ra/p.La,   -p.Ke/p.La;
           p.Kt/p.J,    -p.B/p.J ];

    Bu = [1/p.La;
          0];

    % Dokladna dyskretyzacja ZOH
    Ad = expm(A * p.dt);

    % Bd = integral_0^dt exp(A*t) dt * Bu
    Bd = A \ ((Ad - eye(2)) * Bu);

    %% =========================================================
    %  ROZMIARY SIATKI
    %  =========================================================

    Ni = length(p.i_grid);
    Nw = length(p.omega_grid);
    Nu = length(p.U_grid);

    %% =========================================================
    %  FUNKCJA WARTOSCI
    %  =========================================================

    V = inf(Ni, Nw, p.N + 1);

    % Optymalne sterowanie:
    % Uopt(i_index, omega_index, k)
    Uopt = zeros(Ni, Nw, p.N);

    %% =========================================================
    %  KOSZT KONCOWY
    %  =========================================================

    for ii = 1:Ni
        for iw = 1:Nw

            omega = p.omega_grid(iw);

            V(ii, iw, end) = ...
                p.alpha_terminal * ...
                (omega - p.omega_target)^2;

        end
    end

    %% =========================================================
    %  PROGRAMOWANIE DYNAMICZNE
    %  =========================================================

    for k = p.N:-1:1

        fprintf('DP: krok %d / %d\n', k, p.N);

        for ii = 1:Ni

            i_a = p.i_grid(ii);

            for iw = 1:Nw

                omega = p.omega_grid(iw);

                x = [i_a; omega];

                best_cost = inf;
                best_U = p.Umin;

                for iu = 1:Nu

                    U = p.U_grid(iu);

                    %% -----------------------------------------
                    %  Predykcja kolejnego stanu
                    %  -----------------------------------------

                    x_next = Ad*x + Bd*U;

                    i_next = x_next(1);
                    omega_next = x_next(2);

                    %% -----------------------------------------
                    %  Ograniczenia stanu
                    %  -----------------------------------------

                    if i_next < p.Imin || i_next > p.Imax
                        continue;
                    end

                    if omega_next < 0 || omega_next > p.omega_max
                        continue;
                    end

                    %% -----------------------------------------
                    %  Znalezienie najblizszego punktu siatki
                    %  -----------------------------------------

                    [~, ii_next] = ...
                        min(abs(p.i_grid - i_next));

                    [~, iw_next] = ...
                        min(abs(p.omega_grid - omega_next));

                    %% -----------------------------------------
                    %  Energia elektryczna
                    %  P_el = u * i
                    %  -----------------------------------------

                    P_el = U * i_a;

                    E_step = P_el * p.dt;

                    %% -----------------------------------------
                    %  Uchyb predkosci
                    %  -----------------------------------------

                    speed_error = ...
                        p.beta * ...
                        (omega - p.omega_target)^2 * p.dt;

                    %% -----------------------------------------
                    %  Funkcja kosztu
                    %  -----------------------------------------

                    stage_cost = ...
                        p.lambda_E * E_step + ...
                        speed_error;

                    total_cost = ...
                        stage_cost + ...
                        V(ii_next, iw_next, k+1);

                    %% -----------------------------------------
                    %  Aktualizacja optimum
                    %  -----------------------------------------

                    if total_cost < best_cost

                        best_cost = total_cost;
                        best_U = U;

                    end

                end

                V(ii, iw, k) = best_cost;
                Uopt(ii, iw, k) = best_U;

            end
        end
    end

end