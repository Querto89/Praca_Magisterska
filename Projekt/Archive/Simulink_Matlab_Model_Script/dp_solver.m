function [Uopt, V] = dp_solver(p)
%DP_SOLVER  Programowanie dynamiczne dla silnika DC (model NOMINALNY).
%
%  Wyznacza polityke sterowania Uopt(i_index, omega_index, k) minimalizujaca
%
%     J = sum_k [ lambda_E * u_k * i_k * dt  +  beta * (omega_k - omega_ref)^2 * dt ]
%         + alpha_terminal * (omega_N - omega_ref)^2
%
%  ZMIANY WZGLEDEM WERSJI 1:
%   1. Wektoryzacja po siatce stanu - petla tylko po k i po U
%      (ok. 2 rzedy wielkosci szybciej).
%   2. Interpolacja BILINIOWA funkcji wartosci zamiast "najblizszego wezla".
%      Zaokraglanie do wezla wprowadzalo sztuczny blad kwantyzacji,
%      ktory podnosil residuum Digital Twina i psul detekcje.
%   3. Jawny zwrot funkcji wartosci V (przydatne do analizy w pracy).

    [Ad, Bd] = motor_model(p.nom, p.dt);

    Ni = numel(p.i_grid);
    Nw = numel(p.omega_grid);
    Nu = numel(p.U_grid);

    % Siatka stanu: wiersze = prad, kolumny = predkosc
    [Igrid, Wgrid] = ndgrid(p.i_grid, p.omega_grid);

    V    = inf(Ni, Nw, p.N + 1);
    Uopt = zeros(Ni, Nw, p.N);

    %% =========================================================
    %  KOSZT KONCOWY
    %  =========================================================

    V(:, :, end) = p.alpha_terminal * (Wgrid - p.omega_target).^2;

    %% =========================================================
    %  REKURSJA WSTECZ
    %  =========================================================

    for k = p.N:-1:1

        if mod(k, 10) == 0 || k == p.N
            fprintf('  DP: krok %3d / %d\n', k, p.N);
        end

        V_next = V(:, :, k + 1);

        best_cost = inf(Ni, Nw);
        best_U    = repmat(p.Umin, Ni, Nw);

        for iu = 1:Nu

            U = p.U_grid(iu);

            %% Predykcja stanu (wektorowo dla calej siatki)
            i_next = Ad(1,1)*Igrid + Ad(1,2)*Wgrid + Bd(1)*U;
            w_next = Ad(2,1)*Igrid + Ad(2,2)*Wgrid + Bd(2)*U;

            %% Ograniczenia stanu
            feasible = (i_next >= p.Imin) & (i_next <= p.Imax) & ...
                       (w_next >= 0)      & (w_next <= p.omega_max);

            %% Koszt-do-konca w nastepnym kroku
            if p.dp_interp
                iq = min(max(i_next, p.Imin), p.Imax);
                wq = min(max(w_next, 0),      p.omega_max);
                V_go = interp2(p.omega_grid, p.i_grid, V_next, wq, iq, 'linear');
            else
                % Wariant "najblizszy wezel" (jak w wersji 1) - do porownania
                [~, ii_n] = min(abs(p.i_grid(:)     - i_next(:)'), [], 1);
                [~, iw_n] = min(abs(p.omega_grid(:) - w_next(:)'), [], 1);
                V_go = reshape(V_next(sub2ind([Ni Nw], ii_n, iw_n)), Ni, Nw);
            end

            %% Koszt etapowy
            E_step      = p.lambda_E * U .* Igrid * p.dt;     % energia elektryczna
            speed_error = p.beta * (Wgrid - p.omega_target).^2 * p.dt;

            total_cost = E_step + speed_error + V_go;

            total_cost(~feasible)      = inf;
            total_cost(isnan(V_go))    = inf;

            %% Aktualizacja optimum
            improved            = total_cost < best_cost;
            best_cost(improved) = total_cost(improved);
            best_U(improved)    = U;

        end

        V(:, :, k)    = best_cost;
        Uopt(:, :, k) = best_U;

    end

    n_bad = sum(isinf(V(:, :, 1)), 'all');
    if n_bad > 0
        fprintf(['  UWAGA: %d stanow poczatkowych bez dopuszczalnego ' ...
                 'sterowania (siatka/ograniczenia).\n'], n_bad);
    end

end
