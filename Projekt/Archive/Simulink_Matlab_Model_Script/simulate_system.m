function log = simulate_system(p, Uopt, plant, attack, calib, seed)
%SIMULATE_SYSTEM  Petla zamknieta: obiekt + czujniki + atak + DP + detektor.
%
%  PRZEPLYW W JEDNYM KROKU k:
%
%    obiekt (Simulink/MATLAB, parametry PRAWDZIWE)
%        |  x(k)
%        v
%    czujniki (szum) --> ATAK FDI --> y(k)
%        |                              |
%        |                              +--> Digital Twin --> residuum --> ALARM
%        v
%    regulator DP (tablica Uopt) --> u(k) --> obiekt
%
%  Regulator dziala na POMIARZE, wiec atak wplywa rowniez na jakosc
%  sterowania - to pozwala pokazac sprzezenie bezpieczenstwo <-> optymalnosc.

    if nargin < 4 || isempty(attack), attack = p.attack;   end
    if nargin < 5,                    calib  = [];         end
    if nargin < 6 || isempty(seed),   seed   = p.sensor.seed; end

    N = p.N;

    %% =========================================================
    %  INICJALIZACJA
    %  =========================================================

    x   = plant.init();
    mc  = measurement_channel(p, attack, seed);
    det = detector(p, calib);

    x_true = zeros(2, N + 1);
    y_meas = zeros(2, N + 1);
    U      = zeros(1, N);
    t      = (0:N) * p.dt;

    %% =========================================================
    %  PETLA GLOWNA
    %  =========================================================

    for k = 1:N + 1

        x_true(:, k) = x;

        % --- Pomiar (szum + ewentualny atak) ---
        [y, mc] = mc.apply(mc, k, t(k), x);
        y_meas(:, k) = y;

        % --- Detektor: residuum miedzy pomiarem a predykcja twina ---
        if k > 1
            det = det.update(det, k - 1, y_meas(:, k - 1), U(k - 1), y);
        end

        % --- Regulator DP (na podstawie POMIARU) ---
        if k <= N
            U(k) = dp_policy(p, Uopt, y, k);
            x = plant.step(x, U(k));
        end

    end

    %% =========================================================
    %  WSKAZNIKI JAKOSCI (na stanie PRAWDZIWYM)
    %  =========================================================

    i_true = x_true(1, :);
    w_true = x_true(2, :);

    % Energia elektryczna pobrana: E = sum u*i*dt
    P_el   = U .* i_true(1:N);
    E_true = [0, cumsum(P_el) * p.dt];

    % Calkowy uchyb predkosci
    track_cost = p.beta * sum((w_true - p.omega_target).^2) * p.dt;

    % Koszt wg kryterium DP
    J_total = p.lambda_E * E_true(end) + track_cost + ...
              p.alpha_terminal * (w_true(end) - p.omega_target)^2;

    %% =========================================================
    %  METRYKI DETEKCJI
    %  =========================================================

    idx_alarm = find(det.alarm > 0, 1, 'first');

    if isempty(idx_alarm)
        detected     = false;
        t_detect     = NaN;
        delay_detect = NaN;
    else
        detected = true;
        t_detect = t(idx_alarm + 1);            % residuum k dotyczy chwili k+1
        if strcmpi(attack.type, 'none')
            delay_detect = NaN;                 % to jest falszywy alarm
        else
            delay_detect = max(0, t_detect - attack.start);
        end
    end

    %% =========================================================
    %  LOG
    %  =========================================================

    log.t          = t;
    log.x_true     = x_true;
    log.w_true     = w_true;
    log.i_true     = i_true;
    log.y_meas     = y_meas;
    log.w_meas     = y_meas(2, :);
    log.i_meas     = y_meas(1, :);
    log.U          = U;
    log.T_em       = p.plant.Kt * i_true;       % moment elektromagnetyczny
    log.E          = E_true;

    log.r          = det.r;
    log.z          = det.z;
    log.alarm      = det.alarm;
    log.cusum      = det.cusum;

    log.detected     = detected;
    log.t_detect     = t_detect;
    log.delay_detect = delay_detect;

    log.track_cost = track_cost;
    log.J_total    = J_total;
    log.E_total    = E_true(end);
    log.w_final    = w_true(end);
    log.attack     = attack;
    log.plant_name = plant.name;

end

%% =============================================================
function U = dp_policy(p, Uopt, y, k)
%DP_POLICY  Odczyt polityki DP dla zmierzonego stanu.

    i_m = min(max(y(1), p.Imin), p.Imax);
    w_m = min(max(y(2), 0),      p.omega_max);

    [~, ii] = min(abs(p.i_grid     - i_m));
    [~, iw] = min(abs(p.omega_grid - w_m));

    U = Uopt(ii, iw, k);

    U = min(max(U, p.Umin), p.Umax);

end
