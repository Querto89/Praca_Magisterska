function [w_sim, I_sim, T_sim, E_sim, U_sim, ...
          t_sim, attack_detected] = run_simulink(p, Uopt)

    model_name = 'system_model';

    %% =========================================================
    %  CZAS WEJSCIA
    %  =========================================================

    t_input = (0:p.N)' * p.dt;

    %% =========================================================
    %  WYZNACZENIE TRAJEKTORII STEROWANIA
    %  =========================================================

    U_dp_values = zeros(p.N, 1);

    i_current = p.i_0;
    omega_current = p.omega_0;

    %% =========================================================
    %  MODEL DYSKRETNY
    %  =========================================================

    A = [ -p.Ra/p.La,   -p.Ke/p.La;
           p.Kt/p.J,    -p.B/p.J ];

    Bu = [1/p.La;
          0];

    Ad = expm(A*p.dt);

    Bd = A \ ((Ad - eye(2)) * Bu);

    %% =========================================================
    %  GENEROWANIE STEROWANIA
    %  =========================================================

    for k = 1:p.N

        [~, ii] = min(abs(p.i_grid - i_current));
        [~, iw] = min(abs(p.omega_grid - omega_current));

        U = Uopt(ii, iw, k);

        U = max(p.Umin, min(U, p.Umax));

        U_dp_values(k) = U;

        % Predykcja stanu dla kolejnej decyzji DP
        x_current = [i_current; omega_current];

        x_next = Ad*x_current + Bd*U;

        i_current = x_next(1);
        omega_current = x_next(2);

    end

    %% =========================================================
    %  DANE DO FROM WORKSPACE
    %  =========================================================

    % Ostatnia wartosc jest powielona tylko po to,
    % aby wektor wejściowy mial N+1 punktow czasu.

    U_dp_data = ...
        [t_input, [U_dp_values; U_dp_values(end)]];

    %% =========================================================
    %  URUCHOMIENIE SIMULINK
    %  =========================================================

    load_system(model_name);

    set_param(model_name, ...
        'SolverType', 'Fixed-step');

    set_param(model_name, ...
        'Solver', 'ode1');

    set_param(model_name, ...
        'FixedStep', num2str(p.dt));

    set_param(model_name, ...
        'StopTime', num2str(p.N*p.dt));

    %% =========================================================
    %  PRZEKAZANIE STEROWANIA DO SIMULINK
    %  =========================================================

    assignin('base', ...
        'U_dp_simulink', U_dp_data);

    %% =========================================================
    %  SYMULACJA
    %  =========================================================

    sim_out = sim(model_name, ...
        'ReturnWorkspaceOutputs', 'on');

    %% =========================================================
    %  ODCZYT SYGNALOW
    %  =========================================================

    t_sim = sim_out.tout;

    w_sim.dp = ...
        get_signal_data(sim_out.find('w_dp'));

    I_sim.dp = ...
        get_signal_data(sim_out.find('I_dp'));

    T_sim.dp = ...
        get_signal_data(sim_out.find('T_dp'));

    E_sim.dp = ...
        get_signal_data(sim_out.find('E_dp'));

    U_sim.dp = ...
        get_signal_data(sim_out.find('U_dp'));

    %% =========================================================
    %  FLAGA DETEKCJI
    %  =========================================================

    att_obj = ...
        sim_out.find('outAttack_Detected');

    if ~isempty(att_obj)

        attack_detected = ...
            get_signal_data(att_obj);

    else

        attack_detected = ...
            zeros(size(t_sim));

    end

end


%% =============================================================
%  FUNKCJA POMOCNICZA
%  =============================================================

function data = get_signal_data(obj)

    if isempty(obj)

        data = [];

    elseif isobject(obj) && ...
            isprop(obj, 'Values') && ...
            isobject(obj.Values) && ...
            isprop(obj.Values, 'Data')

        data = obj.Values.Data;

    elseif isobject(obj) && ...
            isprop(obj, 'Data')

        data = obj.Data;

    elseif isstruct(obj) && ...
            isfield(obj, 'signals')

        data = obj.signals.values;

    else

        data = obj;

    end

end