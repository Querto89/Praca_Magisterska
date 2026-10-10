function plant = plant_simulink(p)
%PLANT_SIMULINK  Obiekt "rzeczywisty" symulowany w Simulinku (system_model.slx).
%
%  ARCHITEKTURA
%  ------------
%  Model .slx czyta parametry ze struktury 'p' w base workspace
%  (bloki maja wpisane p.Ra, p.J, p.Kt, ...). Zeby obiekt mial parametry
%  PRAWDZIWE, a nie nominalne, podmieniamy 'p' w base workspace na czas
%  symulacji i przywracamy oryginal po zakonczeniu (onCleanup).
%
%  Dzieki temu NIE trzeba modyfikowac schematu blokowego .slx.
%
%  STEROWANIE W PETLI ZAMKNIETEJ
%  -----------------------------
%  Model jest uruchamiany krok po kroku (po jednym okresie dt), z warunkami
%  poczatkowymi integratorow ustawianymi na biezacy stan. Pozwala to
%  regulatorowi DP reagowac na POMIAR (ktory moze byc sfalszowany przez
%  atak) - bez tego atak nie wplywalby na sterowanie i nie dalo sie badac
%  sprzezenia bezpieczenstwo <-> optymalnosc.
%
%  NAPRAWIONY BLAD
%  ---------------
%  Oryginalnie: Solver='ode1', FixedStep=p.dt=0.05 s. Wartosci wlasne
%  ukladu to -20+-20j, wiec |1 + lambda*dt| = 1 -> jawny Euler zamienial
%  stabilny silnik w niegasnacy oscylator numeryczny. Tu: ode4 z krokiem
%  p.plant.h = dt/50.

    model = p.model_name;

    if ~bdIsLoaded(model)
        load_system(model);
    end

    %% =========================================================
    %  PODMIANA PARAMETROW W BASE WORKSPACE
    %  =========================================================

    had_p = evalin('base', 'exist(''p'', ''var'')') == 1;
    if had_p
        p_backup = evalin('base', 'p');
    else
        p_backup = [];
    end

    q          = p.plant;          % Ra, La, Kt, Ke, J, B, TL
    q.dt       = p.dt;             % FromWorkspace SampleTime
    q.Umin     = p.Umin;           % Saturation
    q.Umax     = p.Umax;

    assignin('base', 'p', q);

    cleanupObj = onCleanup(@() restore_base(had_p, p_backup));

    %% =========================================================
    %  KONFIGURACJA SOLVERA
    %  =========================================================

    set_param(model, 'SolverType', 'Fixed-step');
    set_param(model, 'Solver',     p.plant.solver);
    set_param(model, 'FixedStep',  num2str(p.plant.h, '%.12g'));

    % FromWorkspace: trzymaj ostatnia wartosc, bez interpolacji (ZOH)
    fw = [model '/From' newline 'Workspace'];
    try
        set_param(fw, 'Interpolate', 'off');
        set_param(fw, 'OutputAfterFinalValue', 'Holding final value');
    catch
        warning('plant_simulink:FromWorkspace', ...
            'Nie udalo sie ustawic bloku From Workspace - sprawdz nazwe bloku.');
    end

    %% =========================================================
    %  INTERFEJS OBIEKTU
    %  =========================================================

    k_step = 0;     % licznik krokow (do globalnej osi czasu)

    plant.name  = 'Simulink (system_model.slx)';
    plant.init  = @init;
    plant.step  = @step;
    plant.close = @close_plant;

    function x0 = init()
        k_step = 0;
        x0 = [p.i_0; p.omega_0];
    end

    function x_next = step(x, U)

        t0 = k_step * p.dt;
        t1 = t0 + p.dt;

        % Warunki poczatkowe integratorow = biezacy stan obiektu
        set_param([model '/Integrator2'], ...
            'InitialCondition', num2str(x(1), '%.16g'));   % i_a
        set_param([model '/Integrator1'], ...
            'InitialCondition', num2str(x(2), '%.16g'));   % omega

        % Sterowanie stale na przedziale [t0, t1]
        assignin('base', 'U_dp_simulink', [t0, U; t1, U]);

        set_param(model, 'StartTime', num2str(t0, '%.12g'));
        set_param(model, 'StopTime',  num2str(t1, '%.12g'));

        sim_out = sim(model, 'ReturnWorkspaceOutputs', 'on');

        w = get_signal_data(sim_out, 'w_dp');
        i = get_signal_data(sim_out, 'I_dp');

        if isempty(w) || isempty(i)
            error('plant_simulink:noSignals', ...
                ['Brak sygnalow w_dp / I_dp z modelu. Sprawdz bloki ' ...
                 'To Workspace (format Timeseries).']);
        end

        x_next = [i(end); w(end)];

        k_step = k_step + 1;

    end

    function close_plant()
        clear cleanupObj;   % przywraca base workspace
    end

end

%% =============================================================
function restore_base(had_p, p_backup)
    if had_p
        assignin('base', 'p', p_backup);
    else
        evalin('base', 'clear p');
    end
end
