function Tres = attack_campaign(p, Uopt, calib, n_runs)
%ATTACK_CAMPAIGN  Systematyczna ocena detektora dla roznych atakow FDI.
%
%  Dla kazdego scenariusza (typ ataku x amplituda) uruchamia n_runs
%  przebiegow Monte Carlo (rozne realizacje szumu) i liczy:
%
%    P_D        - prawdopodobienstwo wykrycia (detection rate)
%    delay      - sredni czas do wykrycia [s]
%    E_total    - zuzyta energia [J]       <- wplyw ataku na OPTYMALNOSC
%    w_final    - predkosc koncowa [rad/s]
%
%  To jest glowna tabela wynikowa do rozdzialu eksperymentalnego pracy:
%  pokazuje granice wykrywalnosci (attack stealthiness boundary) oraz
%  koszt ataku dla jakosci sterowania.

    if nargin < 4 || isempty(n_runs)
        n_runs = 30;
    end

    plant = plant_matlab(p);   % kampania: obiekt szybki (setki przebiegow)

    %% =========================================================
    %  SCENARIUSZE
    %  =========================================================

    scen = {};

    scen{end+1} = mk('none',    0,     'brak ataku (odniesienie)');

    % Zakres dobrany tak, by objac GRANICE SKRYTOSCI ataku:
    % walidacja numeryczna wskazuje przejscie 0% -> 100% w okolicy
    % 0.01-0.05 rad/s, czyli rzedu odchylenia szumu czujnika.
    for a = [0.005 0.01 0.02 0.05 0.1 0.2 0.5 1.0]
        scen{end+1} = mk('bias', a, sprintf('bias %.3f rad/s', a));
    end

    for s = [0.5 1.0 2.0 5.0]
        scen{end+1} = mk('ramp', s, sprintf('ramp %.1f rad/s^2', s));
    end

    for g = [1.02 1.05 1.20]
        scen{end+1} = mk('scaling', g, sprintf('scaling x%.2f', g));
    end

    for l = [5 10 20]
        scen{end+1} = mk('replay', l, sprintf('replay lag %d krokow', l));
    end

    %% =========================================================
    %  PRZEBIEGI
    %  =========================================================

    nS = numel(scen);

    name    = strings(nS, 1);
    type    = strings(nS, 1);
    magn    = zeros(nS, 1);
    P_D     = zeros(nS, 1);
    delay   = zeros(nS, 1);
    E_tot   = zeros(nS, 1);
    w_fin   = zeros(nS, 1);
    zmax    = zeros(nS, 1);

    fprintf('  Kampania: %d scenariuszy x %d przebiegow\n', nS, n_runs);

    for s = 1:nS

        a = scen{s}.attack;

        det_hits = 0;
        d_sum    = 0;
        d_cnt    = 0;
        E_sum    = 0;
        w_sum    = 0;
        z_sum    = 0;

        for m = 1:n_runs

            log_m = simulate_system(p, Uopt, plant, a, calib, 5000 + 97*s + m);

            if log_m.detected
                det_hits = det_hits + 1;
                if ~isnan(log_m.delay_detect)
                    d_sum = d_sum + log_m.delay_detect;
                    d_cnt = d_cnt + 1;
                end
            end

            E_sum = E_sum + log_m.E_total;
            w_sum = w_sum + log_m.w_final;
            z_valid = abs(log_m.z(~isnan(log_m.z)));
            if ~isempty(z_valid)
                z_sum = z_sum + max(z_valid);
            end

        end

        name(s)  = scen{s}.label;
        type(s)  = a.type;
        magn(s)  = scen{s}.mag;
        P_D(s)   = det_hits / n_runs;
        delay(s) = ternary(d_cnt > 0, d_sum / max(1, d_cnt), NaN);
        E_tot(s) = E_sum / n_runs;
        w_fin(s) = w_sum / n_runs;
        zmax(s)  = z_sum / n_runs;

        fprintf('    %-28s P_D=%4.0f%%  delay=%6s s  E=%7.2f J  w_end=%5.2f\n', ...
            scen{s}.label, 100*P_D(s), fmt(delay(s)), E_tot(s), w_fin(s));

    end

    Tres = table(name, type, magn, P_D, delay, E_tot, w_fin, zmax, ...
        'VariableNames', {'Scenariusz', 'Typ', 'Amplituda', ...
                          'P_D', 'Opoznienie_s', 'Energia_J', ...
                          'Omega_koncowa', 'z_max_sr'});

    %% =========================================================
    %  WYKRES PODSUMOWUJACY
    %  =========================================================

    plot_campaign(Tres);

    %% ---------------------------------------------------------
    function s = mk(tp, mag, label)
        a = p.attack;
        a.type = tp;
        switch lower(tp)
            case 'bias',    a.bias       = mag;
            case 'ramp',    a.slope      = mag;
            case 'scaling', a.gain       = mag;
            case 'replay',  a.replay_lag = mag;
        end
        s.attack = a;
        s.mag    = mag;
        s.label  = label;
    end

end

%% =============================================================
function plot_campaign(T)

    figure('Name', 'Kampania atakow FDI', 'NumberTitle', 'off', ...
           'Position', [120 80 950 620]);

    isBias = T.Typ == "bias";
    isRamp = T.Typ == "ramp";
    isScal = T.Typ == "scaling";
    isRepl = T.Typ == "replay";

    subplot(2,2,1);
    plot(T.Amplituda(isBias), 100*T.P_D(isBias), '-o', 'LineWidth', 1.6);
    xlabel('Amplituda biasu [rad/s]'); ylabel('P_D [%]');
    title('Wykrywalnosc ataku typu BIAS'); grid on; ylim([-5 105]);

    subplot(2,2,2);
    plot(T.Amplituda(isRamp), 100*T.P_D(isRamp), '-s', 'LineWidth', 1.6);
    hold on;
    yyaxis right;
    plot(T.Amplituda(isRamp), T.Opoznienie_s(isRamp), '--^', 'LineWidth', 1.4);
    ylabel('Opoznienie detekcji [s]');
    yyaxis left; ylabel('P_D [%]'); ylim([-5 105]);
    xlabel('Nachylenie [rad/s^2]');
    title('Atak RAMP: wykrywalnosc i opoznienie'); grid on;

    subplot(2,2,3);
    bar(categorical(T.Scenariusz(isScal | isRepl)), ...
        100*T.P_D(isScal | isRepl));
    ylabel('P_D [%]'); ylim([0 105]);
    title('SCALING vs REPLAY'); grid on;

    subplot(2,2,4);
    E0 = T.Energia_J(T.Typ == "none");
    bar(categorical(T.Scenariusz), T.Energia_J);
    hold on;
    if ~isempty(E0)
        yline(E0(1), '--r', 'bez ataku', 'LineWidth', 1.4);
    end
    ylabel('Energia [J]');
    title('Wplyw ataku na zuzycie energii (optymalnosc)');
    grid on;
    try, xtickangle(45); catch, end

end

%% =============================================================
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function s = fmt(x)
    if isnan(x), s = '  -  '; else, s = sprintf('%5.2f', x); end
end
