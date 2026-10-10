%% EXPERIMENT_05_STATE_GRID
% Wplyw rozdzielczosci siatki stanow (i_a, omega) na DP.
% Uruchom po startup_project z katalogu projektu.
% Zmieniamy TYLKO siatki stanow; siatka napiec, wagi i model sa stale.
% Siatki sa zagniezdzone: 11x31 subset 21x61 subset 41x121.
% UWAGA: wariant gesty moze wymagac kilku minut i wiecej pamieci.

clearvars -except ans;
clc;
fprintf('\n========================================\n');
fprintf('EKSPERYMENT 05: WPLYW SIATKI STANOW\n');
fprintf('========================================\n');

p0 = params();
cfg.nI = [11 21 41];
cfg.nOmega = [31 61 121];
cfg.settlingBand = 0.02;
cfg.limitTolerance = 1e-3;
cfg.resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'results','experiment_05');
if ~exist(cfg.resultDir,'dir'), mkdir(cfg.resultDir); end

n = numel(cfg.nI);
results = repmat(struct('N_i',NaN,'N_omega',NaN,'N_states',NaN, ...
    'DeltaI_A',NaN,'DeltaOmega_rad_s',NaN,'J',NaN,'Energy_J',NaN, ...
    'IAE',NaN,'ISE',NaN,'Overshoot_pct',NaN,'SettlingTime_s',NaN, ...
    'MaxCurrent_A',NaN,'MaxVoltage_V',NaN,'MaxSpeed_rad_s',NaN, ...
    'CurrentLimitExceeded',false,'VoltageLimitExceeded',false, ...
    'SpeedLimitExceeded',false,'ComputationTime_s',NaN, ...
    'Status',"NOT_RUN"),n,1);
traces = cell(n,1);
policies = cell(n,1);

fprintf('Stala siatka U: %d punktow; stale wagi: lambda_E=%g, lambda_w=%g, lambda_N=%g\n', ...
    numel(p0.dp.U_grid),p0.dp.lambda_energy, ...
    p0.dp.lambda_speed,p0.dp.lambda_terminal);

for k=1:n
    p=p0;
    p.dp.i_grid=linspace(p0.limits.Imin,p0.limits.Imax,cfg.nI(k));
    p.dp.omega_grid=linspace(p0.limits.omega_min,p0.limits.omega_max,cfg.nOmega(k));
    results(k).N_i=cfg.nI(k);
    results(k).N_omega=cfg.nOmega(k);
    results(k).N_states=cfg.nI(k)*cfg.nOmega(k);
    results(k).DeltaI_A=p.dp.i_grid(2)-p.dp.i_grid(1);
    results(k).DeltaOmega_rad_s=p.dp.omega_grid(2)-p.dp.omega_grid(1);

    fprintf('\n----------------------------------------\n');
    fprintf('SIATKA %d/%d: %d x %d = %d stanow; Delta i=%.4f A; Delta omega=%.4f rad/s\n', ...
        k,n,cfg.nI(k),cfg.nOmega(k),results(k).N_states, ...
        results(k).DeltaI_A,results(k).DeltaOmega_rad_s);
    fprintf('----------------------------------------\n');

    tic;
    try
        dp=dp_solver(p);
        elapsed=toc;
        assert(isfield(dp,'u_opt') && numel(dp.u_opt)==p.sim.N, ...
            'dp_solver nie zwrocil poprawnej trajektorii u_opt.');
        policies{k}=dp.u_opt;
        fprintf('Symulacja obiektu Simulink...\n');
        raw=run_simulink(p,dp.u_opt);
        tr=normalizeSimulation(raw,p,dp.u_opt);
        traces{k}=tr;
        m=localMetrics(tr,p,cfg.settlingBand,cfg.limitTolerance);
        results(k).J=readObjective(dp);
        results(k).Energy_J=m.Energy_J;
        results(k).IAE=m.IAE;
        results(k).ISE=m.ISE;
        results(k).Overshoot_pct=m.Overshoot_pct;
        results(k).SettlingTime_s=m.SettlingTime_s;
        results(k).MaxCurrent_A=m.MaxCurrent_A;
        results(k).MaxVoltage_V=m.MaxVoltage_V;
        results(k).MaxSpeed_rad_s=m.MaxSpeed_rad_s;
        results(k).CurrentLimitExceeded=m.CurrentLimitExceeded;
        results(k).VoltageLimitExceeded=m.VoltageLimitExceeded;
        results(k).SpeedLimitExceeded=m.SpeedLimitExceeded;
        results(k).ComputationTime_s=elapsed;
        results(k).Status="OK";
        fprintf('J=%.6f | E=%.6f J | IAE=%.6f | ISE=%.6f | t_s=%.4f s\n', ...
            results(k).J,m.Energy_J,m.IAE,m.ISE,m.SettlingTime_s);
        fprintf('Imax=%.4f A | Umax=%.4f V | czas DP=%.2f s\n', ...
            m.MaxCurrent_A,m.MaxVoltage_V,elapsed);
        if m.CurrentLimitExceeded || m.VoltageLimitExceeded || m.SpeedLimitExceeded
            warning('Wariant %d: przekroczono co najmniej jedno ograniczenie!',k);
        end
    catch ME
        results(k).ComputationTime_s=toc;
        results(k).Status="ERROR: "+string(ME.message);
        warning('Wariant %d nie powiodl sie: %s',k,ME.message);
    end
    % Zachowaj wyniki czesciowe nawet w razie problemu kolejnego wariantu.
    T=struct2table(results);
    writetable(T,fullfile(cfg.resultDir,'state_grid_results.csv'));
    save(fullfile(cfg.resultDir,'state_grid_results.mat'), ...
        'T','results','traces','policies','cfg','p0','-v7.3');
end

T=struct2table(results);
fprintf('\n========================================\n');
fprintf('POROWNANIE SIATEK STANOW\n');
fprintf('========================================\n');
disp(T);

ok=find([results.Status]=="OK");
if ~isempty(ok)
    labels=arrayfun(@(j)sprintf('%d x %d (%d stanow)', ...
        cfg.nI(j),cfg.nOmega(j),results(j).N_states),ok, ...
        'UniformOutput',false);
    f1=figure('Name','Eksperyment 05 - przebiegi');
    tiledlayout(3,1,'TileSpacing','compact');
    nexttile; hold on; grid on;
    for j=ok, plot(traces{j}.t,traces{j}.omega,'LineWidth',1.3); end
    yline(p0.reference.omega,'k:','Predkosc zadana');
    yline(p0.limits.omega_max,'r:','Limit predkosci');
    ylabel('\omega [rad/s]'); title('Predkosc katowa');
    legend(labels,'Location','best');
    nexttile; hold on; grid on;
    for j=ok, plot(traces{j}.t,traces{j}.i,'LineWidth',1.3); end
    yline(p0.limits.Imax,'r:','Limit pradu');
    ylabel('i_a [A]'); title('Prad twornika');
    nexttile; hold on; grid on;
    for j=ok, stairs(traces{j}.t,traces{j}.u,'LineWidth',1.2); end
    yline(p0.limits.Umax,'r:','Limit napiecia');
    ylabel('u [V]'); xlabel('Czas [s]'); title('Napiecie sterujace (ZOH)');
    exportgraphics(f1,fullfile(cfg.resultDir,'state_grid_trajectories.png'),'Resolution',200);
    savefig(f1,fullfile(cfg.resultDir,'state_grid_trajectories.fig'));

    f2=figure('Name','Eksperyment 05 - wskazniki');
    tiledlayout(2,2,'TileSpacing','compact');
    x=[results(ok).N_states];
    nexttile; plot(x,[results(ok).J],'-o','LineWidth',1.3); grid on;
    xlabel('Liczba stanow'); ylabel('J'); title('Wartosc funkcji celu');
    nexttile; plot(x,[results(ok).Energy_J],'-o','LineWidth',1.3); grid on;
    xlabel('Liczba stanow'); ylabel('E [J]'); title('Zuzycie energii');
    nexttile; plot(x,[results(ok).IAE],'-o',x,[results(ok).ISE],'-s', ...
        'LineWidth',1.3); grid on;
    xlabel('Liczba stanow'); ylabel('Wskaznik'); legend('IAE','ISE','Location','best');
    title('Jakosc sledzenia');
    nexttile; plot(x,[results(ok).ComputationTime_s],'-o','LineWidth',1.3); grid on;
    xlabel('Liczba stanow'); ylabel('t [s]'); title('Czas obliczen DP');
    exportgraphics(f2,fullfile(cfg.resultDir,'state_grid_metrics.png'),'Resolution',200);
    savefig(f2,fullfile(cfg.resultDir,'state_grid_metrics.fig'));
end

fprintf('\nEXPERIMENT_05: zakonczony. Wyniki: %s\n',cfg.resultDir);

%% Funkcje lokalne
function s = normalizeSimulation(raw,p,uDP)
% Adapter dla typowych struktur zwracanych przez run_simulink.
% W razie innego interfejsu zmien tylko te nazwy pol.
    s.t = getVector(raw,{'t','time','tout','t_sim'});
    s.omega = getVector(raw,{'omega','omega_sim','w','speed'});
    s.i = getVector(raw,{'i','i_a','ia','current','i_sim'});
    u = getVector(raw,{'u','u_a','voltage','u_sim'},false);
    E = getVector(raw,{'E','energy','E_sim','energy_J'},false);

    assert(~isempty(s.t) && ~isempty(s.omega) && ~isempty(s.i), ...
        ['Nie rozpoznano pol wyniku run_simulink. Sprawdz ' ...
         'normalizeSimulation i dopasuj nazwy t, omega, i.']);
    s.t = s.t(:); s.omega = s.omega(:); s.i = s.i(:);
    assert(numel(s.t)==numel(s.omega) && numel(s.t)==numel(s.i), ...
        'Czas, predkosc i prad musza miec jednakowa liczbe probek.');
    [s.t,idx] = unique(s.t,'stable');
    s.omega=s.omega(idx); s.i=s.i(idx);
    if isempty(u)
        % Sterowanie DP jest stale na kazdym kroku p.sim.dt (ZOH).
        idxU = min(floor((s.t+1e-10)/p.sim.dt)+1,numel(uDP));
        s.u = uDP(idxU(:)); s.u=s.u(:);
    elseif isscalar(u)
        s.u = repmat(u,size(s.t));
    elseif numel(u)==numel(idx)
        u=u(:); s.u=u(idx);
    elseif numel(u)==numel(s.t)
        s.u=u(:);
    else
        error('Niezgodna dlugosc wektora napiecia w run_simulink.');
    end
    if isempty(E)
        s.E = cumtrapz(s.t,s.u.*s.i);
    elseif isscalar(E)
        s.E = E;
    elseif numel(E)==numel(idx)
        E=E(:); s.E=E(idx);
    elseif numel(E)==numel(s.t)
        s.E=E(:);
    else
        error('Niezgodna dlugosc wektora energii w run_simulink.');
    end
end

function v = getVector(s,names,required)
    if nargin<3, required=true; end
    v=[];
    if ~isstruct(s)
        error('run_simulink powinien zwrocic strukture. Dopasuj normalizeSimulation.');
    end
    for j=1:numel(names)
        if isfield(s,names{j})
            v=s.(names{j});
            if isa(v,'timeseries'), v=v.Data; end
            if isnumeric(v), v=double(v(:)); return; end
        end
    end
    if required
        fprintf('Dostepne pola run_simulink: %s\n',strjoin(fieldnames(s),', '));
    end
end

function J = readObjective(dp)
    J=NaN;
    for name={'J_opt','J','J_total','total_cost','cost_opt'}
        if isfield(dp,name{1})
            v=dp.(name{1});
            if isnumeric(v) && isscalar(v), J=double(v); return; end
        end
    end
    warning('Nie znaleziono skalarnej wartosci J w dp. Wpisz nazwe pola w readObjective.');
end

function m = localMetrics(s,p,band,tol)
    ref=p.reference.omega;
    e=ref-s.omega;
    m.Energy_J=s.E(end);
    m.IAE=trapz(s.t,abs(e));
    m.ISE=trapz(s.t,e.^2);
    m.Overshoot_pct=100*max(0,max(s.omega)-ref)/max(abs(ref),eps);
    outside=find(abs(e)>band*max(abs(ref),eps));
    if isempty(outside)
        m.SettlingTime_s=s.t(1);
    elseif outside(end)==numel(s.t)
        m.SettlingTime_s=NaN; % nie osiagnieto pasma przed koncem
    else
        m.SettlingTime_s=s.t(outside(end)+1);
    end
    m.MaxCurrent_A=max(s.i);
    m.MaxVoltage_V=max(s.u);
    m.MaxSpeed_rad_s=max(s.omega);
    m.CurrentLimitExceeded=any(s.i>p.limits.Imax+tol | s.i<p.limits.Imin-tol);
    m.VoltageLimitExceeded=any(s.u>p.limits.Umax+tol | s.u<p.limits.Umin-tol);
    m.SpeedLimitExceeded=any(s.omega>p.limits.omega_max+tol | ...
        s.omega<p.limits.omega_min-tol);
end
