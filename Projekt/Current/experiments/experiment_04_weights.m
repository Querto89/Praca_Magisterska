%% EXPERIMENT_04_WEIGHTS
% Wplyw wag funkcji celu DP na energie i jakosc sledzenia.
% Uruchom z katalogu projektu po wykonaniu startup_project.
% Wymagane: params.m, dp_solver.m, run_simulink.m.
% Kazda konfiguracja ma ten sam model, ograniczenia, siatki i horyzont.
% Zmieniana jest tylko lambda_speed; lambda_energy i lambda_terminal sa stale.

clearvars -except ans;
clc;

fprintf('\n========================================\n');
fprintf('EKSPERYMENT 04: WPLYW WAG FUNKCJI CELU\n');
fprintf('========================================\n');

p0 = params();

% Zestaw wag: scenariusz bazowy (5) jest punktem odniesienia.
cfg.lambdaSpeed = [1 5 10];
cfg.lambdaEnergy = 1;
cfg.lambdaTerminal = p0.dp.lambda_terminal;
cfg.referenceIndex = find(cfg.lambdaSpeed == 5,1);
cfg.settlingBand = 0.02;
cfg.limitTolerance = 1e-3;
cfg.resultDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'results','experiment_04');

if ~exist(cfg.resultDir,'dir')
    mkdir(cfg.resultDir);
end

n = numel(cfg.lambdaSpeed);
results = repmat(struct('lambda_speed',NaN,'lambda_energy',NaN, ...
    'lambda_terminal',NaN,'J',NaN,'Energy_J',NaN,'IAE',NaN,'ISE',NaN, ...
    'Overshoot_pct',NaN,'SettlingTime_s',NaN,'MaxCurrent_A',NaN, ...
    'MaxVoltage_V',NaN,'MaxSpeed_rad_s',NaN, ...
    'CurrentLimitExceeded',false,'VoltageLimitExceeded',false, ...
    'SpeedLimitExceeded',false,'ComputationTime_s',NaN),n,1);
traces = cell(n,1);
policies = cell(n,1);

for k = 1:n
    p = p0;
    p.dp.lambda_speed = cfg.lambdaSpeed(k);
    p.dp.lambda_energy = cfg.lambdaEnergy;
    p.dp.lambda_terminal = cfg.lambdaTerminal;

    fprintf('\n----------------------------------------\n');
    fprintf('WARIANT %d/%d: lambda_speed=%.4g, lambda_energy=%.4g, lambda_terminal=%.4g\n', ...
        k,n,p.dp.lambda_speed,p.dp.lambda_energy,p.dp.lambda_terminal);
    fprintf('----------------------------------------\n');

    tic;
    dp = dp_solver(p);
    elapsed = toc;
    assert(isfield(dp,'u_opt'),'dp_solver nie zwrocil pola u_opt.');
    policies{k} = dp.u_opt;

    fprintf('Symulacja obiektu Simulink...\n');
    simResult = run_simulink(p,dp.u_opt);
    trace = normalizeSimulation(simResult,p,dp.u_opt);
    traces{k} = trace;
    m = localMetrics(trace,p,cfg.settlingBand,cfg.limitTolerance);

    results(k).lambda_speed = p.dp.lambda_speed;
    results(k).lambda_energy = p.dp.lambda_energy;
    results(k).lambda_terminal = p.dp.lambda_terminal;
    results(k).J = readObjective(dp);
    results(k).Energy_J = m.Energy_J;
    results(k).IAE = m.IAE;
    results(k).ISE = m.ISE;
    results(k).Overshoot_pct = m.Overshoot_pct;
    results(k).SettlingTime_s = m.SettlingTime_s;
    results(k).MaxCurrent_A = m.MaxCurrent_A;
    results(k).MaxVoltage_V = m.MaxVoltage_V;
    results(k).MaxSpeed_rad_s = m.MaxSpeed_rad_s;
    results(k).CurrentLimitExceeded = m.CurrentLimitExceeded;
    results(k).VoltageLimitExceeded = m.VoltageLimitExceeded;
    results(k).SpeedLimitExceeded = m.SpeedLimitExceeded;
    results(k).ComputationTime_s = elapsed;

    fprintf('J: %.6f | E: %.6f J | IAE: %.6f | ISE: %.6f\n', ...
        results(k).J,m.Energy_J,m.IAE,m.ISE);
    fprintf('ts(2%%): %.4f s | Imax: %.4f A | Umax: %.4f V | czas DP: %.2f s\n', ...
        m.SettlingTime_s,m.MaxCurrent_A,m.MaxVoltage_V,elapsed);
end

T = struct2table(results);
fprintf('\n========================================\n');
fprintf('POROWNANIE WAG (J TYLKO DLA DANEGO ZESTAWU WAG)\n');
fprintf('========================================\n');
disp(T);

% Nie porownuj bezposrednio J pomiedzy roznymi lambda_speed:
% kazda wartosc J odpowiada innej funkcji celu.
if ~isempty(cfg.referenceIndex)
    Eref = results(cfg.referenceIndex).Energy_J;
    fprintf('\nZmiana energii wzgledem lambda_speed=5:\n');
    for k = 1:n
        fprintf('  lambda_speed=%g: %+.3f %%\n',cfg.lambdaSpeed(k), ...
            100*(results(k).Energy_J-Eref)/Eref);
    end
end

labels = arrayfun(@(x)sprintf('lambda_\\omega = %g',x), ...
    cfg.lambdaSpeed,'UniformOutput',false);

f1 = figure('Name','Eksperyment 04 - przebiegi');
tiledlayout(3,1,'TileSpacing','compact');
nexttile; hold on; grid on;
for k=1:n, plot(traces{k}.t,traces{k}.omega,'LineWidth',1.3); end
yline(p0.reference.omega,'k:','Predkosc zadana');
yline(p0.limits.omega_max,'r:','Limit predkosci');
ylabel('\omega [rad/s]'); title('Predkosc katowa'); legend(labels,'Location','best');
nexttile; hold on; grid on;
for k=1:n, plot(traces{k}.t,traces{k}.i,'LineWidth',1.3); end
yline(p0.limits.Imax,'r:','Limit pradu');
ylabel('i_a [A]'); title('Prad twornika');
nexttile; hold on; grid on;
for k=1:n, stairs(traces{k}.t,traces{k}.u,'LineWidth',1.2); end
yline(p0.limits.Umax,'r:','Limit napiecia');
ylabel('u [V]'); xlabel('Czas [s]'); title('Napiecie sterujace');
exportgraphics(f1,fullfile(cfg.resultDir,'weights_trajectories.png'),'Resolution',200);
savefig(f1,fullfile(cfg.resultDir,'weights_trajectories.fig'));

f2 = figure('Name','Eksperyment 04 - wskazniki');
tiledlayout(2,2,'TileSpacing','compact');
x = cfg.lambdaSpeed;
nexttile; plot(x,[results.Energy_J],'-o','LineWidth',1.3); grid on;
xlabel('lambda_\omega'); ylabel('E [J]'); title('Zuzycie energii');
nexttile; plot(x,[results.IAE],'-o',x,[results.ISE],'-s','LineWidth',1.3);
grid on; xlabel('lambda_\omega'); ylabel('Wskaznik'); legend('IAE','ISE','Location','best');
title('Jakosc sledzenia');
nexttile; plot(x,[results.SettlingTime_s],'-o','LineWidth',1.3); grid on;
xlabel('lambda_\omega'); ylabel('t_s [s]'); title('Czas regulacji (2%)');
nexttile; plot(x,[results.ComputationTime_s],'-o','LineWidth',1.3); grid on;
xlabel('lambda_\omega'); ylabel('t [s]'); title('Czas obliczen DP');
exportgraphics(f2,fullfile(cfg.resultDir,'weights_metrics.png'),'Resolution',200);
savefig(f2,fullfile(cfg.resultDir,'weights_metrics.fig'));

writetable(T,fullfile(cfg.resultDir,'weights_results.csv'));
save(fullfile(cfg.resultDir,'weights_results.mat'),'T','results','traces','policies','cfg','p0');
fprintf('\nEXPERIMENT_04: zakonczony. Wyniki zapisano w: %s\n',cfg.resultDir);

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
