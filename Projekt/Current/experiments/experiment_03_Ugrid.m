%% EXPERIMENT 03: Wplyw rozdzielczosci siatki napiecia DP
% Plik: experiments/experiment_03_Ugrid.m
% Wymagane: params.m, dp_solver.m, run_simulink.m, system_model.slx.
% Pozostale parametry sa stale. Siatki 16, 31 i 61 sa zagniezdzone.
% Skrypt nie zmienia motywu ani tla wykresow MATLAB-a.

clearvars; clc;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root,'experiments'));
resultsDir = fullfile(root,'results','experiment_03');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

p0 = params();
nU = [16 31 61];
% Zakres napiecia pozostaje bez zmian: [p0.limits.Umin, p0.limits.Umax].
% Wspolna siatka czasowa do obliczania metryk i wykresow.
dtMetric = 0.001;
tol = 1e-6;

assert(exist('dp_solver','file')==2, 'Nie znaleziono dp_solver.m');
assert(exist('run_simulink','file')==2, 'Nie znaleziono run_simulink.m');
assert(all(nU>=2) && all(mod((nU-1)/(nU(1)-1),1)==0), ...
    'Wybrane siatki nie sa zagniezdzone.');

fprintf('\n========================================\n');
fprintf('EKSPERYMENT 03: WPLYW SIATKI U_grid\n');
fprintf('========================================\n');
fprintf('N_U: %s\n', mat2str(nU));
fprintf('Zakres: [%.3f, %.3f] V\n',p0.limits.Umin,p0.limits.Umax);

runs = cell(numel(nU),1);
dpResults = cell(numel(nU),1);
rows = repmat(struct(),numel(nU),1);
tCommon = (0:dtMetric:p0.sim.Tsim)';

for k = 1:numel(nU)
    p = p0;
    p.dp.U_grid = linspace(p.limits.Umin,p.limits.Umax,nU(k));
    dU = p.dp.U_grid(2)-p.dp.U_grid(1);
    fprintf('\n----------------------------------------\n');
    fprintf('SIATKA %d/%d: N_U=%d, Delta U=%.4f V\n', ...
        k,numel(nU),nU(k),dU);
    fprintf('----------------------------------------\n');

    tStart = tic;
    dp = dp_solver(p);
    dpTime = toc(tStart);
    fprintf('Czas obliczen DP: %.3f s\n',dpTime);

    % Weryfikacja, ze solver rzeczywiscie zwrocil dopuszczalne sterowanie.
    uDP = dp.u_opt(:);
    assert(numel(uDP)==p.sim.N, 'Nieprawidlowa liczba probek dp.u_opt.');
    assert(all(isfinite(uDP)), 'dp.u_opt zawiera NaN/Inf.');
    assert(all(uDP>=p.limits.Umin-tol & uDP<=p.limits.Umax+tol), ...
        'Sterowanie DP wykracza poza zakres napiecia.');

    fprintf('Symulacja obiektu Simulink...\n');
    raw = run_simulink(p,uDP);
    r = normalizePlant(raw,uDP,p);
    r = resamplePlant(r,tCommon,uDP,p);
    m = calculateLocalMetrics(r,p,tol);
    [J,Jsource] = getObjective(dp,p);

    fprintf('Energia: %.6f J | J: %.6f (%s)\n',m.Energy_J,J,Jsource);
    fprintf('IAE: %.6f | ISE: %.6f | czas regulacji: %.4f s\n', ...
        m.IAE,m.ISE,m.SettlingTime_s);
    fprintf('Imax: %.6f A | Umax: %.6f V\n',m.MaxCurrent_A,m.MaxVoltage_V);

    rows(k).N_U = nU(k);
    rows(k).DeltaU_V = dU;
    rows(k).J = J;
    rows(k).J_source = string(Jsource);
    rows(k).Energy_J = m.Energy_J;
    rows(k).IAE = m.IAE;
    rows(k).ISE = m.ISE;
    rows(k).Overshoot_pct = m.Overshoot_pct;
    rows(k).SettlingTime_s = m.SettlingTime_s;
    rows(k).MaxCurrent_A = m.MaxCurrent_A;
    rows(k).MaxVoltage_V = m.MaxVoltage_V;
    rows(k).CurrentLimitExceeded = m.CurrentLimitExceeded;
    rows(k).VoltageLimitExceeded = m.VoltageLimitExceeded;
    rows(k).SpeedLimitExceeded = m.SpeedLimitExceeded;
    rows(k).ComputationTime_s = dpTime;
    runs{k} = r;
    dpResults{k} = dp;
end

metrics = struct2table(rows);
fprintf('\n========================================\n');
fprintf('POROWNANIE SIATEK U_grid\n');
fprintf('========================================\n');
disp(metrics);

% Wykresy. Brak jakichkolwiek zmian figure.Color/gca.Color/motywu.
labels = arrayfun(@(n) sprintf('N_U = %d',n),nU,'UniformOutput',false);
fig1 = figure('Name','Eksperyment 03 - trajektorie','NumberTitle','off');
tiledlayout(3,1,'TileSpacing','compact');
nexttile; hold on;
for k=1:numel(nU), plot(tCommon,runs{k}.omega,'LineWidth',1.2); end
yline(p0.reference.omega,'--','Predkosc zadana');
yline(p0.limits.omega_max,':','Limit predkosci');
title('Predkosc katowa'); ylabel('\omega [rad/s]');
legend(labels,'Location','best'); grid on;
nexttile; hold on;
for k=1:numel(nU), plot(tCommon,runs{k}.i,'LineWidth',1.2); end
yline(p0.limits.Imax,':','Limit pradu');
title('Prad twornika'); ylabel('i_a [A]'); grid on;
nexttile; hold on;
for k=1:numel(nU), stairs(tCommon,runs{k}.u,'LineWidth',1.2); end
title('Napiecie sterujace (ZOH)'); ylabel('u [V]');
xlabel('Czas [s]'); grid on;

fig2 = figure('Name','Eksperyment 03 - jakosc i zlozonosc','NumberTitle','off');
tiledlayout(2,2,'TileSpacing','compact');
nexttile; plot(nU,metrics.J,'-o','LineWidth',1.3); grid on;
xlabel('Liczba sterowan N_U'); ylabel('J'); title('Wartosc funkcji celu');
nexttile; plot(nU,metrics.Energy_J,'-o','LineWidth',1.3); grid on;
xlabel('Liczba sterowan N_U'); ylabel('E [J]'); title('Zuzycie energii');
nexttile; plot(nU,metrics.IAE,'-o','LineWidth',1.3); hold on;
plot(nU,metrics.ISE,'-s','LineWidth',1.3); grid on;
xlabel('Liczba sterowan N_U'); ylabel('Wskaznik');
title('Jakosc sledzenia'); legend('IAE','ISE','Location','best');
nexttile; plot(nU,metrics.ComputationTime_s,'-o','LineWidth',1.3); grid on;
xlabel('Liczba sterowan N_U'); ylabel('Czas [s]'); title('Czas obliczen DP');

fig3 = figure('Name','Eksperyment 03 - energia i czas regulacji','NumberTitle','off');
tiledlayout(1,2);
nexttile; bar(categorical(labels),metrics.Energy_J); grid on;
ylabel('E [J]'); title('Zuzycie energii');
nexttile; bar(categorical(labels),metrics.SettlingTime_s); grid on;
ylabel('t_s [s]'); title('Czas regulacji (pasmo 2%)');

save(fullfile(resultsDir,'experiment_03_results.mat'), ...
    'p0','nU','dtMetric','metrics','runs','dpResults');
writetable(metrics,fullfile(resultsDir,'experiment_03_metrics.csv'));
savefig(fig1,fullfile(resultsDir,'experiment_03_trajectories.fig'));
savefig(fig2,fullfile(resultsDir,'experiment_03_metrics.fig'));
savefig(fig3,fullfile(resultsDir,'experiment_03_energy_settling.fig'));

if any(metrics.CurrentLimitExceeded | metrics.VoltageLimitExceeded | ...
       metrics.SpeedLimitExceeded)
    warning('EXPERIMENT_03: co najmniej jedna trajektoria przekracza ograniczenia.');
end
if all(metrics.J_source=="dp_solver") && any(diff(metrics.J)>1e-5)
    warning(['Funkcja celu J wzrosla po zageszczeniu siatki. ' ...
        'Sprawdz interpolacje DP i metode odczytu J.']);
end
fprintf('\nEXPERIMENT_03: zakonczony. Wyniki zapisano w: %s\n',resultsDir);

%% FUNKCJE LOKALNE
function r = normalizePlant(raw,uDP,p)
    r.t = pickField(raw,{'t','time','tout'});
    r.omega = pickField(raw,{'omega','omega_sim','w'});
    r.i = pickField(raw,{'i_a','i','current'});
    try
        r.u = pickField(raw,{'u','voltage','u_a'});
    catch
        r.u = [];
    end
    r.t = r.t(:); r.omega = r.omega(:); r.i = r.i(:);
    if ~isempty(r.u), r.u = r.u(:); end
    if numel(r.t)~=numel(r.omega) || numel(r.t)~=numel(r.i)
        error('Niezgodne rozmiary sygnalow run_simulink.');
    end
    [r.t,ix] = unique(r.t,'last');
    r.omega = r.omega(ix); r.i = r.i(ix);
    if ~isempty(r.u), r.u = r.u(ix); end
    assert(all(isfinite([r.t;r.omega;r.i])), 'Niepoprawne probki symulacji.');
end
function r = resamplePlant(r,t,uDP,p)
    % Wspolna siatka do metryk. Napiecie: dokladnie ten sam ZOH co DP.
    r.omega = interp1(r.t,r.omega,t,'linear','extrap');
    r.i = interp1(r.t,r.i,t,'linear','extrap');
    tu = (0:numel(uDP)-1)'*p.sim.dt;
    r.u = interp1(tu,uDP,t,'previous','extrap');
    r.t = t;
    r.energy = cumtrapz(t,r.u.*r.i);
end
function x = pickField(s,names)
    for j=1:numel(names)
        if isstruct(s) && isfield(s,names{j})
            x = s.(names{j});
            if isa(x,'timeseries'), x=x.Data; end
            x = x(:); return;
        end
    end
    error('Nie znaleziono pola. Dostepne: %s',strjoin(fieldnames(s),', '));
end
function m = calculateLocalMetrics(r,p,tol)
    e = p.reference.omega-r.omega;
    m.Energy_J = r.energy(end);
    m.IAE = trapz(r.t,abs(e));
    m.ISE = trapz(r.t,e.^2);
    m.Overshoot_pct = max(0,100*(max(r.omega)-p.reference.omega)/p.reference.omega);
    outside = find(abs(e)>0.02*abs(p.reference.omega));
    if isempty(outside)
        m.SettlingTime_s = 0;
    elseif outside(end)==numel(r.t)
        m.SettlingTime_s = NaN;
    else
        m.SettlingTime_s = r.t(outside(end)+1);
    end
    m.MaxCurrent_A = max(r.i);
    m.MaxVoltage_V = max(r.u);
    m.CurrentLimitExceeded = any(r.i>p.limits.Imax+tol | r.i<p.limits.Imin-tol);
    m.VoltageLimitExceeded = any(r.u>p.limits.Umax+tol | r.u<p.limits.Umin-tol);
    m.SpeedLimitExceeded = any(r.omega>p.limits.omega_max+tol | ...
        r.omega<p.limits.omega_min-tol);
end
function [J,source] = getObjective(dp,p)
    % Pierwszenstwo: dokladna wartosc funkcji celu zwrocona przez solver.
    for field = {'J_opt','J','J_total','cost_opt','total_cost'}
        name = field{1};
        if isfield(dp,name) && isnumeric(dp.(name)) && isscalar(dp.(name))
            J=dp.(name); source='dp_solver'; return;
        end
    end
    % Rezerwowo: odtworzenie kosztu z trajektorii zwroconej przez solver.
    % To oszacowanie; nie nalezy utozsamiac go z wartoscia Bellmana,
    % jezeli solver uzywa interpolacji wartosci albo innej kwadratury.
    if isfield(dp,'omega_opt') && isfield(dp,'E_opt') && ...
            isnumeric(dp.E_opt) && isscalar(dp.E_opt)
        w = dp.omega_opt(:);
        if numel(w)==p.sim.N+1
            wStage=w(1:end-1);
        elseif numel(w)==p.sim.N
            wStage=w;
        else
            error('Nieznana dlugosc dp.omega_opt; nie mozna odtworzyc J.');
        end
        J=p.dp.lambda_energy*dp.E_opt + ...
          p.dp.lambda_speed*p.sim.dt*sum((wStage-p.reference.omega).^2) + ...
          p.dp.lambda_terminal*(w(end)-p.reference.omega)^2;
        source='estimated_from_trajectory';
        warning('Solver nie udostepnia J_opt. J oszacowano z trajektorii.');
    else
        error(['Nie znaleziono wartosci J w dp_solver. ' ...
            'Dodaj pole dp.J_opt do wyniku solvera lub dostosuj getObjective.']);
    end
end
