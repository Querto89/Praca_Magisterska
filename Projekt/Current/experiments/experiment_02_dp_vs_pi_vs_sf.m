%% EXPERIMENT 02: DP vs kaskadowy PI vs sprzezenie od stanu z calkowaniem
% Plik: experiments/experiment_02_dp_vs_pi_vs_sf.m
% Modele: classic_methods/cascade_control.slx i classic_methods/state_feedback.slx
% Skrypt nie zmienia motywu ani kolorow tla wykresow MATLAB.

clearvars; clc;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root,'classic_methods'), fullfile(root,'experiments'));
resultsDir = fullfile(root,'results','experiment_02');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

p = params();
assignin('base','p',p);

cfg.piModel = 'cascade_control';
cfg.sfModel = 'state_feedback';
% Jesli nazwy To Workspace sa inne, funkcja resolveSignals odczyta je
% z modeli. Mozesz tez podac je recznie ponizej.
cfg.pi.omega = ''; cfg.pi.current = ''; cfg.pi.voltage = '';
cfg.sf.omega = ''; cfg.sf.current = ''; cfg.sf.voltage = '';
cfg.maxStep = '0.001';
cfg.sampleTime = 0.001;
cfg.tol = 1e-6;

% Kontrola plikow i sygnalow PRZED kosztownym obliczeniem DP.
cfg.pi = resolveSignals(cfg.piModel,cfg.pi,'PI');
cfg.sf = resolveSignals(cfg.sfModel,cfg.sf,'SF');
fprintf('\n========================================\n');
fprintf('EKSPERYMENT 02: DP vs PI vs SF\n');
fprintf('========================================\n');
fprintf('PI: %s | omega=%s, i=%s, u=%s\n',cfg.piModel, ...
    cfg.pi.omega,cfg.pi.current,cfg.pi.voltage);
fprintf('SF: %s | omega=%s, i=%s, u=%s\n',cfg.sfModel, ...
    cfg.sf.omega,cfg.sf.current,cfg.sf.voltage);

%% 1. DP i niezalezny obiekt Simulink
fprintf('\nObliczanie DP...\n');
tic; dp = dp_solver(p); computationTimeDP = toc;
fprintf('Czas obliczen DP: %.3f s\n',computationTimeDP);
fprintf('Symulacja obiektu z sygnalem DP...\n');
rawDP = run_simulink(p,dp.u_opt);
DP = normalizePlant(rawDP,dp,p);

%% 2. Dwa regulatory klasyczne
fprintf('\nSymulacja regulatora PI...\n');
PI = simulateController(cfg.piModel,cfg.pi,p,cfg.maxStep);
fprintf('\nSymulacja regulatora SF...\n');
SF = simulateController(cfg.sfModel,cfg.sf,p,cfg.maxStep);

%% 3. Wspolna siatka i identyczne wskazniki
% Przebiegi napiecia DP sa ZOH, napiecia regulatorow sa ciagle.
t = (0:cfg.sampleTime:p.sim.Tsim)';
DP = resampleRun(DP,t,'previous');
PI = resampleRun(PI,t,'linear');
SF = resampleRun(SF,t,'linear');

mDP = localMetrics(DP,p,cfg.tol);
mPI = localMetrics(PI,p,cfg.tol);
mSF = localMetrics(SF,p,cfg.tol);
metrics = [mDP;mPI;mSF];
metrics.Method = categorical({'DP';'PI';'SF'});
metrics = movevars(metrics,'Method','Before',1);
fprintf('\n========================================\n');
fprintf('POROWNANIE WSKAZNIKOW JAKOSCI\n');
fprintf('========================================\n');
disp(metrics);

E = metrics.Energy_J;
energySavingsVsPI_pct = 100*(E(2)-E(1))/E(2);
energySavingsVsSF_pct = 100*(E(3)-E(1))/E(3);
fprintf('Zmniejszenie energii DP wzgledem PI: %.3f %%\n',energySavingsVsPI_pct);
fprintf('Zmniejszenie energii DP wzgledem SF: %.3f %%\n',energySavingsVsSF_pct);

%% 4. Wykresy: bez ustawiania tla figure/gca
fig1 = figure('Name','Eksperyment 02 - przebiegi','NumberTitle','off');
tiledlayout(4,1,'TileSpacing','compact');
nexttile;
plot(t,DP.omega,'LineWidth',1.3); hold on;
plot(t,PI.omega,'--','LineWidth',1.3);
plot(t,SF.omega,':','LineWidth',1.6);
yline(p.reference.omega,'-.','Predkosc zadana');
yline(p.limits.omega_max,':','Limit predkosci');
ylabel('\omega [rad/s]'); title('Predkosc katowa');
legend('DP','PI','SF','Location','best'); grid on;
nexttile;
plot(t,DP.i,'LineWidth',1.3); hold on;
plot(t,PI.i,'--','LineWidth',1.3);
plot(t,SF.i,':','LineWidth',1.6);
yline(p.limits.Imax,':','Limit pradu');
ylabel('i_a [A]'); title('Prad twornika'); grid on;
nexttile;
stairs(t,DP.u,'LineWidth',1.3); hold on;
plot(t,PI.u,'--','LineWidth',1.3);
plot(t,SF.u,':','LineWidth',1.6);
yline(p.limits.Umax,':','Limit napiecia');
ylabel('u [V]'); title('Napiecie sterujace'); grid on;
nexttile;
plot(t,DP.energy,'LineWidth',1.3); hold on;
plot(t,PI.energy,'--','LineWidth',1.3);
plot(t,SF.energy,':','LineWidth',1.6);
ylabel('E [J]'); xlabel('Czas [s]'); title('Energia elektryczna'); grid on;

fig2 = figure('Name','Eksperyment 02 - wskazniki','NumberTitle','off');
tiledlayout(1,2);
nexttile; bar(categorical({'DP','PI','SF'}),metrics.Energy_J);
ylabel('E [J]'); title('Zuzycie energii'); grid on;
nexttile; bar(categorical({'DP','PI','SF'}),metrics.SettlingTime_s);
ylabel('t_s [s]'); title('Czas regulacji (pasmo 2%)'); grid on;

%% 5. Wyniki
save(fullfile(resultsDir,'experiment_02_results.mat'), ...
    'p','cfg','dp','DP','PI','SF','metrics','computationTimeDP', ...
    'energySavingsVsPI_pct','energySavingsVsSF_pct');
writetable(metrics,fullfile(resultsDir,'experiment_02_metrics.csv'));
savefig(fig1,fullfile(resultsDir,'experiment_02_trajectories.fig'));
savefig(fig2,fullfile(resultsDir,'experiment_02_metrics.fig'));
fprintf('\nEXPERIMENT_02: OK. Wyniki zapisano w: %s\n',resultsDir);

%% FUNKCJE LOKALNE
function signal = resolveSignals(modelName,signal,method)
    if isempty(which([modelName '.slx'])) && isempty(which([modelName '.mdl']))
        error('Nie znaleziono modelu %s. Sprawdz folder classic_methods.',modelName);
    end
    load_system(modelName);
    blocks = find_system(modelName, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'MatchFilter', @Simulink.match.activeVariants, ...
    'BlockType', 'ToWorkspace');
    vars = cell(size(blocks));
    for k=1:numel(blocks)
        vars{k} = get_param(blocks{k},'VariableName');
    end
    % Dla kazdej wielkosci wymagamy jednoznacznego dopasowania.
    signal.omega = chooseVar(signal.omega,vars,blocks,method,'predkosc', ...
        {'omega','speed','predkosc','w_mech','w_motor'});
    signal.current = chooseVar(signal.current,vars,blocks,method,'prad', ...
        {'current','i_a','ia','prad','motor_i','_i'});
    signal.voltage = chooseVar(signal.voltage,vars,blocks,method,'napiecie', ...
        {'voltage','u_a','ua','napiecie','motor_u','_u'});
    selected = {signal.omega,signal.current,signal.voltage};
    if numel(unique(selected))~=3
        error('Model %s: ten sam sygnal wybrano dla roznych wielkosci.',modelName);
    end
end
function name = chooseVar(explicit,vars,blocks,method,quantity,patterns)
    if ~isempty(explicit)
        if ~ismember(explicit,vars)
            error('%s: sygnal %s=%s nie istnieje w To Workspace.',method,quantity,explicit);
        end
        name=explicit; return;
    end
    score=zeros(size(vars));
    for k=1:numel(vars)
        v=lower(vars{k}); b=lower(blocks{k});
        if startsWith(v,lower(method)),score(k)=score(k)+1;end
        for j=1:numel(patterns)
            if contains(v,patterns{j}),score(k)=score(k)+10;end
            if contains(b,patterns{j}),score(k)=score(k)+2;end
        end
        if contains(v,'ref') || contains(v,'zad') || contains(v,'raw') || ...
                contains(v,'error') || contains(v,'uchyb')
            score(k)=score(k)-30;
        end
    end
    candidates=find(score>0);
    if numel(candidates)==1
        name=vars{candidates}; return;
    end
    if ~isempty(candidates)
        best=max(score(candidates)); candidates=candidates(score(candidates)==best);
        if numel(candidates)==1
            name=vars{candidates}; return;
        end
    end
    fprintf('\nDostepne sygnaly To Workspace dla %s:\n',method);
    for k=1:numel(vars),fprintf('  %s  ->  %s\n',blocks{k},vars{k});end
    error(['Nie mozna jednoznacznie rozpoznac sygnalu %s (%s). ' ...
        'Ustaw odpowiednie cfg.pi/cfg.sf na poczatku skryptu.'],quantity,method);
end
function run = normalizePlant(raw,dp,p)
    run.t = pick(raw,{'t','time','tout'});
    run.omega = pick(raw,{'omega','omega_sim','w'});
    run.i = pick(raw,{'i_a','i','current'});
    try
        run.u = pick(raw,{'u','voltage','u_a'});
    catch
        tt=(0:numel(dp.u_opt)-1)'*p.sim.dt;
        run.u=interp1(tt,dp.u_opt(:),run.t,'previous','extrap');
    end
    run=sanitize(run);
end
function run = simulateController(modelName,signal,p,maxStep)
    load_system(modelName);
    assignin('base','p',p);
    out=sim(modelName,'StopTime',num2str(p.sim.Tsim), ...
        'SolverType','Variable-step','Solver','ode45', ...
        'MaxStep',maxStep,'ReturnWorkspaceOutputs','on');
    [to,wo]=unpackTS(out.get(signal.omega));
    [ti,ii]=unpackTS(out.get(signal.current));
    [tu,uu]=unpackTS(out.get(signal.voltage));
    run.t=unique([to;ti;tu]);
    run.omega=interp1(to,wo,run.t,'linear','extrap');
    run.i=interp1(ti,ii,run.t,'linear','extrap');
    run.u=interp1(tu,uu,run.t,'linear','extrap');
    run=sanitize(run);
end
function [t,x] = unpackTS(v)
    if isa(v,'timeseries')
        t=v.Time(:); x=squeeze(v.Data); x=x(:);
    elseif isstruct(v) && isfield(v,'time') && isfield(v,'signals')
        t=v.time(:); x=v.signals.values(:);
    else
        error('Sygnał To Workspace nie jest timeseries ani Structure with time.');
    end
    [t,idx]=unique(t,'last'); x=x(idx);
end
function v=pick(s,names)
    for k=1:numel(names)
        if isstruct(s) && isfield(s,names{k})
            v=s.(names{k});
            if isa(v,'timeseries'),v=v.Data;end
            v=v(:);return;
        end
    end
    error('Brak oczekiwanego pola w run_simulink. Dostepne: %s', ...
        strjoin(fieldnames(s),', '));
end
function r=sanitize(r)
    r.t=r.t(:);r.omega=r.omega(:);r.i=r.i(:);r.u=r.u(:);
    [r.t,idx]=unique(r.t,'last');
    r.omega=r.omega(idx);r.i=r.i(idx);r.u=r.u(idx);
    assert(all(isfinite([r.t;r.omega;r.i;r.u])),'Niefinitywne probki.');
end
function r=resampleRun(r,t,uInterpolation)
    r.omega=interp1(r.t,r.omega,t,'linear','extrap');
    r.i=interp1(r.t,r.i,t,'linear','extrap');
    r.u=interp1(r.t,r.u,t,uInterpolation,'extrap');
    r.t=t;
    r.energy=cumtrapz(t,r.u.*r.i);
end
function m=localMetrics(r,p,tol)
    e=p.reference.omega-r.omega;
    m=table(r.energy(end),trapz(r.t,abs(e)),trapz(r.t,e.^2), ...
        max(0,100*(max(r.omega)-p.reference.omega)/p.reference.omega), ...
        settling(r.t,r.omega,p.reference.omega),max(r.i),max(r.u), ...
        any(r.i>p.limits.Imax+tol | r.i<p.limits.Imin-tol), ...
        any(r.u>p.limits.Umax+tol | r.u<p.limits.Umin-tol), ...
        any(r.omega>p.limits.omega_max+tol | r.omega<p.limits.omega_min-tol), ...
        'VariableNames',{'Energy_J','IAE','ISE','Overshoot_pct', ...
        'SettlingTime_s','MaxCurrent_A','MaxVoltage_V', ...
        'CurrentLimitExceeded','VoltageLimitExceeded','SpeedLimitExceeded'});
end
function ts=settling(t,w,ref)
    outside=find(abs(w-ref)>0.02*abs(ref));
    if isempty(outside),ts=0;
    elseif outside(end)==numel(t),ts=NaN;
    else,ts=t(outside(end)+1);
    end
end
