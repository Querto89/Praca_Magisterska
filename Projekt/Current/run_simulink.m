function result = run_simulink(p, u)
% RUN_SIMULINK
% Uruchamia niezalezny model obiektu silnika DC w Simulinku.
%
% Wejscia:
%   p - struktura parametrow z params.m
%   u - wektor napiecia twornika [V]
%
% Wyjscie:
%   result - struktura zawierajaca wyniki symulacji:
%
%       result.t
%       result.u
%       result.i_a
%       result.omega
%       result.M_e
%       result.E


%% ============================================================
%  PRZYGOTOWANIE SYGNALU STERUJACEGO
%  ============================================================

u = u(:);

N  = p.sim.N;
dt = p.sim.dt;

if length(u) ~= N
    error(['Wektor sterowania u musi zawierac dokladnie ', ...
           num2str(N), ' probek.']);
end


%% ============================================================
%  OGRANICZENIE NAPIECIA
%  ============================================================

u = min(max(u, p.limits.Umin), p.limits.Umax);


%% ============================================================
%  SYGNAL DLA BLOKU FROM WORKSPACE
%
%  Dodajemy ostatnia probke w chwili Tsim, aby sygnal byl
%  zdefiniowany na calym przedziale symulacji.
%  ============================================================

t_input = (0:N)' * dt;

u_input = [u;
           u(end)];

u_simulink = timeseries(u_input, t_input);


%% ============================================================
%  PRZEKAZANIE DANYCH DO WORKSPACE
%  ============================================================

assignin('base', 'p', p);
assignin('base', 'u_simulink', u_simulink);


%% ============================================================
%  URUCHOMIENIE MODELU SIMULINK
%  ============================================================

model_name = 'system_model';

load_system(model_name);

% Konfiguracja solvera
set_param(model_name, ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode45', ...
    'MaxStep', '0.0001');

sim_out = sim(model_name, ...
    'StopTime', num2str(p.sim.Tsim), ...
    'ReturnWorkspaceOutputs', 'on');


%% ============================================================
%  ODCZYT WYNIKOW
%  ============================================================

omega_ts  = sim_out.get('out_omega');
i_ts      = sim_out.get('out_i');
torque_ts = sim_out.get('out_torque');
u_ts      = sim_out.get('out_u');


%% ============================================================
%  ZAPIS DO STRUKTURY WYNIKOWEJ
%  ============================================================

result.t     = omega_ts.Time(:);
result.omega = omega_ts.Data(:);

result.i_a = i_ts.Data(:);
result.M_e = torque_ts.Data(:);

result.u = u_ts.Data(:);


%% ============================================================
%  ENERGIA ELEKTRYCZNA
%
%  E(t) = integral u(t)*i_a(t) dt
%  ============================================================

% Dane zapisane przez Simulink powinny miec wspolna os czasu.
% Dla bezpieczenstwa interpolujemy napiecie i prad na os
% czasowa predkosci.

u_common = interp1( ...
    u_ts.Time(:), ...
    u_ts.Data(:), ...
    result.t, ...
    'previous', ...
    'extrap');

i_common = interp1( ...
    i_ts.Time(:), ...
    i_ts.Data(:), ...
    result.t, ...
    'linear', ...
    'extrap');

P_el = u_common .* i_common;

result.P_el = P_el;
result.E = cumtrapz(result.t, P_el);

end