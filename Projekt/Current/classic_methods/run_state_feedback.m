function result = run_state_feedback(p)
% STATE_FEEDBACK
% Uruchamia regulator ze sprzezeniem od stanu,
% calka uchybu i mechanizmem anti-windup.

assignin('base', 'p', p);

model_name = 'state_feedback';

load_system(model_name);

set_param(model_name, ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode45', ...
    'MaxStep', '0.001');

sim_out = sim(model_name, ...
    'StopTime', num2str(p.sim.Tsim), ...
    'ReturnWorkspaceOutputs', 'on');

% Odczyt sygnalow
omega_ts = sim_out.get('sf_omega');
i_ts     = sim_out.get('sf_i');
u_ts     = sim_out.get('sf_u');
torque_ts = sim_out.get('sf_torque');
xi_ts    = sim_out.get('sf_xi');
u_raw_ts = sim_out.get('sf_u_raw');

% Wspolna os czasu
result.t = omega_ts.Time(:);
result.omega = omega_ts.Data(:);

result.i_a = interp1( ...
    i_ts.Time(:), i_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.u = interp1( ...
    u_ts.Time(:), u_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.M_e = interp1( ...
    torque_ts.Time(:), torque_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.xi = interp1( ...
    xi_ts.Time(:), xi_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.u_raw = interp1( ...
    u_raw_ts.Time(:), u_raw_ts.Data(:), ...
    result.t, 'linear', 'extrap');

% Moc i energia elektryczna
result.P_el = result.u .* result.i_a;
result.E = cumtrapz(result.t, result.P_el);

result.method = 'state_feedback';

end