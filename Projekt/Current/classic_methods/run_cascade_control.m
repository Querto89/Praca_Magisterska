function result = run_cascade_control(p)
% CASCADE_CONTROL
% Uruchamia kaskadowy regulator PI predkosci i pradu.

assignin('base', 'p', p);

model_name = 'cascade_control';

load_system(model_name);

set_param(model_name, ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode45', ...
    'MaxStep', '0.001');

sim_out = sim(model_name, ...
    'StopTime', num2str(p.sim.Tsim), ...
    'ReturnWorkspaceOutputs', 'on');

% Odczyt sygnalow
i_ref_ts  = sim_out.get('classic_i_ref');
i_ts      = sim_out.get('classic_i');
u_ts      = sim_out.get('classic_u');
omega_ts  = sim_out.get('classic_omega');
torque_ts = sim_out.get('classic_torque');

% Wspolna os czasu
result.t = omega_ts.Time(:);
result.omega = omega_ts.Data(:);

result.i_ref = interp1( ...
    i_ref_ts.Time(:), i_ref_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.i_a = interp1( ...
    i_ts.Time(:), i_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.u = interp1( ...
    u_ts.Time(:), u_ts.Data(:), ...
    result.t, 'linear', 'extrap');

result.M_e = interp1( ...
    torque_ts.Time(:), torque_ts.Data(:), ...
    result.t, 'linear', 'extrap');

% Moc i energia elektryczna
result.P_el = result.u .* result.i_a;
result.E = cumtrapz(result.t, result.P_el);

result.method = 'cascade_pi';

end