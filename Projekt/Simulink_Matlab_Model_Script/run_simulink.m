function [w_sim, T_sim, E_sim, t_sim] = run_simulink(p, Uopt, T_base_val)
    model_name = 'system_model'; 
    
    t_input = (0 : p.N)' * p.dt; 
    
    T_dp_values = zeros(p.N, 1);
    curr_w = p.omega_0;
    for k = 1:p.N
        [~, idx] = min(abs(p.omega_grid - curr_w));
        T = Uopt(idx, k);
        T_dp_values(k) = T;
        curr_w = curr_w + p.dt*(1/p.J)*(T - p.B*curr_w);
    end
    
    T_dp_data = [t_input, [T_dp_values; T_dp_values(end)]];
    T_base_data = [t_input, ones(p.N + 1, 1) * T_base_val];
    
    load_system(model_name);
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'Solver', 'ode1');
    set_param(model_name, 'FixedStep', num2str(p.dt));
    set_param(model_name, 'StopTime', num2str(p.N * p.dt));
    
    assignin('base', 'T_dp_simulink', T_dp_data); 
    assignin('base', 'T_base', T_base_data);       
    
    sim_out = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
    
    t_sim = sim_out.tout;
    w_sim.dp   = sim_out.find('w_dp');
    T_sim.dp   = sim_out.find('T_dp');
    E_sim.dp   = sim_out.find('E_dp');
    
    w_sim.base = sim_out.find('w_base');
    T_sim.base = sim_out.find('T_base');
    E_sim.base = sim_out.find('E_base');
end