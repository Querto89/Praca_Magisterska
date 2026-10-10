function [omega_traj, T_traj] = simulate_baseline(p)
    omega_traj = zeros(1, p.N + 1);
    T_traj = zeros(1, p.N);
    
    omega_traj(1) = p.omega_0; 
    
    for k = 1:p.N
        T_traj(k) = p.T_base_val;
        omega_traj(k+1) = model_discrete(omega_traj(k), T_traj(k), p);
    end
end