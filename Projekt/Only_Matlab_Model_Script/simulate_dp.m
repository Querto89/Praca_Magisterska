function [omega_traj, T_traj] = simulate_dp(p, Uopt)
    omega_traj = zeros(1, p.N + 1); 
    T_traj = zeros(1, p.N); 
    omega_traj(1) = p.omega_0; 

    for k = 1:p.N
        [~, i] = min(abs(p.omega_grid - omega_traj(k)));
        T = Uopt(i, k);
        T_traj(k) = T;
        omega_traj(k+1) = model_discrete(omega_traj(k), T, p);
    end
end