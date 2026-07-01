function Uopt = dp_solver(p)
    V = inf(length(p.omega_grid), p.N+1);
    Uopt = zeros(length(p.omega_grid), p.N);
    
    V(:, end) = p.alpha_terminal * (p.omega_grid - p.omega_target).^2;
    
    for k = p.N:-1:1
        for i = 1:length(p.omega_grid)
            omega = p.omega_grid(i);
            
            for u = 1:length(p.T_grid)
                T = p.T_grid(u);
                
                omega_next = omega + p.dt * (1/p.J) * (T - p.B * omega);
                [~, j] = min(abs(p.omega_grid - omega_next));
                L = (T * omega * p.dt) + (p.beta * (omega - p.omega_target)^2 * p.dt);
                
                cost = L + V(j, k+1);
                
                if cost < V(i, k)
                    V(i, k) = cost;
                    Uopt(i, k) = T;
                end
            end
        end
    end
end