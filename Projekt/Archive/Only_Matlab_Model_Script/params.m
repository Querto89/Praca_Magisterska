function p = params()
    p.J  = 0.01;
    p.B  = 0.1;
    p.dt = 0.05;
    p.omega_0 = 0;
    p.T_base_val = 1.2;
    p.beta = 5;
    p.Tmax = 2;
    p.omega_max = 12;
    p.omega_target = 10;
    p.N = 40;
    p.omega_grid = linspace(0, p.omega_max, 60);
    p.T_grid     = linspace(0, p.Tmax, 30);
    p.alpha_terminal = 100;   % waga kosztu końcowego
end
