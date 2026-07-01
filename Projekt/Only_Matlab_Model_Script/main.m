clear;
clc;

p = params();

Uopt = dp_solver(p);
[omega_dp, T_dp] = simulate_dp(p, Uopt);

[omega_base, T_base] = simulate_baseline(p);

plots(p, omega_dp, T_dp, omega_base, T_base);
