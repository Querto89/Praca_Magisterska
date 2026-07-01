function omega_next = model_discrete(omega, T, p)
    omega_next = omega + p.dt*(1/p.J)*(T - p.B*omega);
end