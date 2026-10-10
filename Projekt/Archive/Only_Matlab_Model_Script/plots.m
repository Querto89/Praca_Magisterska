function plots(p, omega_dp, T_dp, omega_base, T_base)
    t_state = (0:p.N) * p.dt;
    t_ctrl  = (0:p.N-1) * p.dt;
    
    figure('Name', 'Wyniki Optymalizacji DP vs Baseline', 'NumberTitle', 'off');
    
    % --- WYKRES PRĘDKOŚCI ---
    subplot(3,1,1);
    stairs(t_state, omega_dp, 'LineWidth', 1.5); hold on;
    plot(t_state, omega_base, '--', 'LineWidth', 1.5);
    title(['Dynamika prędkości (\alpha=', num2str(p.alpha_terminal), ', \beta=', num2str(p.beta), ')']);
    ylabel('\omega [rad/s]');
    legend('DP', 'Baseline', 'Location', 'best');
    grid on;
    
    % --- WYKRES MOMENTU ---
    subplot(3,1,2);
    % Używamy t_ctrl, bo T ma długość N
    stairs(t_ctrl, T_dp, 'LineWidth', 1.5); hold on;
    plot(t_ctrl, T_base, '--', 'LineWidth', 1.5);
    title('Moment napędowy (Zmienna decyzyjna)');
    ylabel('T [Nm]');
    grid on;
    
    % --- WYKRES ENERGII ---
    subplot(3,1,3);
    % Obliczamy energię (N+1 punktów, zaczynając od 0)
    E_dp = [0, cumsum(T_dp .* omega_dp(1:end-1)) * p.dt];
    E_base = [0, cumsum(T_base .* omega_base(1:end-1)) * p.dt];
    
    stairs(t_state, E_dp, 'LineWidth', 1.5); hold on;
    plot(t_state, E_base, '--', 'LineWidth', 1.5);
    title('Zużycie energii mechanicznej');
    ylabel('E [J]');
    xlabel('t [s]');
    grid on;
end