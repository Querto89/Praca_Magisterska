function plots(p, omega_dp, T_dp, E_dp, omega_base, T_base, E_base)
    omega_dp = omega_dp(:)'; omega_base = omega_base(:)';
    T_dp = T_dp(:)'; T_base = T_base(:)';
    E_dp = E_dp(:)'; E_base = E_base(:)';
    t_omega = (0:length(omega_dp)-1) * p.dt;
    t_T     = (0:length(T_dp)-1) * p.dt;
    t_E     = (0:length(E_dp)-1) * p.dt;

    figure('Name', 'Porównanie DP vs Baseline (Simulink)', 'NumberTitle', 'off');

    subplot(3,1,1);
    stairs(t_omega, omega_dp, 'LineWidth', 1.5); hold on;
    plot(t_omega, omega_base, '--', 'LineWidth', 1.5);
    title(['Dynamika (\alpha=', num2str(p.alpha_terminal), ', \beta=', num2str(p.beta), ')']);
    ylabel('\omega [rad/s]');
    legend('DP', 'Baseline', 'Location', 'best');
    grid on;

    subplot(3,1,2);
    stairs(t_T, T_dp, 'LineWidth', 1.5); hold on;
    plot(t_T, T_base, '--', 'LineWidth', 1.5);
    title('Moment napędowy (Zmienna decyzyjna)');
    ylabel('T [Nm]');
    grid on;

    subplot(3,1,3);
    stairs(t_E, E_dp, 'LineWidth', 1.5); hold on;
    plot(t_E, E_base, '--', 'LineWidth', 1.5);
    title('Zużycie energii mechanicznej');
    ylabel('E [J]');
    xlabel('t [s]');
    grid on;
end