function plots_security(p, log, calib)
%PLOTS_SECURITY  Wizualizacja detekcji ataku FDI opartej na Digital Twinie.
%
%  Panele:
%    1. Predkosc prawdziwa vs zmierzona (pokazuje, ile "widzi" regulator)
%    2. Residuum surowe + skalibrowane obciazenie mu(k) i pasmo +-k*sigma
%    3. Residuum znormalizowane z(k) i statystyka CUSUM
%    4. Flaga alarmu

    t  = log.t;
    tr = t(2:end);     % residuum k dotyczy chwili k+1

    figure('Name', 'Digital Twin - detekcja FDI', 'NumberTitle', 'off', ...
           'Position', [150 60 900 860]);

    %% =========================================================
    %  1. PREDKOSC
    %  =========================================================

    subplot(4,1,1);

    plot(t, log.w_true, 'g-',  'LineWidth', 1.8); hold on;
    plot(t, log.w_meas, 'r--', 'LineWidth', 1.3);
    yline(p.omega_target, ':k', '\omega_{target}');
    xline(log.attack.start, ':m', 'start ataku', 'LineWidth', 1.3);

    title(sprintf('Predkosc: rzeczywista vs zmierzona (atak: %s)', ...
        log.attack.type));
    ylabel('\omega [rad/s]');
    legend('\omega_{true} (obiekt)', '\omega_{meas} (po ataku)', ...
        'Location', 'best');
    grid on;

    %% =========================================================
    %  2. RESIDUUM SUROWE + PASMO KALIBRACYJNE
    %  =========================================================

    subplot(4,1,2);

    plot(tr, log.r, 'b-', 'LineWidth', 1.5); hold on;

    if nargin > 2 && ~isempty(calib)
        upper = calib.mu + p.det.k_sigma * calib.sd;
        lower = calib.mu - p.det.k_sigma * calib.sd;

        fill([tr, fliplr(tr)], [upper, fliplr(lower)], ...
            [0.85 0.9 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.7);

        plot(tr, calib.mu, 'k--', 'LineWidth', 1.2);
        plot(tr, log.r, 'b-', 'LineWidth', 1.5);   % na wierzchu

        legend('residuum r(k)', ...
            sprintf('pasmo \\pm%.0f\\sigma', p.det.k_sigma), ...
            '\mu(k) - obciazenie od niedopasowania', ...
            'Location', 'best');
    end

    xline(log.attack.start, ':m', 'LineWidth', 1.3);

    title('Residuum Digital Twina (predykcja jednokrokowa)');
    ylabel('r [rad/s]');
    grid on;

    %% =========================================================
    %  3. RESIDUUM ZNORMALIZOWANE + CUSUM
    %  =========================================================

    subplot(4,1,3);

    plot(tr, abs(log.z), 'b-', 'LineWidth', 1.5); hold on;
    yline(p.det.k_sigma, ':r', sprintf('prog %.0f\\sigma', p.det.k_sigma), ...
        'LineWidth', 1.4);

    if p.det.use_cusum
        plot(tr, log.cusum, 'Color', [0.9 0.5 0], 'LineWidth', 1.3);
        yline(p.det.cusum_h, ':', 'prog CUSUM', 'Color', [0.9 0.5 0], ...
            'LineWidth', 1.2);
        legend('|z(k)|', 'prog', 'CUSUM S(k)', 'Location', 'best');
    else
        legend('|z(k)|', 'prog', 'Location', 'best');
    end

    xline(log.attack.start, ':m', 'LineWidth', 1.3);

    title('Statystyka decyzyjna (residuum znormalizowane)');
    ylabel('|z| [\sigma]');
    set(gca, 'YScale', 'log');
    grid on;

    %% =========================================================
    %  4. ALARM
    %  =========================================================

    subplot(4,1,4);

    stairs(tr, log.alarm, 'LineWidth', 1.8); hold on;
    xline(log.attack.start, ':m', 'start ataku', 'LineWidth', 1.3);

    if log.detected
        xline(log.t_detect, '-r', ...
            sprintf('detekcja (opoznienie %.2f s)', log.delay_detect), ...
            'LineWidth', 1.5);
    end

    title('Flaga detekcji ataku FDI');
    ylabel('Alarm'); xlabel('Czas [s]');
    ylim([-0.2 1.2]); yticks([0 1]);
    grid on;

end
