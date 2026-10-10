function plots_nominal(p, log_dp, log_base)
%PLOTS_NOMINAL  Analiza jakosci sterowania: DP vs regulator odniesienia.
%
%  Pokazuje, co daje optymalizacja DP w warunkach BEZ ATAKU:
%  trajektorie predkosci, napiecie sterujace, prad i zuzycie energii.

    t  = log_dp.t;
    tu = t(1:end-1);

    figure('Name', 'Analiza nominalna: DP vs baseline', 'NumberTitle', 'off', ...
           'Position', [100 80 780 860]);

    %% =========================================================
    %  1. PREDKOSC
    %  =========================================================

    subplot(4,1,1);

    plot(t, log_dp.w_true, 'LineWidth', 1.8); hold on;
    if nargin > 2 && ~isempty(log_base)
        plot(t, log_base.w_true, '--', 'LineWidth', 1.5);
    end
    yline(p.omega_target, ':k', '\omega_{target}', 'LineWidth', 1.1);

    title(sprintf('Trajektoria predkosci (\\lambda_E=%g, \\beta=%g, \\alpha=%g)', ...
        p.lambda_E, p.beta, p.alpha_terminal));
    ylabel('\omega [rad/s]');
    legend_entries('DP', nargin > 2 && ~isempty(log_base));
    grid on;

    %% =========================================================
    %  2. STEROWANIE
    %  =========================================================

    subplot(4,1,2);

    stairs(tu, log_dp.U, 'LineWidth', 1.8); hold on;
    if nargin > 2 && ~isempty(log_base)
        stairs(tu, log_base.U, '--', 'LineWidth', 1.5);
    end
    yline(p.Umax, ':k', 'U_{max}');

    title('Napiecie sterujace (zmienna decyzyjna DP)');
    ylabel('u [V]');
    grid on;

    %% =========================================================
    %  3. PRAD
    %  =========================================================

    subplot(4,1,3);

    plot(t, log_dp.i_true, 'LineWidth', 1.8); hold on;
    if nargin > 2 && ~isempty(log_base)
        plot(t, log_base.i_true, '--', 'LineWidth', 1.5);
    end
    yline(p.Imax, ':k', 'I_{max}');

    title('Prad twornika (ograniczenie stanu w DP)');
    ylabel('i_a [A]');
    grid on;

    %% =========================================================
    %  4. ENERGIA
    %  =========================================================

    subplot(4,1,4);

    plot(t, log_dp.E, 'LineWidth', 1.8); hold on;
    if nargin > 2 && ~isempty(log_base)
        plot(t, log_base.E, '--', 'LineWidth', 1.5);
        txt = sprintf('DP: %.2f J | baseline: %.2f J (%.1f%%)', ...
            log_dp.E_total, log_base.E_total, ...
            100*(log_dp.E_total - log_base.E_total)/max(1e-9, log_base.E_total));
    else
        txt = sprintf('DP: %.2f J', log_dp.E_total);
    end

    title(['Skumulowana energia elektryczna  -  ' txt]);
    ylabel('E [J]'); xlabel('Czas [s]');
    grid on;

end

%% =============================================================
function legend_entries(main_name, has_base)
    if has_base
        legend(main_name, 'Baseline (stale napiecie)', 'Cel', ...
            'Location', 'best');
    else
        legend(main_name, 'Cel', 'Location', 'best');
    end
end
