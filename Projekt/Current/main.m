clear;
clc;
close all;

p = params();

u = 8 * ones(p.sim.N, 1);

[t, i_a, omega, M_e, E] = run_matlab(p, u);

figure;

subplot(3,1,1);
plot(t, i_a, 'LineWidth', 1.5);
grid on;
ylabel('i_a [A]');
title('Prad twornika');

subplot(3,1,2);
plot(t, omega, 'LineWidth', 1.5);
grid on;
ylabel('\omega [rad/s]');
title('Predkosc katowa');

subplot(3,1,3);
plot(t, M_e, 'LineWidth', 1.5);
grid on;
ylabel('M_e [N m]');
xlabel('t [s]');
title('Moment elektromagnetyczny');

fprintf('Predkosc koncowa: %.4f rad/s\n', omega(end));
fprintf('Prad koncowy:     %.4f A\n', i_a(end));
fprintf('Moment koncowy:   %.4f N*m\n', M_e(end));
fprintf('Energia:          %.4f J\n', E(end));