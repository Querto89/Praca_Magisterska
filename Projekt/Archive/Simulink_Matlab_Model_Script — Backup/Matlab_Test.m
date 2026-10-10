clear; clc;

% 1. Wczytanie parametrów
p = params();
disp('Parametry załadowane pomyślnie.');

% 2. Uruchomienie algorytmu DP
disp('Trwa obliczanie algorytmu DP...');
tic;
Uopt = dp_solver(p);
toc;
disp('Algorytm DP zakończył działanie.');

% Sprawdzenie poprawności wymiarów macierzy wyjściowej
expected_size = [length(p.omega_grid), p.N];
assert(isequal(size(Uopt), expected_size), 'Błąd: Wymiary Uopt są niezgodne z oczekiwanymi!');
disp('Wymiary macierzy sterowania Uopt są poprawne.');

% 3. Symulacja offline trajektorii w czystym MATLAB-ie (zamiast Simulinka)
curr_w = p.omega_0;
w_traj = zeros(p.N + 1, 1);
w_traj(1) = curr_w;
T_traj = zeros(p.N, 1);
E_traj = zeros(p.N + 1, 1);

for k = 1:p.N
    % Znajdź najbliższy indeks w siatce stanu dla obecnej prędkości
    [~, idx] = min(abs(p.omega_grid - curr_w));
    
    % Pobierz optymalny moment z tabeli polityki DP
    T = Uopt(idx, k);
    T_traj(k) = T;
    
    % Zaktualizuj stan układu (jawna metoda Eulera)
    curr_w = curr_w + p.dt * (1/p.J) * (T - p.B * curr_w);
    w_traj(k+1) = curr_w;
    
    % Oblicz skumulowaną energię elektryczną (straty w miedzi + praca)
    I_val = T / p.Kt;
    power_loss = p.R_ohm * (I_val^2);
    E_traj(k+1) = E_traj(k) + power_loss * p.dt;
end

% 4. Wykres weryfikacyjny
time_vec = (0:p.N) * p.dt;
figure('Name', 'Test MATLAB-only (DP)', 'NumberTitle', 'off');

subplot(2,1,1);
plot(time_vec, w_traj, '-o', 'LineWidth', 1.5, 'MarkerSize', 4);
hold on;
yline(p.omega_target, '--r', 'Cel (\omega_{target})', 'LineWidth', 1.2);
title('Weryfikacja DP: Trajektoria prędkości wirnika');
ylabel('\omega [rad/s]');
grid on;
legend('Trajektoria DP', 'Cel', 'Location', 'best');

subplot(2,1,2);
stairs(time_vec(1:end-1), T_traj, 'LineWidth', 1.5);
title('Optymalny moment sterujący T(t)');
ylabel('T [Nm]');
xlabel('Czas [s]');
grid on;

disp('Test zakończony sukcesem! Jeśli wykres ma sens fizyczny, model w MATLABie działa poprawnie.');