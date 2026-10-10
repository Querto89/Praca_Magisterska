function metrics = calculate_metrics(p, result)
% CALCULATE_METRICS
% Oblicza wskazniki jakosci sterowania silnikiem DC.
%
% Wejscia:
%   p      - struktura parametrow z params.m
%   result - wyniki symulacji
%
% Wymagane pola result:
%   t, omega, i_a, u, E
%
% Wyjscie:
%   metrics - struktura wskaznikow jakosci

%% ============================================================
%  PRZYGOTOWANIE DANYCH
%  ============================================================

t     = result.t(:);
omega = result.omega(:);
i_a   = result.i_a(:);
u     = result.u(:);
E     = result.E(:);

omega_ref = p.reference.omega;

% Wszystkie sygnaly musza miec wspolna os czasu.

n = numel(t);

if any([numel(omega), numel(i_a), numel(u), numel(E)] ~= n)
    error('calculate_metrics: Niezgodne dlugosci sygnalow.');
end

if n < 2 || any(diff(t) <= 0)
    error('calculate_metrics: Czas musi byc scisle rosnacy.');
end

if any(~isfinite([t; omega; i_a; u; E]))
    error('calculate_metrics: Sygnaly zawieraja NaN lub Inf.');
end

%% ============================================================
%  BLAD REGULACJI
%  ============================================================

e = omega_ref - omega;

metrics.omega_ref = omega_ref;

metrics.omega_final = omega(end);
metrics.error_final = e(end);

%% ============================================================
%  CALKOWE WSKAZNIKI JAKOSCI
%
%  IAE = integral |e(t)| dt
%  ISE = integral e(t)^2 dt
%  ============================================================

metrics.IAE = trapz(t, abs(e));
metrics.ISE = trapz(t, e.^2);

%% ============================================================
%  PRZEREGULOWANIE
%  ============================================================

metrics.omega_max = max(omega);
metrics.omega_min = min(omega);

if omega_ref > 0
    metrics.overshoot_percent = ...
        max(0, (metrics.omega_max - omega_ref) ...
        / omega_ref * 100);
else
    metrics.overshoot_percent = NaN;
end

%% ============================================================
%  CZAS REGULACJI - PASMO +/- 2%
%
%  Czas regulacji jest pierwsza chwila, od ktorej
%  predkosc pozostaje w zadanym pasmie az do konca.
%  ============================================================

tolerance = 0.02 * abs(omega_ref);

outside = abs(e) > tolerance;

last_outside = find(outside, 1, 'last');

if isempty(last_outside)

    metrics.settling_time_2pct = t(1);

elseif last_outside == n

    % Uklad nie osiagnal trwale pasma do konca symulacji.
    metrics.settling_time_2pct = NaN;

else

    % Interpolacja liniowa miedzy ostatnia probka poza
    % pasmem a nastepna probka wewnatrz pasma.

    k = last_outside;

    a1 = abs(e(k));
    a2 = abs(e(k+1));

    if a1 ~= a2

        alpha = (tolerance - a1) / (a2 - a1);
        alpha = max(0, min(1, alpha));

        metrics.settling_time_2pct = ...
            t(k) + alpha * (t(k+1) - t(k));

    else

        metrics.settling_time_2pct = t(k+1);

    end

end

%% ============================================================
%  PRAD I NAPIECIE
%  ============================================================

metrics.i_max = max(i_a);
metrics.i_min = min(i_a);

metrics.u_max = max(u);
metrics.u_min = min(u);

%% ============================================================
%  ENERGIA ELEKTRYCZNA
%  ============================================================

% Korzystamy z energii obliczonej przez funkcje
% symulacyjna, zamiast ponownie ja calkowac.

metrics.energy_J = E(end) - E(1);

%% ============================================================
%  WERYFIKACJA OGRANICZEN
%  ============================================================

tol = 1e-6;

metrics.current_limit_exceeded = ...
    any(i_a > p.limits.Imax + tol) || ...
    any(i_a < p.limits.Imin - tol);

metrics.voltage_limit_exceeded = ...
    any(u > p.limits.Umax + tol) || ...
    any(u < p.limits.Umin - tol);

metrics.speed_limit_exceeded = ...
    any(omega > p.limits.omega_max + tol) || ...
    any(omega < p.limits.omega_min - tol);

%% ============================================================
%  INFORMACJE DODATKOWE
%  ============================================================

metrics.simulation_time = t(end) - t(1);

end