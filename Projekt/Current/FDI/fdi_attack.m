function attack = fdi_attack(t, omega, p)
% FDI_ATTACK
% Generuje falszywy pomiar predkosci silnika DC.
%
% Obslugiwane rodzaje atakow:
%   bias  - stale przesuniecie od start_time
%   ramp  - liniowe narastanie do magnitude
%   pulse - przesuniecie przez okreslony czas
%
% Wejscia:
%   t      - wektor czasu [s]
%   omega  - rzeczywista predkosc [rad/s]
%   p      - parametry projektu
%
% Wyjscia:
%   attack.t
%   attack.a
%   attack.active
%   attack.omega_measured

%% 1. PRZYGOTOWANIE DANYCH

t = t(:);
omega = omega(:);

assert(numel(t) == numel(omega), ...
    'Wektory t i omega musza miec taka sama dlugosc.');

assert(all(isfinite(t)) && all(isfinite(omega)), ...
    'Dane wejsciowe musza byc skonczone.');

assert(all(diff(t) > 0), ...
    'Wektor czasu musi byc scisle rosnacy.');

%% 2. INICJALIZACJA

a = zeros(size(omega));
active = false(size(omega));

%% 3. GENEROWANIE ATAKU

if p.fdi.enabled

    t_start = p.fdi.start_time;
    magnitude = p.fdi.magnitude;

    assert(isfinite(t_start) && isfinite(magnitude), ...
        'Parametry ataku musza byc skonczone.');

    switch lower(char(p.fdi.type))

        case 'bias'

            active = t >= t_start;

            a(active) = magnitude;

        case 'ramp'

            duration = p.fdi.ramp_duration;

            assert(isfinite(duration) && duration > 0, ...
                'ramp_duration musi byc dodatni.');

            active = t >= t_start;

            elapsed = t(active) - t_start;

            fraction = min(elapsed / duration, 1);

            a(active) = magnitude .* fraction;

        case 'pulse'

            duration = p.fdi.pulse_duration;

            assert(isfinite(duration) && duration > 0, ...
                'pulse_duration musi byc dodatni.');

            active = ...
                t >= t_start & ...
                t < t_start + duration;

            a(active) = magnitude;

        otherwise

            error('Nieznany typ ataku FDI: %s', ...
                char(p.fdi.type));

    end

end

%% 4. ZAPIS WYNIKOW

attack.t = t;
attack.a = a;
attack.active = active;
attack.omega_measured = omega + a;

end