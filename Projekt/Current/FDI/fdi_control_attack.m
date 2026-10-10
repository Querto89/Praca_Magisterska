
function attack = fdi_control_attack(t, u, p)
% FDI_CONTROL_ATTACK
% Manipulacja sygnalem napiecia wyznaczonym przez DP.
%
% Wejscia:
%   t - chwile rozpoczecia przedzialow sterowania [s]
%   u - optymalna sekwencja napiec DP [V]
%   p - parametry projektu
%
% Wyjscia:
%   attack.t
%   attack.u_commanded
%   attack.u_actual
%   attack.a
%   attack.active
%   attack.delta_u
%
% Obslugiwane ataki:
%   bias, ramp, pulse

%% 1. PRZYGOTOWANIE DANYCH

t = t(:);
u = u(:);

assert(numel(t) == numel(u), ...
    'Wektory t i u musza miec taka sama dlugosc.');

assert(all(isfinite(t)) && all(isfinite(u)), ...
    'Dane wejsciowe musza byc skonczone.');

assert(all(diff(t) > 0), ...
    'Wektor czasu musi byc scisle rosnacy.');

%% 2. INICJALIZACJA

a = zeros(size(u));
active = false(size(u));

%% 3. GENEROWANIE ATAKU

if p.fdi.enabled

    t_start = p.fdi.start_time;
    magnitude = p.fdi.magnitude;

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

            a(active) = magnitude .* ...
                min(elapsed / duration, 1);

        case 'pulse'

            duration = p.fdi.pulse_duration;

            assert(isfinite(duration) && duration > 0, ...
                'pulse_duration musi byc dodatni.');

            active = ...
                t >= t_start & ...
                t < t_start + duration;

            a(active) = magnitude;

        otherwise

            error('Nieznany typ ataku: %s', ...
                char(p.fdi.type));

    end

end

%% 4. NAPIECIE PO ATAKU

u_actual = min( ...
    max(u + a, p.limits.Umin), ...
    p.limits.Umax);

%% 5. ZAPIS WYNIKOW

attack.t = t;
attack.u_commanded = u;
attack.u_actual = u_actual;
attack.a = a;
attack.active = active;

% Rzeczywista zmiana napiecia po uwzglednieniu nasycenia
attack.delta_u = u_actual - u;

end