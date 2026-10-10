
function detection = fdi_detector(t, omega_measured, omega_dt, p)
% FDI_DETECTOR
% Wykrywa anomalie pomiaru predkosci na podstawie
% porownania z cyfrowym blizniakiem.
%
% Wejscia:
%   t              - wektor czasu [s]
%   omega_measured - predkosc zmierzona [rad/s]
%   omega_dt       - predkosc przewidywana przez DT [rad/s]
%   p              - parametry projektu
%
% Wyjscia:
%   detection.t
%   detection.residual
%   detection.alarm
%   detection.detected
%   detection.first_detection_time

%% 1. Przygotowanie danych

t = t(:);
omega_measured = omega_measured(:);
omega_dt = omega_dt(:);

assert(numel(t) == numel(omega_measured) && ...
       numel(t) == numel(omega_dt), ...
       'Wszystkie wektory musza miec taka sama dlugosc.');

assert(all(isfinite(t)) && ...
       all(isfinite(omega_measured)) && ...
       all(isfinite(omega_dt)), ...
       'Dane musza byc skonczone.');

assert(all(diff(t) > 0), ...
       'Wektor czasu musi byc scisle rosnacy.');

assert(isfinite(p.detection.threshold) && ...
       p.detection.threshold >= 0, ...
       'Prog detekcji musi byc nieujemny.');

%% 2. Residuum

residual = abs(omega_measured - omega_dt);

%% 3. Decyzja detektora

alarm = residual > p.detection.threshold;

first_index = find(alarm, 1, 'first');

%% 4. Wyniki

detection.t = t;
detection.residual = residual;
detection.alarm = alarm;
detection.detected = any(alarm);

if isempty(first_index)
    detection.first_detection_time = NaN;
else
    detection.first_detection_time = t(first_index);
end

end