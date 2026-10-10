function mc = measurement_channel(p, attack, seed)
%MEASUREMENT_CHANNEL  Tor pomiarowy: szum czujnikow + atak FDI.
%
%  MIEJSCE WSTRZYKNIECIA ATAKU
%  ---------------------------
%  Atak dziala na KANALE POMIAROWYM, nie na fizyce silnika. Silnik nadal
%  spelnia rownania rozniczkowe - falszowane jest to, co widzi regulator
%  i detektor. To jest wlasciwa definicja ataku FDI (w odroznieniu od
%  uszkodzenia fizycznego / awarii aktuatora).
%
%  TYPY ATAKOW
%    'none'    - brak
%    'bias'    - staly offset            y = y + a
%    'ramp'    - narastajacy offset      y = y + s*(t - t_atk)
%    'scaling' - skalowanie              y = g * y
%    'replay'  - powtarzanie starych probek (atak najtrudniejszy do wykrycia)
%
%  Uzycie:
%    mc = measurement_channel(p, p.attack, seed);
%    [y, mc] = mc.apply(mc, k, t, x_true);

    if nargin < 3 || isempty(seed)
        seed = p.sensor.seed;
    end

    mc.rng     = RandStream('mt19937ar', 'Seed', seed);
    mc.attack  = attack;
    mc.p       = p;
    mc.buffer  = zeros(2, p.N + 1);   % historia pomiarow (dla 'replay')
    mc.n       = 0;
    mc.apply   = @apply;

end

%% =============================================================
function [y, mc] = apply(mc, k, t, x_true)
%APPLY  Zwraca pomiar y = [i_meas; omega_meas] dla stanu prawdziwego x_true.

    p = mc.p;
    a = mc.attack;

    %% ---------------------------------------------------------
    %  1. SZUM CZUJNIKOW (zawsze obecny)
    %  ---------------------------------------------------------

    y = x_true(:) + [p.sensor.sigma_i * randn(mc.rng);
                     p.sensor.sigma_w * randn(mc.rng)];

    %% ---------------------------------------------------------
    %  2. ZAPIS CZYSTEGO POMIARU DO BUFORA (przed atakiem)
    %  ---------------------------------------------------------

    mc.n = k;
    mc.buffer(:, k) = y;

    %% ---------------------------------------------------------
    %  3. ATAK FDI
    %  ---------------------------------------------------------

    if strcmpi(a.type, 'none') || t < a.start
        return;
    end

    y_att = y;

    switch lower(a.type)

        case 'bias'
            delta = a.bias;
            if a.on_omega,   y_att(2) = y(2) + delta; end
            if a.on_current, y_att(1) = y(1) + delta; end

        case 'ramp'
            delta = a.slope * (t - a.start);
            if a.on_omega,   y_att(2) = y(2) + delta; end
            if a.on_current, y_att(1) = y(1) + delta; end

        case 'scaling'
            if a.on_omega,   y_att(2) = a.gain * y(2); end
            if a.on_current, y_att(1) = a.gain * y(1); end

        case 'replay'
            % Powtarza probke sprzed 'replay_lag' krokow.
            idx = max(1, k - a.replay_lag);
            if a.on_omega,   y_att(2) = mc.buffer(2, idx); end
            if a.on_current, y_att(1) = mc.buffer(1, idx); end

        otherwise
            error('measurement_channel:unknownAttack', ...
                'Nieznany typ ataku: %s', a.type);
    end

    y = y_att;

end
