function result = run_matlab(p, u)
% RUN_MATLAB
% Symulacja modelu matematycznego silnika pradu stalego.
%
% Model pelni role Digital Twin.
%
% Wejscia:
%   p - struktura parametrow z params.m
%   u - wektor napiecia twornika [V]
%
% Wyjscie:
%   result.t      - czas [s]
%   result.u      - napiecie twornika [V]
%   result.i_a    - prad twornika [A]
%   result.M_e    - moment elektromagnetyczny [N*m]
%   result.omega  - predkosc katowa [rad/s]
%   result.P_el   - moc elektryczna [W]
%   result.E      - energia pobrana [J]


%% ============================================================
%  PARAMETRY MODELU
%  ============================================================

Ra = p.motor.Ra;
La = p.motor.La;

Kt = p.motor.Kt;
Ke = p.motor.Ke;

J  = p.motor.J;
B  = p.motor.B;

TL = p.motor.TL;

dt = p.sim.dt;
N  = p.sim.N;

energy_substeps = p.sim.energy_substeps;


%% ============================================================
%  SPRAWDZENIE SYGNALU WEJSCIOWEGO
%  ============================================================

u = u(:);

if length(u) ~= N

    error( ...
        ['run_matlab: wektor u musi zawierac dokladnie ', ...
         num2str(N), ' probek.']);

end


%% ============================================================
%  OGRANICZENIE NAPIECIA
%  ============================================================

u = max( ...
    p.limits.Umin, ...
    min(u, p.limits.Umax));


%% ============================================================
%  WEKTOR CZASU
%  ============================================================

t = (0:N)' * dt;


%% ============================================================
%  INICJALIZACJA ZMIENNYCH
%  ============================================================

i_a = zeros(N+1,1);

omega = zeros(N+1,1);

M_e = zeros(N+1,1);

P_el = zeros(N+1,1);

E = zeros(N+1,1);


%% ============================================================
%  WARUNKI POCZATKOWE
%  ============================================================

i_a(1) = p.initial.i_a;

omega(1) = p.initial.omega;

M_e(1) = Kt * i_a(1);

P_el(1) = u(1) * i_a(1);


%% ============================================================
%  MODEL CIAGLY W POSTACI STANU
%
%  x = [i_a; omega]
%
%  dx/dt = A*x + Bu*u + BT*TL
%  ============================================================

A = [ -Ra/La,  -Ke/La;
       Kt/J,    -B/J ];

Bu = [1/La;
      0];

BT = [0;
     -1/J];


%% ============================================================
%  DOKLADNA DYSKRETYZACJA ZOH
%
%  x(k+1) =
%
%       Ad*x(k)
%       + Bdu*u(k)
%       + BdT*TL
%  ============================================================

B_all = [Bu BT];

M = [A, B_all;
     zeros(2,4)];

Md = expm(M * dt);

Ad  = Md(1:2,1:2);

Bdu = Md(1:2,3);

BdT = Md(1:2,4);


%% ============================================================
%  MACIERZE DLA PODKROKOW ENERGII
%
%  Energia:
%
%       E_k = integral u_k*i_a(t) dt
%
%  Sterowanie u_k jest stale w pojedynczym kroku (ZOH),
%  natomiast prad zmienia sie zgodnie z dynamika silnika.
%  ============================================================

tau = linspace( ...
    0, ...
    dt, ...
    energy_substeps + 1);


Ad_tau = zeros( ...
    2, ...
    2, ...
    energy_substeps + 1);

Bdu_tau = zeros( ...
    2, ...
    energy_substeps + 1);

BdT_tau = zeros( ...
    2, ...
    energy_substeps + 1);


for m = 1:(energy_substeps + 1)

    M_tau = expm(M * tau(m));

    Ad_tau(:,:,m) = ...
        M_tau(1:2,1:2);

    Bdu_tau(:,m) = ...
        M_tau(1:2,3);

    BdT_tau(:,m) = ...
        M_tau(1:2,4);

end


%% ============================================================
%  SYMULACJA DIGITAL TWIN
%  ============================================================

for k = 1:N

    %% --------------------------------------------------------
    %  AKTUALNY STAN
    %  --------------------------------------------------------

    x = [i_a(k);
         omega(k)];


    %% --------------------------------------------------------
    %  NASTEPNY STAN
    %  --------------------------------------------------------

    x_next = ...
        Ad*x ...
        + Bdu*u(k) ...
        + BdT*TL;


    %% --------------------------------------------------------
    %  ZAPIS STANOW
    %  --------------------------------------------------------

    i_a(k+1) = x_next(1);

    omega(k+1) = x_next(2);


    %% --------------------------------------------------------
    %  MOMENT ELEKTROMAGNETYCZNY
    %  --------------------------------------------------------

    M_e(k+1) = ...
        Kt * i_a(k+1);


    %% --------------------------------------------------------
    %  PRZEBIEG PRADU WEWNATRZ KROKU
    %  --------------------------------------------------------

    i_internal = zeros( ...
        energy_substeps + 1, ...
        1);


    for m = 1:(energy_substeps + 1)

        x_tau = ...
            Ad_tau(:,:,m)*x ...
            + Bdu_tau(:,m)*u(k) ...
            + BdT_tau(:,m)*TL;

        i_internal(m) = ...
            x_tau(1);

    end


    %% --------------------------------------------------------
    %  ENERGIA ELEKTRYCZNA W KROKU
    %
    %  E_k =
    %
    %       integral u_k*i_a(t) dt
    %
    %  Przy ZOH:
    %
    %  E_k =
    %
    %       u_k * integral i_a(t) dt
    %  --------------------------------------------------------

    E_step = ...
        u(k) * trapz(tau, i_internal);


    %% --------------------------------------------------------
    %  ENERGIA SKUMULOWANA
    %  --------------------------------------------------------

    E(k+1) = ...
        E(k) + E_step;


    %% --------------------------------------------------------
    %  MOC ELEKTRYCZNA
    %
    %  P_el(t_k) = u_k * i_a(t_k)
    %  --------------------------------------------------------

    P_el(k) = ...
        u(k) * i_a(k);

end


%% ============================================================
%  OSTATNIA PROBKA MOCY
%  ============================================================

P_el(end) = ...
    u(end) * i_a(end);


%% ============================================================
%  ZAPIS WYNIKOW
%  ============================================================

result.t = t;


% Napiecie posiada N przedzialow sterowania.
% Ostatnia wartosc zostaje powtorzona w chwili Tsim
% w celu zachowania wspolnego wymiaru wektorow wynikowych.

result.u = [u;
            u(end)];


result.i_a = i_a;

result.M_e = M_e;

result.omega = omega;

result.P_el = P_el;

result.E = E;

end