function [Ad, Bd, A, Bu] = motor_model(q, h)
%MOTOR_MODEL  Model silnika DC w przestrzeni stanow + dyskretyzacja ZOH.
%
%  x = [i_a; omega]
%
%  di/dt     = (u - Ra*i - Ke*omega) / La
%  domega/dt = (Kt*i - B*omega - TL)  / J
%
%  Wejscie:
%    q - struktura z polami Ra, La, Kt, Ke, J, B  (p.nom lub p.plant)
%    h - krok dyskretyzacji [s]
%
%  Wyjscie:
%    Ad, Bd - macierze dyskretne (dokladne ZOH)
%    A, Bu  - macierze ciagle
%
%  UWAGA: jedno zrodlo prawdy dla rownan. DP, Digital Twin i obiekt
%  korzystaja z TEJ SAMEJ struktury rownan - roznia sie wylacznie
%  wartosciami parametrow i sposobem calkowania.

    A = [ -q.Ra/q.La,   -q.Ke/q.La;
           q.Kt/q.J,    -q.B/q.J ];

    Bu = [1/q.La;
          0];

    Ad = expm(A * h);

    % Bd = integral_0^h exp(A*t) dt * Bu   (A nieosobliwa dla tego silnika)
    if rcond(A) > 1e-12
        Bd = A \ ((Ad - eye(2)) * Bu);
    else
        % Wariant odporny: rozszerzona macierz (metoda Van Loana)
        M  = expm([A, Bu; zeros(1,3)] * h);
        Bd = M(1:2, 3);
    end

end
