function plant = plant_matlab(p)
%PLANT_MATLAB  Obiekt "rzeczywisty" calkowany drobnym krokiem w MATLAB-ie.
%
%  Uzywa parametrow PRAWDZIWYCH (p.plant), nie nominalnych.
%  Krok calkowania h = dt/substeps, wiec bledy numeryczne sa pomijalne
%  w porownaniu z niedopasowaniem parametrow.
%
%  Sluzy jako:
%    - szybki zamiennik Simulinka w kampaniach Monte Carlo
%      (setki przebiegow - Simulink bylby za wolny),
%    - wzorzec do weryfikacji poprawnosci modelu Simulink (verify_plant.m).

    [Adh, Bdh] = motor_model(p.plant, p.plant.h);

    % Wplyw momentu obciazenia TL (stalego) na krok h
    if p.plant.TL ~= 0
        A  = [ -p.plant.Ra/p.plant.La, -p.plant.Ke/p.plant.La;
                p.plant.Kt/p.plant.J,  -p.plant.B/p.plant.J ];
        Bt = [0; -1/p.plant.J];
        Bdt = A \ ((Adh - eye(2)) * Bt);
    else
        Bdt = [0; 0];
    end

    plant.name  = 'MATLAB (drobny krok)';
    plant.init  = @() [p.i_0; p.omega_0];
    plant.step  = @step;
    plant.close = @() [];

    function x_next = step(x, U)
        x_next = x;
        for s = 1:p.plant.substeps
            x_next = Adh * x_next + Bdh * U + Bdt * p.plant.TL;
        end
    end

end
