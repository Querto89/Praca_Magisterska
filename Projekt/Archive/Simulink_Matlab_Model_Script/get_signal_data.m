function data = get_signal_data(sim_out, name)
%GET_SIGNAL_DATA  Odporny odczyt sygnalu z wyniku sim().
%
%  Obsluguje rozne formaty zwracane przez Simulinka w zaleznosci od wersji
%  i ustawien: Simulink.SimulationOutput, timeseries, struct with time,
%  surowa macierz.

    data = [];

    obj = [];

    % 1) Dostep przez pole / property
    try
        if isprop(sim_out, name) || isfield(sim_out, name)
            obj = sim_out.(name);
        end
    catch
    end

    % 2) Dostep przez find() (Dataset / SimulationOutput)
    if isempty(obj)
        try
            obj = sim_out.find(name);
        catch
        end
    end

    % 3) Dostep przez get()
    if isempty(obj)
        try
            obj = get(sim_out, name);
        catch
        end
    end

    if isempty(obj)
        return;
    end

    % Rozpakowanie wartosci
    if isa(obj, 'timeseries')
        data = obj.Data;
    elseif isobject(obj) && isprop(obj, 'Values') && isa(obj.Values, 'timeseries')
        data = obj.Values.Data;
    elseif isobject(obj) && isprop(obj, 'Data')
        data = obj.Data;
    elseif isstruct(obj) && isfield(obj, 'signals')
        data = obj.signals.values;
    elseif isnumeric(obj)
        data = obj;
    end

    data = data(:);

end
