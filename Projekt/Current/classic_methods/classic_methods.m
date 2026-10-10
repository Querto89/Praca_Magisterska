function result = classic_methods(p, method)
% CLASSIC_METHODS
% Wybiera i uruchamia klasyczna metode sterowania.
%
% result = classic_methods(p)
% result = classic_methods(p, 'cascade_pi')
% result = classic_methods(p, 'state_feedback')

if nargin < 2 || isempty(method)
    method = 'cascade_pi';
end

switch lower(char(method))

    case 'cascade_pi'
        result = run_cascade_control(p);

    case 'state_feedback'
        result = run_state_feedback(p);

    otherwise
        error('Nieznana metoda sterowania: %s', ...
            char(method));

end

end