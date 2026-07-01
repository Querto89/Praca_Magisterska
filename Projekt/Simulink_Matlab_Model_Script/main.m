clear; clc;
p = params();
Uopt = dp_solver(p);
[w, T, E, t] = run_simulink(p, Uopt, p.T_base_val);
plots(p, w.dp(:)', T.dp(1:end-1)', E.dp(:)', w.base(:)', T.base(1:end-1)', E.base(:)');