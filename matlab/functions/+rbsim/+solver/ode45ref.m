function sol = ode45ref(f, tOut, x0, opts)
%ODE45REF Reference integration with MATLAB ode45 on a given output grid.
%
%   sol = rbsim.solver.ode45ref(f, tOut, x0)
%   sol = rbsim.solver.ode45ref(f, tOut, x0, RelTol=..., AbsTol=...)
%
%   Inputs
%     f: xdot = f(t, x).
%     tOut: 1 x N output times (increasing), s.
%     x0: n x 1 initial state.
%   Output struct with the same fields as INTEGRATERK4 (t, x, solver).
%
%   Used only for validation/comparison studies (ADR-0009). No SO(3) projection is
%   applied, so drift of R'R from I is a measured diagnostic here.
arguments
    f (1,1) function_handle
    tOut (1,:) double {mustBeFinite}
    x0 (:,1) double {mustBeFinite}
    opts.RelTol (1,1) double {mustBePositive} = 1e-10
    opts.AbsTol (1,1) double {mustBePositive} = 1e-12
end
o = odeset('RelTol', opts.RelTol, 'AbsTol', opts.AbsTol);
tic;
[tt, xx] = ode45(f, tOut, x0, o);
sol.t = tt.';
sol.x = xx.';
sol.solver = struct('name', 'ode45', 'RelTol', opts.RelTol, 'AbsTol', opts.AbsTol, ...
    'wallTime', toc);
end
