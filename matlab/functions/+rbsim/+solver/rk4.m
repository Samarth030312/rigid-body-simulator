function sol = rk4(f, tspan, x0, dt, opts)
%RK4  Fixed-step classical 4th-order Runge-Kutta integration.
%
%   sol = rbsim.solver.rk4(f, tspan, x0, dt)
%   sol = rbsim.solver.rk4(f, tspan, x0, dt, Name=Value)
%
%   Inputs
%     f: function handle xdot = f(t, x), x an n x 1 column.
%     tspan: [t0 tf], s (tf > t0).
%     x0: n x 1 initial state.
%     dt: step size, s. The last step is shortened to land exactly on tf.
%   Options
%     PostStep: handle [x, corr] = PostStep(x) applied after every step, e.g.
%                   rotation-matrix re-orthonormalisation. corr = size of the
%                   correction (logged). Default: none.
%     OutputEvery: store every k-th step (default 1). First and last are always kept.
%     StopFcn: handle tf = StopFcn(t, x); integration stops after the first
%                   step where it returns true (e.g. dice at rest). Default: none.
%   Output (struct)
%     t: 1 x N stored times, s
%     x: n x N stored states
%     solver: struct(name, dt, steps, wallTime, maxCorrection, stoppedEarly)
%
%   Method: k1 = f(t,x), k2 = f(t+h/2, x+h k1/2), k3 = f(t+h/2, x+h k2/2),
%   k4 = f(t+h, x+h k3), x+ = x + h(k1 + 2k2 + 2k3 + k4)/6.
%   The classical four-stage Runge-Kutta rule, chosen in ADR-0009 for a fixed,
%   reproducible step and a uniform output grid for animation.
    arguments
        f (1,1) function_handle
        tspan (1,2) double {mustBeFinite}
        x0 (:,1) double {mustBeFinite}
        dt (1,1) double {mustBePositive, mustBeFinite}
        opts.PostStep = []
        opts.OutputEvery (1,1) double {mustBeInteger, mustBePositive} = 1
        opts.StopFcn = []
    end
    t0 = tspan(1); tf = tspan(2);
    if tf <= t0
        error('rbsim:solver:badSpan', 'tspan(2) must exceed tspan(1).');
    end
    nSteps = ceil((tf - t0) / dt - 1e-9);
    nStore = floor(nSteps / opts.OutputEvery) + 2;
    T = zeros(1, nStore);
    X = zeros(numel(x0), nStore);
    T(1) = t0; X(:, 1) = x0;
    k = 1;
    x = x0; t = t0;
    maxCorr = 0;
    stopped = false;
    tic;
    for i = 1:nSteps
        h = min(dt, tf - t);
        k1 = f(t, x);
        k2 = f(t + h/2, x + (h/2) * k1);
        k3 = f(t + h/2, x + (h/2) * k2);
        k4 = f(t + h, x + h * k3);
        x = x + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
        t = t0 + (i < nSteps) * i * dt + (i == nSteps) * (tf - t0);
        if ~isempty(opts.PostStep)
            [x, corr] = opts.PostStep(x);
            maxCorr = max(maxCorr, corr);
        end
        if ~all(isfinite(x))
            error('rbsim:solver:nonFinite', 'State became non-finite at t = %g s.', t);
        end
        if ~isempty(opts.StopFcn) && opts.StopFcn(t, x)
            stopped = true;
        end
        if mod(i, opts.OutputEvery) == 0 || i == nSteps || stopped
            k = k + 1;
            T(k) = t; X(:, k) = x;
        end
        if stopped
            break
        end
    end
    sol.t = T(1:k);
    sol.x = X(:, 1:k);
    sol.solver = struct('name', 'RK4 (fixed step)', 'dt', dt, 'steps', i, ...
        'wallTime', toc, 'maxCorrection', maxCorr, 'stoppedEarly', stopped);
end
