function res = simulate(p)
%SIMULATE Simulate a thrown dice; returns the standard result struct.
%
%   res = rbsim.dice.simulate(p)       p from DICEDEFAULTPARAMS (possibly modified)
%
%   Pipeline (docs/architecture/architecture.md): validate -> initial state ->
%   Newton-Euler derivative -> RK4 with SO(3) projection -> diagnostics -> outcome.
%   The face outcome is read from the final simulated orientation; nothing about
%   the result is prescribed.
%
%   Result fields
%     t, x, params, solver                       (see architecture.md)
%     diag.kinetic,.potential,.spring,.energy  J (energy incl. contact springs)
%     diag.contactCount   vertices in contact
%     diag.orthError      ||R'R - I||_F
%     diag.topFace,.topAlignment
%     diag.H_A            3xN angular momentum about C in {A}
%     summary.finalFace,.finalAlignment,.settled,.settleTime,.bounces,
%.maxHeight,.energyDissipated
p = rbsim.dice.validate(p);
G = rbsim.dice.geometry(p.edge);
x0 = rbsim.dice.initialState(p);
I = rbsim.body.inertiaCube(p.mass, p.edge);
f = @(t, x) rbsim.dice.dynamics(t, x, p, G, I);
stopFcn = [];
if p.contact.enabled && p.stopWhenSettled
    det = rbsim.dice.RestDetector(p, G, 0.1);
    stopFcn = @(t, x) det.check(t, x);
end
sol = rbsim.solver.rk4(f, [0 p.tFinal], x0, p.dt, ...
    PostStep=@rbsim.body.projectState, OutputEvery=p.outputEvery, StopFcn=stopFcn);
res.t = sol.t;
res.x = sol.x;
res.params = p;
res.solver = sol.solver;
res.geometry = G;
N = numel(sol.t);
d = struct('kinetic', zeros(1,N), 'potential', zeros(1,N), 'spring', zeros(1,N), ...
    'energy', zeros(1,N), 'contactCount', zeros(1,N), 'orthError', zeros(1,N), ...
    'topFace', zeros(1,N), 'topAlignment', zeros(1,N), 'H_A', zeros(3,N), ...
    'speed', zeros(1,N), 'spin', zeros(1,N));
for k = 1:N
    xk = sol.x(:, k);
    [T, V, H] = rbsim.body.energy(xk, p.mass, I, p.g);
    [~, ~, info] = rbsim.dice.contactLoads(xk, p, G);
    R = reshape(xk(7:15), 3, 3);
    d.kinetic(k) = T; d.potential(k) = V; d.spring(k) = info.springEnergy;
    d.energy(k) = T + V + info.springEnergy;
    d.contactCount(k) = info.n;
    d.orthError(k) = rbsim.math.rotationError(R);
    [d.topFace(k), d.topAlignment(k)] = rbsim.dice.topFace(R, G);
    d.H_A(:, k) = H;
    d.speed(k) = norm(xk(4:6));
    d.spin(k) = norm(xk(16:18));
end
res.diag = d;
res.summary = diceSummary(res);
end

function s = diceSummary(res)
d = res.diag;
s.finalFace = d.topFace(end);
s.finalAlignment = d.topAlignment(end);
% "Settled": same criterion as rbsim.dice.RestDetector (single source of truth).
atRest = arrayfun(@(k) rbsim.dice.RestDetector.isAtRest(res.x(:, k), res.params, ...
    res.geometry), 1:numel(res.t));
s.settled = atRest(end);
s.settleTime = NaN;
if s.settled
    k = find(~atRest, 1, 'last');
    if isempty(k), k = 0; end
    s.settleTime = res.t(min(k + 1, numel(res.t)));
end
inContact = d.contactCount > 0;
s.bounces = nnz(diff(inContact) == 1);   % new contact episodes
s.maxHeight = max(res.x(3, :));
s.energyDissipated = d.energy(1) - d.energy(end);
end
