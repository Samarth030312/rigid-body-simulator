function res = simulate(p)
%SIMULATE  Simulate the planar 3R manipulator; standard result struct.
%
%   res = rbsim.robot.simulate(p)
%
%   Result
%     t, x (6xN: [q; qd] in rad, rad/s), params, solver
%     diag.tau (3xN applied actuator torque, N m),.kinetic,.potential,.energy,
%.power (W),.work (actuator + damper work, integrated with the state, J),
%.energyBalance
%          (E(t) - E(0) - work(t), J; ~0 when the dynamics are consistent),
%.tool (3xN),.toolSpeed,.qRef (3xN, trajectory mode)
%     summary.maxEnergyBalanceError,.finalToolPosition,.trackingErrorMax
    p = rbsim.robot.validate(p);
    x0 = [deg2rad(p.q0(:)); deg2rad(p.qd0(:))];
    qStart = x0(1:3); qEnd = deg2rad(p.control.qTargetDeg(:));
    ref = @(t) rbsim.robot.reference(t, qStart, qEnd, p.control.duration);
    % Internally the actuator + damper work W is integrated as a 7th state
    % (dW/dt = tauNet . qd, the net mechanical power) so the energy balance is
    % checked to integrator accuracy; the public state stays x = [q; qd] (CONVENTIONS 6.3).
    sol = rbsim.solver.rk4(@(t, z) augmented(t, z, p, ref), [0 p.tFinal], [x0; 0], p.dt, ...
        OutputEvery=p.outputEvery);
    res.t = sol.t; res.x = sol.x(1:6, :); res.params = p; res.solver = sol.solver;
    work = sol.x(7, :);
    N = numel(sol.t);
    d.tau = zeros(3, N); d.kinetic = zeros(1, N); d.potential = zeros(1, N);
    d.tool = zeros(3, N); d.toolSpeed = zeros(1, N); d.qRef = nan(3, N);
    d.power = zeros(1, N);
    for k = 1:N
        xk = res.x(:, k);
        d.tau(:, k) = rbsim.robot.jointTorque(sol.t(k), xk, p, ref);
        [d.kinetic(k), d.potential(k)] = rbsim.robot.energy(xk, p);
        fk = rbsim.robot.forwardKinematics(xk(1:3), p);
        d.tool(:, k) = fk.tool;
        v0 = rbsim.robot.toolKinematics(xk(1:3), xk(4:6), zeros(3,1), p);
        d.toolSpeed(k) = norm(v0);
        d.power(k) = (d.tau(:, k) - p.damping(:) .* xk(4:6)).' * xk(4:6);   % P = tau . qd, W
        if strcmp(p.control.mode, 'trajectory'), d.qRef(:, k) = ref(sol.t(k)); end
    end
    d.energy = d.kinetic + d.potential;
    d.work = work;
    d.energyBalance = d.energy - d.energy(1) - d.work;
    res.diag = d;
    res.summary.maxEnergyBalanceError = max(abs(d.energyBalance));
    res.summary.finalToolPosition = d.tool(:, end);
    res.summary.trackingErrorMax = NaN;
    if strcmp(p.control.mode, 'trajectory')
        res.summary.trackingErrorMax = max(vecnorm(res.x(1:3, :) - d.qRef));
    end
end

function zd = augmented(t, z, p, ref)
    [xd, tauNet] = rbsim.robot.dynamics(t, z(1:6), p, ref);
    zd = [xd; tauNet.' * z(4:6)];
end
