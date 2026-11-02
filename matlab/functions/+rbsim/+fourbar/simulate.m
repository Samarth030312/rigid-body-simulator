function res = simulate(p)
%SIMULATE Simulate the four-bar mechanism; standard result struct.
%
%   res = rbsim.fourbar.simulate(p)
%
%   State x = [theta1; theta1dot] (rad, rad/s), CONVENTIONS 6.4.
%   Drive modes:
%     'free'   tau = 0 (swings/rotates under gravity)       -> forward dynamics
%     'torque' tau = p.drive.torque                          -> forward dynamics
%     'speed'  theta1 = theta0 + w t (kinematic drive)       -> inverse dynamics
%   The work of the drive, load and damper is integrated with the state (as in
%   rbsim.robot.simulate) so diag.energyBalance checks the dynamics to RK4 accuracy.
%
%   Result diag fields (1xN unless noted): tau, theta2, phi1, omega (3xN),
%   alpha (3xN), reactions (struct of 2xN), energy, kinetic, potential, work,
%   energyBalance, closureResidual (max |loop-closure| of L5-6/s21, m),
%   eta1, transmissionDeg, detJ, points (struct of 2xN: A, B)
p = rbsim.fourbar.validate(p);
th0 = deg2rad(p.theta0Deg);
switch p.drive.mode
    case 'speed'
        w = deg2rad(p.drive.speedDeg);
        x0 = [th0; w];
    otherwise
        x0 = [th0; deg2rad(p.thetaDot0Deg)];
end
f = @(t, z) augmented(t, z, p);
sol = rbsim.solver.rk4(f, [0 p.tFinal], [x0; 0], p.dt, OutputEvery=p.outputEvery);
res.t = sol.t; res.x = sol.x(1:2, :); res.params = p; res.solver = sol.solver;
N = numel(res.t);
d.tau = zeros(1,N); d.theta2 = zeros(1,N); d.phi1 = zeros(1,N);
d.omega = zeros(3,N); d.alpha = zeros(3,N); d.kinetic = zeros(1,N);
d.potential = zeros(1,N); d.closureResidual = zeros(1,N); d.eta1 = zeros(1,N);
d.transmissionDeg = zeros(1,N); d.detJ = zeros(1,N); d.angleSum = zeros(1,N);
d.reactions = struct('OL', zeros(2,N), 'A', zeros(2,N), 'B', zeros(2,N), 'OR', zeros(2,N));
d.points = struct('A', zeros(2,N), 'B', zeros(2,N), 'C', zeros(2,3,N));
for k = 1:N
    [s, kin] = solveDynamics(res.x(:, k), p);
    kin = rbsim.fourbar.kinematics(res.x(1,k), res.x(2,k), s.thetaDDot1, p);
    d.tau(k) = s.tau; d.theta2(k) = kin.theta2; d.phi1(k) = kin.phi1;
    d.omega(:, k) = kin.omega.'; d.alpha(:, k) = kin.alpha.';
    [d.kinetic(k), d.potential(k)] = rbsim.fourbar.energy(res.x(1,k), res.x(2,k), p);
    d.closureResidual(k) = norm(kin.closure); d.eta1(k) = kin.eta1;
    d.angleSum(k) = kin.angleSum;
    d.transmissionDeg(k) = rad2deg(kin.transmission); d.detJ(k) = kin.detJ;
    d.reactions.OL(:,k) = s.F_OL; d.reactions.A(:,k) = s.F_A;
    d.reactions.B(:,k) = s.F_B; d.reactions.OR(:,k) = s.F_OR;
    d.points.A(:,k) = kin.A; d.points.B(:,k) = kin.B; d.points.C(:,:,k) = kin.C;
end
d.energy = d.kinetic + d.potential;
d.work = sol.x(3, :);
d.energyBalance = d.energy - d.energy(1) - d.work;
res.diag = d;
[isG, type] = rbsim.fourbar.grashof(p.l);
res.summary = struct('grashof', isG, 'type', type, ...
    'maxClosureResidual', max(d.closureResidual), ...
    'maxEnergyBalanceError', max(abs(d.energyBalance)), ...
    'minTransmissionDeg', min(d.transmissionDeg), ...
    'maxDriveTorque', max(abs(d.tau)), ...
    'outputRangeDeg', rad2deg([min(unwrap(d.phi1)) max(unwrap(d.phi1))]));
end

function [s, kin] = solveDynamics(x, p)
if strcmp(p.drive.mode, 'speed')
    [s, kin] = rbsim.fourbar.newtonEuler(x(1), x(2), p, struct('type', 'accel', 'value', 0));
else
    tau = 0;
    if strcmp(p.drive.mode, 'torque'), tau = p.drive.torque; end
    [s, kin] = rbsim.fourbar.newtonEuler(x(1), x(2), p, struct('type', 'torque', 'value', tau));
end
end

function zd = augmented(~, z, p)
% z = [theta1; theta1dot; W], W = work of drive + load + crank damper
s = solveDynamics(z(1:2), p);
kin = rbsim.fourbar.kinematics(z(1), z(2), s.thetaDDot1, p);
power = (s.tau - p.drive.damping * z(2)) * z(2) + p.drive.loadTorque * kin.omega(3);
zd = [z(2); s.thetaDDot1; power];
end
