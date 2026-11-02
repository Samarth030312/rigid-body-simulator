function tests = test_fourbar
%TEST_FOURBAR Level 2/3/5 validation of the four-bar mechanism.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
tc.TestData.p = rbsim.fourbar.defaults();
tc.TestData.th = linspace(0, 2*pi, 37);
end

% ---------- position ----------
function testConstraintAndClosureResidualsVanish(tc)
% eta1 = 0  and the three loop-closure equations.
p = tc.TestData.p;
for b = [1 -1]
    p.branch = b;
    for th = tc.TestData.th
        [pos, ok] = rbsim.fourbar.position(th, p);
        verifyTrue(tc, ok);
        verifyEqual(tc, pos.eta1, 0, 'AbsTol', 1e-14);
        verifyEqual(tc, pos.closure, [0; 0], 'AbsTol', 1e-14);
        verifyEqual(tc, pos.angleSum, 0, 'AbsTol', 1e-12);
        verifyEqual(tc, norm(pos.B - pos.A), p.l(3), 'AbsTol', 1e-14);
        verifyEqual(tc, norm(pos.B - pos.OR), p.l(4), 'AbsTol', 1e-14);
    end
end
end

function testOpenBranchIsTheUncrossedAssembly(tc)
%: theta1 ~ 75 deg, coupler above the ground, obtuse output angle.
p = tc.TestData.p;
pos = rbsim.fourbar.position(deg2rad(75), p);
verifyGreaterThan(tc, pos.B(2), 0);
verifyGreaterThan(tc, pos.phi1, pi/2);
verifyLessThan(tc, pos.phi1, pi);
p.branch = -1;
pos2 = rbsim.fourbar.position(deg2rad(75), p);
verifyLessThan(tc, pos2.B(2), 0);          % crossed branch is the mirror assembly
end

function testFreudensteinCheckIndependent(tc)
% Independent check: solve eta1(theta1, phi1) = 0 numerically with fzero,
% starting near the closed-form answer.
p = tc.TestData.p; l = p.l;
eta = @(t, f) (l(2)*cos(t) - l(1) - l(4)*cos(f))^2 + (l(2)*sin(t) - l(4)*sin(f))^2 - l(3)^2;
for th = tc.TestData.th(1:6:end)
    pos = rbsim.fourbar.position(th, p);
    fz = fzero(@(f) eta(th, f), pos.phi1 + 0.01);
    verifyEqual(tc, fz, pos.phi1, 'AbsTol', 1e-9);
end
end

function testGrashofClassification(tc)
[g, t] = rbsim.fourbar.grashof([0.40 0.12 0.36 0.28]); verifyTrue(tc, g); verifyEqual(tc, t, 'crank-rocker');
[g, t] = rbsim.fourbar.grashof([0.10 0.30 0.35 0.30]); verifyTrue(tc, g); verifyEqual(tc, t, 'double-crank');
[g, t] = rbsim.fourbar.grashof([0.30 0.35 0.10 0.30]); verifyTrue(tc, g); verifyEqual(tc, t, 'double-rocker');
[g, t] = rbsim.fourbar.grashof([0.40 0.30 0.35 0.32]); verifyFalse(tc, g); verifyEqual(tc, t, 'triple-rocker (non-Grashof)');
end

function testAssemblyFailureReported(tc)
% Non-Grashof linkage: some crank angles cannot be assembled -> ok = false.
p = tc.TestData.p; p.l = [0.40 0.30 0.35 0.32];
[~, ok] = rbsim.fourbar.position(pi, p);
verifyFalse(tc, ok);
p.theta0Deg = 180;
verifyError(tc, @() rbsim.fourbar.simulate(p), 'rbsim:fourbar:param');
end

% ---------- velocity / acceleration ----------
function testVelocityAccelerationVsFiniteDifference(tc)
p = tc.TestData.p; h = 1e-5;
for th = tc.TestData.th(1:4:end)
    w = 2.3; al = -1.7;
    ang = @(t) angles(th + w*t + 0.5*al*t^2, p);
    pts = @(t) comPoints(th + w*t + 0.5*al*t^2, p);
    kin = rbsim.fourbar.kinematics(th, w, al, p);
    % angles are wrapped to (-pi, pi]; take differences on the circle
    wrap = @(a) mod(a + pi, 2*pi) - pi;
    dp = wrap(ang(h) - ang(0)); dm = wrap(ang(0) - ang(-h));
    verifyEqual(tc, kin.omega, (dp + dm) / (2*h), 'AbsTol', 1e-7);
    verifyEqual(tc, kin.alpha, (dp - dm) / h^2, 'AbsTol', 2e-4);
    verifyEqual(tc, kin.vC, (pts(h) - pts(-h)) / (2*h), 'AbsTol', 1e-8);
    verifyEqual(tc, kin.aC, (pts(h) - 2*pts(0) + pts(-h)) / h^2, 'AbsTol', 1e-4);
end
end

function a = angles(th, p)
pos = rbsim.fourbar.position(th, p);
a = [pos.theta1, pos.theta2, pos.phi1];
end

function C = comPoints(th, p)
kin = rbsim.fourbar.kinematics(th, 0, 0, p); C = kin.C;
end

% ---------- dynamics ----------
function testNewtonEulerMatchesEnergyEquation(tc)
% Independent check of the FBD solution: one-DOF energy equation
%   Meff th1dd + 1/2 dMeff th1dot^2 + dV = tau    (no reactions involved)
p = tc.TestData.p;
for th = tc.TestData.th(1:3:end)
    w = 4.1; tau = 0.8;
    s = rbsim.fourbar.newtonEuler(th, w, p, struct('type', 'torque', 'value', tau));
    [~, ~, M, dM, dV] = rbsim.fourbar.energy(th, w, p);
    verifyEqual(tc, M * s.thetaDDot1 + 0.5 * dM * w^2 + dV, tau, 'AbsTol', 1e-6);
end
end

function testInverseForwardConsistency(tc)
p = tc.TestData.p;
for th = tc.TestData.th(1:5:end)
    s1 = rbsim.fourbar.newtonEuler(th, 3, p, struct('type', 'accel', 'value', -2.5));
    s2 = rbsim.fourbar.newtonEuler(th, 3, p, struct('type', 'torque', 'value', s1.tau));
    verifyEqual(tc, s2.thetaDDot1, -2.5, 'AbsTol', 1e-10);
    verifyEqual(tc, s2.F_A, s1.F_A, 'AbsTol', 1e-9);
end
end

function testStaticsVirtualWork(tc)
% Static holding torque equals dV/dtheta1 (virtual work,,);
% ground reactions carry the total weight (equilibrium,).
p = tc.TestData.p;
for th = tc.TestData.th(1:6:end)
    s = rbsim.fourbar.newtonEuler(th, 0, p, struct('type', 'accel', 'value', 0));
    [~, ~, ~, ~, dV] = rbsim.fourbar.energy(th, 0, p);
    verifyEqual(tc, s.tau, dV, 'AbsTol', 1e-7);
    verifyEqual(tc, s.F_OL + s.F_OR, [0; p.g * sum(p.m)], 'AbsTol', 1e-10);
end
end

function testGeneralizedForceOfACrankPinLoad(tc)
%   For forces F2 at A and F3 at B and a moment M1 on
% the crank, Q_theta = M1 - F_x2 l1 sin(theta) + F_y2 l1 cos(theta). In statics
% with g = 0, a force F2 applied at A must be balanced by tau = -Q_F-part.
p = tc.TestData.p; p.g = 0; p.m = [0 0 0]; p.Izz = [0 0 0];
p.m(2) = 1e-12;   % keep the linear system well-posed without affecting statics
th = deg2rad(50);
% Apply F2 via equivalent load torque on the crank: tau_required = -(Q from F2)
F2 = [1.5; -0.7];
Q = -F2(1) * p.l(2) * sin(th) + F2(2) * p.l(2) * cos(th);
kin = rbsim.fourbar.kinematics(th, 1, 0, p);
dA = kin.vC(:, 2) - p.r(2) * [-sin(kin.theta2); cos(kin.theta2)] * kin.k(2);  % dA/dtheta1
verifyEqual(tc, F2.' * dA, Q, 'AbsTol', 1e-14);     % virtual work of F2 = Q delta theta
end

function testFreeMotionConservesEnergy(tc)
p = tc.TestData.p; p.tFinal = 2;
res = rbsim.fourbar.simulate(p);
verifyLessThan(tc, res.summary.maxEnergyBalanceError, 1e-6);
verifyLessThan(tc, max(abs(res.diag.energy - res.diag.energy(1))), 1e-6);
verifyLessThan(tc, res.summary.maxClosureResidual, 1e-12);
end

function testTorqueDriveEnergyBalance(tc)
p = tc.TestData.p; p.drive.mode = 'torque'; p.drive.torque = 0.3;
p.drive.damping = 0.01; p.drive.loadTorque = -0.05; p.tFinal = 2;
res = rbsim.fourbar.simulate(p);
verifyLessThan(tc, res.summary.maxEnergyBalanceError, 1e-6);
verifyGreaterThan(tc, max(res.diag.work), 0.1);
end

function testSpeedDriveInverseDynamics(tc)
% Constant crank speed: theta1 = theta0 + w t exactly; required torque satisfies
% tau w = dE/dt (power balance,).
p = tc.TestData.p; p.drive.mode = 'speed'; p.drive.speedDeg = 180; p.tFinal = 2;
p.outputEvery = 1;
res = rbsim.fourbar.simulate(p);
w = deg2rad(180);
verifyEqual(tc, res.x(1, :), deg2rad(p.theta0Deg) + w * res.t, 'AbsTol', 1e-12);
verifyLessThan(tc, res.summary.maxEnergyBalanceError, 1e-6);
dEdt = gradient(res.diag.energy, res.t);
verifyEqual(tc, res.diag.tau(3:end-2) * w, dEdt(3:end-2), 'AbsTol', 2e-3);
end

function testToggleDetected(tc)
% Change-point linkage reaches a collinear configuration -> clear error.
p = tc.TestData.p;
p.l = [0.30 0.10 0.30 0.10];              % s + L = p + q (change point)
p.r = p.l(2:4) / 2;
verifyError(tc, @() rbsim.fourbar.newtonEuler(0, 1, p, struct('type', 'torque', 'value', 0)), ...
    'rbsim:fourbar:singular');
end
