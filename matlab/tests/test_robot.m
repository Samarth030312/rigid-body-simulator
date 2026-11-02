function tests = test_robot
%TEST_ROBOT  Level 2/3/5 validation of the planar 3R manipulator.
    tests = functiontests(localfunctions);
end

function setupOnce(tc)
    rng(3, 'twister');
    tc.TestData.p = rbsim.robot.defaults();
    tc.TestData.Q = (rand(3, 8) - 0.5) * 2 * pi;
    tc.TestData.QD = (rand(3, 8) - 0.5) * 6;
    tc.TestData.QDD = (rand(3, 8) - 0.5) * 10;
end

% ---------- kinematics ----------
function testForwardKinematicsKnownPoses(tc)
    p = tc.TestData.p; L = sum(p.l);
    fk = rbsim.robot.forwardKinematics([0;0;0], p);
    tc.verifyEqual(fk.tool, [L; 0; 0], 'AbsTol', 1e-15);
    fk = rbsim.robot.forwardKinematics([pi/2;0;0], p);
    tc.verifyEqual(fk.tool, [0; L; 0], 'AbsTol', 1e-15);
    fk = rbsim.robot.forwardKinematics([0; pi/2; -pi/2], p);
    tc.verifyEqual(fk.tool, [p.l(1) + p.l(3); p.l(2); 0], 'AbsTol', 1e-15);
end

function testForwardKinematicsClosedForm(tc)
% x = l1 c1 + l2 c12 + l3 c123 (planar closed form, cf.);
% ^0_3[R] = R(Z, theta1+theta2+theta3).
    p = tc.TestData.p;
    for k = 1:size(tc.TestData.Q, 2)
        q = tc.TestData.Q(:, k); a = cumsum(q);
        fk = rbsim.robot.forwardKinematics(q, p);
        tc.verifyEqual(fk.tool, [p.l * cos(a); p.l * sin(a); 0], 'AbsTol', 1e-14);
        tc.verifyEqual(fk.R{3}, rbsim.math.rotZ(sum(q)), 'AbsTol', 1e-14);
        tc.verifyEqual(fk.C(:, 2), fk.O(:, 2) + p.r(2) * [cos(a(2)); sin(a(2)); 0], 'AbsTol', 1e-14);
        tc.verifyEqual(norm(fk.O(:, 3) - fk.O(:, 2)), p.l(2), 'AbsTol', 1e-14);  % rigid links
    end
end

function testToolVelocityAccelerationPropagation(tc)
% Propagation  vs finite differences of forward kinematics.
    p = tc.TestData.p; h = 1e-5;
    for k = 1:4
        q = tc.TestData.Q(:, k); qd = tc.TestData.QD(:, k); qdd = tc.TestData.QDD(:, k);
        X = @(t) toolAt(q + qd*t + 0.5*qdd*t^2, p);
        vNum = (X(h) - X(-h)) / (2*h);
        aNum = (X(h) - 2*X(0) + X(-h)) / h^2;
        [v0, a0, vT, aT] = rbsim.robot.toolKinematics(q, qd, qdd, p);
        tc.verifyEqual(v0, vNum, 'AbsTol', 1e-8);
        tc.verifyEqual(a0, aNum, 'AbsTol', 1e-4);
        fk = rbsim.robot.forwardKinematics(q, p);
        tc.verifyEqual(vT, fk.R{4}.' * v0, 'AbsTol', 1e-12);         % {Tool} components
        tc.verifyEqual(aT, fk.R{4}.' * a0, 'AbsTol', 1e-12);
    end
end

function x = toolAt(q, p)
    fk = rbsim.robot.forwardKinematics(q, p); x = fk.tool;
end

% ---------- dynamics ----------
function testReducesToThePlanar2REquations(tc)
% Link 3 massless -> tau1, tau2 must equal  exactly; tau3 = 0.
    p = tc.TestData.p; p.m(3) = 0; p.Izz(3) = 0;
    m1 = p.m(1); m2 = p.m(2); l1 = p.l(1); r1 = p.r(1); r2 = p.r(2);
    I1 = p.Izz(1); I2 = p.Izz(2); g = p.g;
    for k = 1:size(tc.TestData.Q, 2)
        q = tc.TestData.Q(:, k); qd = tc.TestData.QD(:, k); qdd = tc.TestData.QDD(:, k);
        c1 = cos(q(1)); c2 = cos(q(2)); s2 = sin(q(2)); c12 = cos(q(1) + q(2));
        M = [I1 + I2 + m2*l1^2 + m1*r1^2 + m2*r2^2 + 2*m2*l1*r2*c2, I2 + m2*r2^2 + m2*l1*r2*c2;
             I2 + m2*r2^2 + m2*l1*r2*c2,                            I2 + m2*r2^2];
        C = [-m2*l1*r2*s2*(2*qd(1) + qd(2))*qd(2); m2*l1*r2*s2*qd(1)^2];
        G = [m2*g*(l1*c1 + r2*c12) + m1*g*r1*c1; m2*r2*g*c12];
        tauRef = M * qdd(1:2) + C + G;
        tau = rbsim.robot.newtonEuler(q, qd, qdd, p);
        tc.verifyEqual(tau(1:2), tauRef, 'AbsTol', 1e-11);
        tc.verifyEqual(tau(3), 0, 'AbsTol', 1e-13);
    end
end

function testGravityTorqueEqualsVirtualWork(tc)
% Static torques G(q) = dV/dq (virtual work,,).
    p = tc.TestData.p; h = 1e-6;
    for k = 1:size(tc.TestData.Q, 2)
        q = tc.TestData.Q(:, k);
        G = rbsim.robot.newtonEuler(q, zeros(3,1), zeros(3,1), p);
        dV = zeros(3, 1);
        for j = 1:3
            e = zeros(3,1); e(j) = h;
            [~, Vp] = rbsim.robot.energy([q + e; 0;0;0], p);
            [~, Vm] = rbsim.robot.energy([q - e; 0;0;0], p);
            dV(j) = (Vp - Vm) / (2*h);
        end
        tc.verifyEqual(G, dV, 'AbsTol', 1e-7);
    end
end

function testMassMatrixMatchesKineticEnergy(tc)
% T = 1/2 qd' M qd, with T computed link-by-link (independent of M).
    p = tc.TestData.p;
    for k = 1:size(tc.TestData.Q, 2)
        q = tc.TestData.Q(:, k); qd = tc.TestData.QD(:, k);
        M = rbsim.robot.massMatrix(q, p);
        T = rbsim.robot.energy([q; qd], p);
        tc.verifyEqual(0.5 * qd.' * M * qd, T, 'RelTol', 1e-12);
        tc.verifyEqual(M, M.', 'AbsTol', 1e-14);
        tc.verifyGreaterThan(min(eig(M)), 0);
    end
end

function testInverseForwardDynamicsConsistency(tc)
% qdd -> RNE tau -> forward dynamics -> same qdd.
    p = tc.TestData.p;
    for k = 1:4
        q = tc.TestData.Q(:, k); qd = tc.TestData.QD(:, k); qdd = tc.TestData.QDD(:, k);
        tau = rbsim.robot.newtonEuler(q, qd, qdd, p);
        p2 = p; p2.control.mode = 'constant'; p2.control.tau = tau.';
        xd = rbsim.robot.dynamics(0, [q; qd], p2, []);
        tc.verifyEqual(xd(4:6), qdd, 'AbsTol', 1e-10);
    end
end

function testToolForceStatics(tc)
% Pure statics, g = 0: tau = J' F for a tool force (virtual work).
    p = tc.TestData.p; q = [0.3; -0.8; 0.5];
    F0 = [2; -3; 0];                          % force exerted by the tool, in {0}
    fk = rbsim.robot.forwardKinematics(q, p);
    FT = fk.R{4}.' * F0;
    tau = rbsim.robot.newtonEuler(q, zeros(3,1), zeros(3,1), p, 0, FT);
    J = zeros(3); h = 1e-6;
    for j = 1:3
        e = zeros(3,1); e(j) = h;
        J(:, j) = (toolAt(q + e, p) - toolAt(q - e, p)) / (2*h);
    end
    tc.verifyEqual(tau, J.' * F0, 'AbsTol', 1e-8);
end

% ---------- simulation ----------
function testFreeMotionConservesEnergy(tc)
% Passive triple pendulum, no damping: E = T + V constant.
    p = tc.TestData.p; p.tFinal = 3; p.dt = 5e-4; p.outputEvery = 20;
    res = rbsim.robot.simulate(p);
    dE = res.diag.energy - res.diag.energy(1);
    tc.verifyLessThan(max(abs(dE)), 1e-6 * max(abs(res.diag.energy)) + 1e-8);
    tc.verifyGreaterThan(max(res.diag.kinetic), 1);          % it really moved
end

function testDampingDissipates(tc)
    p = tc.TestData.p; p.damping = [0.5 0.5 0.5]; p.tFinal = 3; p.outputEvery = 1;
    res = rbsim.robot.simulate(p);
    tc.verifyLessThan(max(diff(res.diag.energy)), 1e-9);
    tc.verifyLessThan(res.summary.maxEnergyBalanceError, 1e-8);   % E - E0 = damper work
end

function testPDRegulationReachesTarget(tc)
    p = tc.TestData.p; p.control.mode = 'pd'; p.tFinal = 4;
    res = rbsim.robot.simulate(p);
    tc.verifyEqual(res.x(1:3, end), deg2rad(p.control.qTargetDeg(:)), 'AbsTol', 1e-3);
    % Energy balance E - E0 = actuator work (tau.qd = dE/dt,). Residual is
    % RK4 error: it must be small and fall ~16x when dt halves (4th order).
    e1 = res.summary.maxEnergyBalanceError;
    tc.verifyLessThan(e1, 1e-3);
    p.dt = p.dt / 2; p.outputEvery = 2 * p.outputEvery;
    e2 = rbsim.robot.simulate(p).summary.maxEnergyBalanceError;
    tc.verifyGreaterThan(e1 / e2, 10);
end

function testComputedTorqueTracking(tc)
    p = tc.TestData.p; p.control.mode = 'trajectory'; p.tFinal = 2.5; p.outputEvery = 1;
    res = rbsim.robot.simulate(p);
    tc.verifyLessThan(res.summary.trackingErrorMax, 1e-6);
    tc.verifyEqual(res.x(1:3, end), deg2rad(p.control.qTargetDeg(:)), 'AbsTol', 1e-6);
end

function testStaticHoldWithGravityTorque(tc)
% Applying tau = G(q0) holds the arm still (equilibrium,).
    p = tc.TestData.p; q0 = deg2rad(p.q0(:));
    p.control.mode = 'constant';
    p.control.tau = rbsim.robot.newtonEuler(q0, zeros(3,1), zeros(3,1), p).';
    p.tFinal = 1;
    res = rbsim.robot.simulate(p);
    tc.verifyEqual(res.x(:, end), [q0; zeros(3,1)], "AbsTol", 1e-10);
end

function testParameterValidation(tc)
    p = tc.TestData.p; p.r(2) = 1.0;
    tc.verifyError(@() rbsim.robot.simulate(p), 'rbsim:robot:param');
    p = tc.TestData.p; p.l(1) = 0;
    tc.verifyError(@() rbsim.robot.simulate(p), 'rbsim:robot:param');
    p = tc.TestData.p; p.control.mode = 'magic';
    tc.verifyError(@() rbsim.robot.simulate(p), 'rbsim:robot:param');
end
