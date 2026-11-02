function tests = test_angular_velocity
%TEST_ANGULAR_VELOCITY  Level 1-2 validation: skew matrices, angular velocity,
%   Euler-angle rate relations, and point velocity and acceleration.
    tests = functiontests(localfunctions);
end

function testSkewIsCrossProduct(tc)
    w = [0.3; -1.2; 2.5]; p = [1.1; 0.4; -0.7];
    tc.verifyEqual(rbsim.math.skew(w) * p, cross(w, p), 'AbsTol', 1e-14);
    tc.verifyEqual(rbsim.math.skew(w).', -rbsim.math.skew(w));
    tc.verifyEqual(rbsim.math.unskew(rbsim.math.skew(w)), w);
end

function testOmegaFromNumericalRdotMatchesXYZRates(tc)
%: w^s from Rdot R' for X-Y-Z angles, checked by central differences.
    th = [0.4; -0.3; 1.2]; thd = [0.7; -1.1; 0.5]; h = 1e-6;
    R = @(t) rbsim.math.rotBodyFixed('XYZ', (th + thd*t).');
    Rdot = (R(h) - R(-h)) / (2*h);
    [wsNum, wbNum] = rbsim.body.angularVelocityFromRdot(R(0), Rdot);
    [ws, wb] = rbsim.math.eulerRatesXYZ(th, thd);
    tc.verifyEqual(ws, wsNum, 'AbsTol', 1e-8);
    tc.verifyEqual(wb, wbNum, 'AbsTol', 1e-8);
end

function testOmegaFromNumericalRdotMatchesZYZRates(tc)
%: both w^s and w^b for Z-Y-Z angles.
    th = [0.9; 0.6; -0.4]; thd = [-0.3; 1.5; 0.8]; h = 1e-6;
    R = @(t) rbsim.math.rotBodyFixed('ZYZ', (th + thd*t).');
    Rdot = (R(h) - R(-h)) / (2*h);
    [wsNum, wbNum] = rbsim.body.angularVelocityFromRdot(R(0), Rdot);
    [ws, wb] = rbsim.math.eulerRatesZYZ(th, thd);
    tc.verifyEqual(ws, wsNum, 'AbsTol', 1e-8);
    tc.verifyEqual(wb, wbNum, 'AbsTol', 1e-8);
    tc.verifyEqual(ws, R(0) * wb, 'AbsTol', 1e-12);   % : w^s = R w^b
end

function testAngularVelocityOfAKnownAngleHistory(tc)
%: theta1 = 5t^2-4, theta2 = -10t, theta3 = 7t^3+9t-1, t = 3 s.
    t = 3;
    th  = [5*t^2 - 4; -10*t; 7*t^3 + 9*t - 1];
    thd = [10*t; -10; 21*t^2 + 9];
    [ws, wb] = rbsim.math.eulerRatesZYZ(th, thd);
    tc.verifyEqual(ws, [-194.7397; -21.1580; 60.5418], 'AbsTol', 5e-4);
    tc.verifyEqual(wb, [-15.6650; 27.0776; 202.6275], 'AbsTol', 5e-4);
    R = rbsim.math.rotBodyFixed('ZYZ', th.');
    tc.verifyEqual(R.' * ws, wb, 'AbsTol', 1e-9);     % "w^b = R' w^s" on s13
end

function testEulerRateInverseXYZ(tc)
    th = [0.3; 0.5; -0.8]; thd = [1.2; -0.4; 0.9];
    ws = rbsim.math.eulerRatesXYZ(th, thd);
    tc.verifyEqual(rbsim.math.eulerRatesFromOmegaXYZ(th, ws), thd, 'AbsTol', 1e-12);
    tc.verifyError(@() rbsim.math.eulerRatesFromOmegaXYZ([0; pi/2; 0], [1; 0; 0]), ...
        'rbsim:math:singular');
end

function testRotationRateConsistency(tc)
% Rdot = R [w^b x] = [w^s x] R
    R = rbsim.math.rotBodyFixed('XYZ', [0.2 -0.5 0.9]); wb = [0.4; -1.0; 2.0];
    Rdot = rbsim.body.rotationRate(R, wb);
    tc.verifyEqual(Rdot, rbsim.math.skew(R*wb) * R, 'AbsTol', 1e-14);
    [ws, wb2] = rbsim.body.angularVelocityFromRdot(R, Rdot);
    tc.verifyEqual(wb2, wb, 'AbsTol', 1e-14);
    tc.verifyEqual(ws, R*wb, 'AbsTol', 1e-14);
end

function testLinearVelocityOfAKnownPositionHistory(tc)
%: V(2) = [78; 35; 84] m/s (analytic derivative, checked numerically).
    p = @(t) [4*t^3 + 8*t^2 - 2*t + 5; 9*t^2 - t - 6; 7*t^3 + 10];
    h = 1e-5;
    tc.verifyEqual((p(2+h) - p(2-h)) / (2*h), [78; 35; 84], 'AbsTol', 1e-6);
end

function testPointVelocityMatchesDifferentiation(tc)
% V_p = V_O + w^s x (R ^Bp) + R ^BV_p, checked against d/dt of ^Ap.
    th0 = [0.1; 0.2; 0.3]; thd = [0.5; -0.7; 1.1];
    O = @(t) [1 + 2*t; -t^2; 3*t];
    pB = @(t) [0.2; 0.1 + 0.3*t; -0.4];
    R = @(t) rbsim.math.rotBodyFixed('XYZ', (th0 + thd*t).');
    pA = @(t) O(t) + R(t) * pB(t);
    h = 1e-6; t = 0;
    vNum = (pA(t+h) - pA(t-h)) / (2*h);
    ws = rbsim.math.eulerRatesXYZ(th0, thd);
    v = rbsim.body.pointVelocity([2; 0; 3], ws, R(0), pB(0), [0; 0.3; 0]);
    tc.verifyEqual(v, vNum, 'AbsTol', 1e-8);
end

function testPointAccelerationMatchesDifferentiation(tc)
% Five-term acceleration  vs second central difference.
    th0 = [0.1; 0.2; 0.3]; thd = [0.5; -0.7; 1.1];
    O = @(t) [1 + 2*t; -t^2; 3*t];
    pB = @(t) [0.2 + 0.5*t^2; 0.1 + 0.3*t; -0.4];
    R = @(t) rbsim.math.rotBodyFixed('XYZ', (th0 + thd*t).');
    pA = @(t) O(t) + R(t) * pB(t);
    h = 1e-4; t = 0;
    aNum = (pA(t+h) - 2*pA(t) + pA(t-h)) / h^2;
    ws = @(t) rbsim.math.eulerRatesXYZ(th0 + thd*t, thd);
    alpha = (ws(h) - ws(-h)) / (2*h);
    a = rbsim.body.pointAcceleration([0; -2; 0], alpha, ws(0), R(0), pB(0), [0; 0.3; 0], [1; 0; 0]);
    tc.verifyEqual(a, aNum, 'AbsTol', 1e-5);
end

function testPointPositionVectorised(tc)
    R = rbsim.math.rotZ(pi/2); O = [1; 2; 3];
    P = rbsim.body.pointPosition(O, R, [1 0; 0 1; 0 0]);
    tc.verifyEqual(P, [1 0; 3 2; 3 3], 'AbsTol', 1e-14);
end
