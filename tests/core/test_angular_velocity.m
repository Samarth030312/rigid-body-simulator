function tests = test_angular_velocity
%TEST_ANGULAR_VELOCITY Level 1-2 validation: skew matrices, angular velocity,
%   Euler-angle rate relations, point velocity/acceleration (L7-9).
tests = functiontests(localfunctions);
end

function testSkewIsCrossProduct(tc)
% L7-9/s6
w = [0.3; -1.2; 2.5]; p = [1.1; 0.4; -0.7];
verifyEqual(tc, skew(w) * p, cross(w, p), 'AbsTol', 1e-14);
verifyEqual(tc, skew(w).', -skew(w));
verifyEqual(tc, unskew(skew(w)), w);
end

function testOmegaFromNumericalRdotMatchesLectureXYZ(tc)
% L7-9/s7: w^s from Rdot R' for X-Y-Z angles, checked by central differences.
th = [0.4; -0.3; 1.2]; thd = [0.7; -1.1; 0.5]; h = 1e-6;
R = @(t) rotBodyFixed('XYZ', (th + thd*t).');
Rdot = (R(h) - R(-h)) / (2*h);
[wsNum, wbNum] = angularVelocityFromRdot(R(0), Rdot);
[ws, wb] = eulerRatesXYZ(th, thd);
verifyEqual(tc, ws, wsNum, 'AbsTol', 1e-8);
verifyEqual(tc, wb, wbNum, 'AbsTol', 1e-8);
end

function testOmegaFromNumericalRdotMatchesLectureZYZ(tc)
% L7-9/s8, s10: both w^s and w^b for Z-Y-Z angles.
th = [0.9; 0.6; -0.4]; thd = [-0.3; 1.5; 0.8]; h = 1e-6;
R = @(t) rotBodyFixed('ZYZ', (th + thd*t).');
Rdot = (R(h) - R(-h)) / (2*h);
[wsNum, wbNum] = angularVelocityFromRdot(R(0), Rdot);
[ws, wb] = eulerRatesZYZ(th, thd);
verifyEqual(tc, ws, wsNum, 'AbsTol', 1e-8);
verifyEqual(tc, wb, wbNum, 'AbsTol', 1e-8);
verifyEqual(tc, ws, R(0) * wb, 'AbsTol', 1e-12);   % L7-9/s10: w^s = R w^b
end

function testLectureNumericalAngularVelocity(tc)
% L7-9/s12-13: theta1 = 5t^2-4, theta2 = -10t, theta3 = 7t^3+9t-1, t = 3 s.
t = 3;
th  = [5*t^2 - 4; -10*t; 7*t^3 + 9*t - 1];
thd = [10*t; -10; 21*t^2 + 9];
[ws, wb] = eulerRatesZYZ(th, thd);
verifyEqual(tc, ws, [-194.7397; -21.1580; 60.5418], 'AbsTol', 5e-4);
verifyEqual(tc, wb, [-15.6650; 27.0776; 202.6275], 'AbsTol', 5e-4);
R = rotBodyFixed('ZYZ', th.');
verifyEqual(tc, R.' * ws, wb, 'AbsTol', 1e-9);     % "w^b = R' w^s" on s13
end

function testEulerRateInverseXYZ(tc)
th = [0.3; 0.5; -0.8]; thd = [1.2; -0.4; 0.9];
ws = eulerRatesXYZ(th, thd);
verifyEqual(tc, eulerRatesFromOmegaXYZ(th, ws), thd, 'AbsTol', 1e-12);
verifyError(tc, @() eulerRatesFromOmegaXYZ([0; pi/2; 0], [1; 0; 0]), ...
    'eulerRatesFromOmegaXYZ:singular');
end

function testRotationRateConsistency(tc)
% Rdot = R [w^b x] = [w^s x] R  (L7-9/s9, s22)
R = rotBodyFixed('XYZ', [0.2 -0.5 0.9]); wb = [0.4; -1.0; 2.0];
Rdot = rotationRate(R, wb);
verifyEqual(tc, Rdot, skew(R*wb) * R, 'AbsTol', 1e-14);
[ws, wb2] = angularVelocityFromRdot(R, Rdot);
verifyEqual(tc, wb2, wb, 'AbsTol', 1e-14);
verifyEqual(tc, ws, R*wb, 'AbsTol', 1e-14);
end

function testLectureLinearVelocityExample(tc)
% L7-9/s11: V(2) = [78; 35; 84] m/s (analytic derivative, checked numerically).
p = @(t) [4*t^3 + 8*t^2 - 2*t + 5; 9*t^2 - t - 6; 7*t^3 + 10];
h = 1e-5;
verifyEqual(tc, (p(2+h) - p(2-h)) / (2*h), [78; 35; 84], 'AbsTol', 1e-6);
end

function testPointVelocityMatchesDifferentiation(tc)
% V_p = V_O + w^s x (R ^Bp) + R ^BV_p, checked against d/dt of ^Ap (L7-9/s20-23).
th0 = [0.1; 0.2; 0.3]; thd = [0.5; -0.7; 1.1];
O = @(t) [1 + 2*t; -t^2; 3*t];
pB = @(t) [0.2; 0.1 + 0.3*t; -0.4];
R = @(t) rotBodyFixed('XYZ', (th0 + thd*t).');
pA = @(t) O(t) + R(t) * pB(t);
h = 1e-6; t = 0;
vNum = (pA(t+h) - pA(t-h)) / (2*h);
ws = eulerRatesXYZ(th0, thd);
v = pointVelocity([2; 0; 3], ws, R(0), pB(0), [0; 0.3; 0]);
verifyEqual(tc, v, vNum, 'AbsTol', 1e-8);
end

function testPointAccelerationMatchesDifferentiation(tc)
% Five-term acceleration (L7-9/s23-24) vs second central difference.
th0 = [0.1; 0.2; 0.3]; thd = [0.5; -0.7; 1.1];
O = @(t) [1 + 2*t; -t^2; 3*t];
pB = @(t) [0.2 + 0.5*t^2; 0.1 + 0.3*t; -0.4];
R = @(t) rotBodyFixed('XYZ', (th0 + thd*t).');
pA = @(t) O(t) + R(t) * pB(t);
h = 1e-4; t = 0;
aNum = (pA(t+h) - 2*pA(t) + pA(t-h)) / h^2;
ws = @(t) eulerRatesXYZ(th0 + thd*t, thd);
alpha = (ws(h) - ws(-h)) / (2*h);
a = pointAcceleration([0; -2; 0], alpha, ws(0), R(0), pB(0), [0; 0.3; 0], [1; 0; 0]);
verifyEqual(tc, a, aNum, 'AbsTol', 1e-5);
end

function testPointPositionVectorised(tc)
R = rotZ(pi/2); O = [1; 2; 3];
P = pointPosition(O, R, [1 0; 0 1; 0 0]);
verifyEqual(tc, P, [1 0; 3 2; 3 3], 'AbsTol', 1e-14);
end
