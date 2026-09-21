function tests = test_rotations
%TEST_ROTATIONS Level 1 validation of rotation-matrix primitives (core/math).
%   Reference values are lecture slides (L2-4) where available.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
tc.TestData.tol = 1e-12;       % algebraic identities
tc.TestData.tolLecture = 5e-5; % lecture values are printed to 4 decimals
rng(209, 'twister');           % deterministic random samples
tc.TestData.angles = (rand(50, 3) - 0.5) * 2 * pi;
end

% ---------- simple rotations ----------
function testSimpleRotationsAreProperOrthonormal(tc)
for phi = linspace(-2*pi, 2*pi, 25)
    for R = {rotX(phi), rotY(phi), rotZ(phi)}
        verifyTrue(tc, isRotationMatrix(R{1}, 1e-12));
    end
end
end

function testSimpleRotationRightHandRule(tc)
% +90 deg about Z maps X_B onto Y_A (counter-clockwise about +Z).
verifyEqual(tc, rotZ(pi/2) * [1;0;0], [0;1;0], 'AbsTol', tc.TestData.tol);
verifyEqual(tc, rotX(pi/2) * [0;1;0], [0;0;1], 'AbsTol', tc.TestData.tol);
verifyEqual(tc, rotY(pi/2) * [0;0;1], [1;0;0], 'AbsTol', tc.TestData.tol);
end

function testLectureExample1RotZ45(tc)
% L2-4/s12
Rexp = [1/sqrt(2) -1/sqrt(2) 0; 1/sqrt(2) 1/sqrt(2) 0; 0 0 1];
verifyEqual(tc, rotZ(pi/4), Rexp, 'AbsTol', tc.TestData.tol);
end

function testColumnsAreBodyAxesInA(tc)
% L2-4/s8: columns of ^A_B[R] are X_B, Y_B, Z_B described in {A}.
R = rotBodyFixed('XYZ', [0.3 -0.7 1.1]);
verifyEqual(tc, R * [1;0;0], R(:,1), 'AbsTol', tc.TestData.tol);
verifyEqual(tc, R * [0;0;1], R(:,3), 'AbsTol', tc.TestData.tol);
end

function testInverseIsTranspose(tc)
% L2-4/s9
for i = 1:size(tc.TestData.angles, 1)
    R = rotBodyFixed('ZYZ', tc.TestData.angles(i,:));
    verifyEqual(tc, R \ eye(3), R.', 'AbsTol', 1e-12);
    verifyEqual(tc, det(R), 1, 'AbsTol', 1e-12);
end
end

% ---------- composition (L2-4/s21-36) ----------
function testLectureBodyFixedXYZ(tc)
% L2-4/s50 and s59: body-fixed X-Y-Z 60, 45, -75 deg.
R = rotBodyFixed('XYZ', deg2rad([60 45 -75]));
Rexp = [0.1830 0.6830 0.7071; -0.3245 0.7209 -0.6124; -0.9280 -0.1174 0.3536];
verifyEqual(tc, R, Rexp, 'AbsTol', tc.TestData.tolLecture);
end

function testLectureSpaceFixedXYZ(tc)
% L2-4/s44: space-fixed X-Y-Z 60, 45, -75 deg.
R = rotSpaceFixed('XYZ', deg2rad([60 45 -75]));
Rexp = [0.1830 0.6415 -0.7450; -0.6830 -0.4621 -0.5657; -0.7071 0.6124 0.3536];
verifyEqual(tc, R, Rexp, 'AbsTol', tc.TestData.tolLecture);
end

function testLectureBodyFixedZYZ(tc)
% L2-4/s64: body-fixed Z-Y-Z 30, 60, 90 deg.
R = rotBodyFixed('ZYZ', deg2rad([30 60 90]));
Rexp = [-0.5 -0.4330 0.75; 0.8660 -0.25 0.4330; 0 0.8660 0.5];
verifyEqual(tc, R, Rexp, 'AbsTol', tc.TestData.tolLecture);
end

function testLectureClosedFormXYZ(tc)
% Closed-form X-Y-Z matrix printed on L2-4/s25.
th = [0.4 -1.2 2.5];
c = cos(th); s = sin(th);
Rexp = [c(2)*c(3), -c(2)*s(3), s(2);
        s(1)*s(2)*c(3)+s(3)*c(1), -s(1)*s(2)*s(3)+c(3)*c(1), -s(1)*c(2);
       -c(1)*s(2)*c(3)+s(3)*s(1),  c(1)*s(2)*s(3)+c(3)*s(1),  c(1)*c(2)];
verifyEqual(tc, rotBodyFixed('XYZ', th), Rexp, 'AbsTol', tc.TestData.tol);
end

function testLectureClosedFormSpaceFixedXYZ(tc)
% Closed-form space-fixed X-Y-Z matrix printed on L2-4/s36.
th = [0.4 -1.2 2.5];
c = cos(th); s = sin(th);
Rexp = [c(2)*c(3), s(1)*s(2)*c(3)-c(1)*s(3), c(1)*s(2)*c(3)+s(1)*s(3);
        c(2)*s(3), s(1)*s(2)*s(3)+c(1)*c(3), c(1)*s(2)*s(3)-s(1)*c(3);
       -s(2),      s(1)*c(2),                c(1)*c(2)];
verifyEqual(tc, rotSpaceFixed('XYZ', th), Rexp, 'AbsTol', tc.TestData.tol);
end

function testSpaceFixedByAxisAngleMethod(tc)
% Reproduce the lecture derivation (L2-4/s32-35): each later rotation is about
% the FIXED axis expressed in the current body frame, then post-multiplied.
th = [0.7 -0.4 1.3];
R1 = rotX(th(1));
kY = R1.' * [0;1;0];                   % ^{B1} Y_A
R2 = R1 * rotAxisAngle(kY, th(2));
kZ = R2.' * [0;0;1];                   % ^{B2} Z_A
R3 = R2 * rotAxisAngle(kZ, th(3));
verifyEqual(tc, rotSpaceFixed('XYZ', th), R3, 'AbsTol', 1e-12);
end

function testBodyXYZEqualsSpaceZYX(tc)
% L2-4/s59
th = deg2rad([60 45 -75]);
verifyEqual(tc, rotBodyFixed('XYZ', th), ...
    rotSpaceFixed('ZYX', fliplr(th)), 'AbsTol', tc.TestData.tol);
end

function testCompositionIsNotCommutative(tc)
% L2-4/s21: order of rotation is important.
A = rotX(0.5); B = rotY(0.8);
verifyGreaterThan(tc, norm(A*B - B*A, 'fro'), 0.1);
end

function testRepeatedAxisRejected(tc)
verifyError(tc, @() rotBodyFixed('XXY', [1 2 3]), 'rotBodyFixed:repeatedAxis');
verifyError(tc, @() rotBodyFixed('XY', [1 2 3]), 'rotBodyFixed:sizeMismatch');
end

% ---------- axis-angle (L2-4/s14-18) ----------
function testAxisAngleMatchesSimpleRotations(tc)
phi = 0.83;
verifyEqual(tc, rotAxisAngle([1;0;0], phi), rotX(phi), 'AbsTol', tc.TestData.tol);
verifyEqual(tc, rotAxisAngle([0;1;0], phi), rotY(phi), 'AbsTol', tc.TestData.tol);
verifyEqual(tc, rotAxisAngle([0;0;2], phi), rotZ(phi), 'AbsTol', tc.TestData.tol);
end

function testRodriguesFormula(tc)
% L2-4/s16: Q' = Q cos + (k x Q) sin + (1-cos)(k.Q) k
k = [1; 2; -2] / 3; th = 1.1; Q = [0.3; -1.4; 2.2];
Qp = Q*cos(th) + cross(k, Q)*sin(th) + (1 - cos(th))*dot(k, Q)*k;
verifyEqual(tc, rotAxisAngle(k, th) * Q, Qp, 'AbsTol', tc.TestData.tol);
end

function testAxisAngleRoundTrip(tc)
for i = 1:size(tc.TestData.angles, 1)
    R = rotBodyFixed('XYZ', tc.TestData.angles(i,:));
    [k, phi] = rotToAxisAngle(R);
    verifyEqual(tc, rotAxisAngle(k, phi), R, 'AbsTol', 1e-9);
    verifyEqual(tc, R * k, k, 'AbsTol', 1e-9);   % L2-4/s15: k is fixed
end
end

function testAxisAngleSpecialCases(tc)
[k, phi] = rotToAxisAngle(eye(3));
verifyEqual(tc, phi, 0); verifyEqual(tc, norm(k), 1, 'AbsTol', 1e-12);
kIn = [1; -1; 2] / sqrt(6);
[k, phi] = rotToAxisAngle(rotAxisAngle(kIn, pi));
verifyEqual(tc, phi, pi, 'AbsTol', 1e-9);
verifyEqual(tc, abs(dot(k, kIn)), 1, 'AbsTol', 1e-9);   % axis sign is free at pi
end

function testEigenvaluesOfR(tc)
% L2-4/s14: eigenvalues +1, exp(+-i phi) with phi = acos((trace-1)/2).
R = rotBodyFixed('XYZ', [0.2 0.9 -0.4]);
phi = acos((trace(R) - 1)/2);
ev = sort(eig(R), 'ComparisonMethod', 'real');
verifyEqual(tc, abs(ev), ones(3,1), 'AbsTol', 1e-12);
verifyEqual(tc, sort(angle(ev)), [-phi; 0; phi], 'AbsTol', 1e-12);
end

% ---------- inverse Euler (L2-4/s26-27, s31) ----------
function testEulerXYZRoundTrip(tc)
for i = 1:size(tc.TestData.angles, 1)
    th = tc.TestData.angles(i,:);
    R = rotBodyFixed('XYZ', th);
    thBack = rotToEulerXYZ(R);
    verifyEqual(tc, rotBodyFixed('XYZ', thBack), R, 'AbsTol', 1e-10);
end
end

function testEulerXYZLectureValues(tc)
th = deg2rad([60 45 -75]);
verifyEqual(tc, rotToEulerXYZ(rotBodyFixed('XYZ', th)), th, 'AbsTol', 1e-12);
end

function testEulerXYZSingular(tc)
% L2-4/s27: theta2 = +-pi/2 -> theta3 = 0 convention, R still reproduced.
for t2 = [pi/2, -pi/2]
    R = rotBodyFixed('XYZ', [0.4 t2 0.3]);
    th = rotToEulerXYZ(R);
    verifyEqual(tc, th(3), 0);
    verifyEqual(tc, rotBodyFixed('XYZ', th), R, 'AbsTol', 1e-10);
end
end

function testEulerZYZRoundTrip(tc)
for i = 1:size(tc.TestData.angles, 1)
    th = tc.TestData.angles(i,:);
    R = rotBodyFixed('ZYZ', th);
    verifyEqual(tc, rotBodyFixed('ZYZ', rotToEulerZYZ(R)), R, 'AbsTol', 1e-10);
end
end

function testEulerZYZSingular(tc)
% L2-4/s31: r33 = +-1 cases.
R = rotBodyFixed('ZYZ', [0.5 0 0.2]);
th = rotToEulerZYZ(R);
verifyEqual(tc, th(1:2), [0 0]); verifyEqual(tc, th(3), 0.7, 'AbsTol', 1e-12);
R = rotBodyFixed('ZYZ', [0 pi 0.2]);
verifyEqual(tc, rotBodyFixed('ZYZ', rotToEulerZYZ(R)), R, 'AbsTol', 1e-12);
end

% ---------- numerical hygiene ----------
function testOrthonormalizeRestoresSO3(tc)
R = rotBodyFixed('XYZ', [0.3 0.2 0.1]);
Rn = orthonormalize(R + 1e-3 * [1 2 0; -1 0 3; 2 1 -1]);
verifyTrue(tc, isRotationMatrix(Rn, 1e-12));
verifyLessThan(tc, norm(Rn - R, 'fro'), 1e-2);
verifyEqual(tc, orthonormalize(R), R, 'AbsTol', 1e-12);
end

function testIsRotationRejectsReflection(tc)
verifyFalse(tc, isRotationMatrix(diag([1 1 -1])));
verifyFalse(tc, isRotationMatrix(2 * eye(3)));
verifyFalse(tc, isRotationMatrix(eye(2)));
end
