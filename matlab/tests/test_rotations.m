function tests = test_rotations
%TEST_ROTATIONS  Level 1 validation of rotation-matrix primitives (core/math).
%   Reference values are closed-form matrices evaluated independently.
    tests = functiontests(localfunctions);
end

function setupOnce(tc)
    tc.TestData.tol = 1e-12;       % algebraic identities
    tc.TestData.tolRef = 5e-5; % reference matrices quoted to 4 decimals
    rng(209, 'twister');           % deterministic random samples
    tc.TestData.angles = (rand(50, 3) - 0.5) * 2 * pi;
end

% ---------- simple rotations ----------
function testSimpleRotationsAreProperOrthonormal(tc)
    for phi = linspace(-2*pi, 2*pi, 25)
        for R = {rbsim.math.rotX(phi), rbsim.math.rotY(phi), rbsim.math.rotZ(phi)}
            tc.verifyTrue(rbsim.math.isRotationMatrix(R{1}, 1e-12));
        end
    end
end

function testSimpleRotationRightHandRule(tc)
% +90 deg about Z maps X_B onto Y_A (counter-clockwise about +Z).
    tc.verifyEqual(rbsim.math.rotZ(pi/2) * [1;0;0], [0;1;0], 'AbsTol', tc.TestData.tol);
    tc.verifyEqual(rbsim.math.rotX(pi/2) * [0;1;0], [0;0;1], 'AbsTol', tc.TestData.tol);
    tc.verifyEqual(rbsim.math.rotY(pi/2) * [0;0;1], [1;0;0], 'AbsTol', tc.TestData.tol);
end

function testRotationOfFortyFiveDegreesAboutZ(tc)
    Rexp = [1/sqrt(2) -1/sqrt(2) 0; 1/sqrt(2) 1/sqrt(2) 0; 0 0 1];
    tc.verifyEqual(rbsim.math.rotZ(pi/4), Rexp, 'AbsTol', tc.TestData.tol);
end

function testColumnsAreBodyAxesInA(tc)
%: columns of ^A_B[R] are X_B, Y_B, Z_B described in {A}.
    R = rbsim.math.rotBodyFixed('XYZ', [0.3 -0.7 1.1]);
    tc.verifyEqual(R * [1;0;0], R(:,1), 'AbsTol', tc.TestData.tol);
    tc.verifyEqual(R * [0;0;1], R(:,3), 'AbsTol', tc.TestData.tol);
end

function testInverseIsTranspose(tc)
    for i = 1:size(tc.TestData.angles, 1)
        R = rbsim.math.rotBodyFixed('ZYZ', tc.TestData.angles(i,:));
        tc.verifyEqual(R \ eye(3), R.', 'AbsTol', 1e-12);
        tc.verifyEqual(det(R), 1, 'AbsTol', 1e-12);
    end
end

% ---------- composition  ----------
function testBodyFixedXYZAgainstItsClosedForm(tc)
%  and s59: body-fixed X-Y-Z 60, 45, -75 deg.
    R = rbsim.math.rotBodyFixed('XYZ', deg2rad([60 45 -75]));
    Rexp = [0.1830 0.6830 0.7071; -0.3245 0.7209 -0.6124; -0.9280 -0.1174 0.3536];
    tc.verifyEqual(R, Rexp, 'AbsTol', tc.TestData.tolRef);
end

function testSpaceFixedXYZAgainstItsClosedForm(tc)
%: space-fixed X-Y-Z 60, 45, -75 deg.
    R = rbsim.math.rotSpaceFixed('XYZ', deg2rad([60 45 -75]));
    Rexp = [0.1830 0.6415 -0.7450; -0.6830 -0.4621 -0.5657; -0.7071 0.6124 0.3536];
    tc.verifyEqual(R, Rexp, 'AbsTol', tc.TestData.tolRef);
end

function testBodyFixedZYZAgainstItsClosedForm(tc)
%: body-fixed Z-Y-Z 30, 60, 90 deg.
    R = rbsim.math.rotBodyFixed('ZYZ', deg2rad([30 60 90]));
    Rexp = [-0.5 -0.4330 0.75; 0.8660 -0.25 0.4330; 0 0.8660 0.5];
    tc.verifyEqual(R, Rexp, 'AbsTol', tc.TestData.tolRef);
end

function testBodyFixedXYZMatrixEntries(tc)
% Closed-form X-Y-Z matrix printed on.
    th = [0.4 -1.2 2.5];
    c = cos(th); s = sin(th);
    Rexp = [c(2)*c(3), -c(2)*s(3), s(2);
            s(1)*s(2)*c(3)+s(3)*c(1), -s(1)*s(2)*s(3)+c(3)*c(1), -s(1)*c(2);
           -c(1)*s(2)*c(3)+s(3)*s(1),  c(1)*s(2)*s(3)+c(3)*s(1),  c(1)*c(2)];
    tc.verifyEqual(rbsim.math.rotBodyFixed('XYZ', th), Rexp, 'AbsTol', tc.TestData.tol);
end

function testSpaceFixedXYZMatrixEntries(tc)
% Closed-form space-fixed X-Y-Z matrix printed on.
    th = [0.4 -1.2 2.5];
    c = cos(th); s = sin(th);
    Rexp = [c(2)*c(3), s(1)*s(2)*c(3)-c(1)*s(3), c(1)*s(2)*c(3)+s(1)*s(3);
            c(2)*s(3), s(1)*s(2)*s(3)+c(1)*c(3), c(1)*s(2)*s(3)-s(1)*c(3);
           -s(2),      s(1)*c(2),                c(1)*c(2)];
    tc.verifyEqual(rbsim.math.rotSpaceFixed('XYZ', th), Rexp, 'AbsTol', tc.TestData.tol);
end

function testSpaceFixedByAxisAngleMethod(tc)
% Reproduce the derivation: each later rotation is about
% the FIXED axis expressed in the current body frame, then post-multiplied.
    th = [0.7 -0.4 1.3];
    R1 = rbsim.math.rotX(th(1));
    kY = R1.' * [0;1;0];                   % ^{B1} Y_A
    R2 = R1 * rbsim.math.rotAxisAngle(kY, th(2));
    kZ = R2.' * [0;0;1];                   % ^{B2} Z_A
    R3 = R2 * rbsim.math.rotAxisAngle(kZ, th(3));
    tc.verifyEqual(rbsim.math.rotSpaceFixed('XYZ', th), R3, 'AbsTol', 1e-12);
end

function testBodyXYZEqualsSpaceZYX(tc)
    th = deg2rad([60 45 -75]);
    tc.verifyEqual(rbsim.math.rotBodyFixed('XYZ', th), ...
        rbsim.math.rotSpaceFixed('ZYX', fliplr(th)), 'AbsTol', tc.TestData.tol);
end

function testCompositionIsNotCommutative(tc)
%: order of rotation is important.
    A = rbsim.math.rotX(0.5); B = rbsim.math.rotY(0.8);
    tc.verifyGreaterThan(norm(A*B - B*A, 'fro'), 0.1);
end

function testRepeatedAxisRejected(tc)
    tc.verifyError(@() rbsim.math.rotBodyFixed('XXY', [1 2 3]), 'rbsim:math:repeatedAxis');
    tc.verifyError(@() rbsim.math.rotBodyFixed('XY', [1 2 3]), 'rbsim:math:sizeMismatch');
end

% ---------- axis-angle  ----------
function testAxisAngleMatchesSimpleRotations(tc)
    phi = 0.83;
    tc.verifyEqual(rbsim.math.rotAxisAngle([1;0;0], phi), rbsim.math.rotX(phi), 'AbsTol', tc.TestData.tol);
    tc.verifyEqual(rbsim.math.rotAxisAngle([0;1;0], phi), rbsim.math.rotY(phi), 'AbsTol', tc.TestData.tol);
    tc.verifyEqual(rbsim.math.rotAxisAngle([0;0;2], phi), rbsim.math.rotZ(phi), 'AbsTol', tc.TestData.tol);
end

function testRodriguesFormula(tc)
%: Q' = Q cos + (k x Q) sin + (1-cos)(k.Q) k
    k = [1; 2; -2] / 3; th = 1.1; Q = [0.3; -1.4; 2.2];
    Qp = Q*cos(th) + cross(k, Q)*sin(th) + (1 - cos(th))*dot(k, Q)*k;
    tc.verifyEqual(rbsim.math.rotAxisAngle(k, th) * Q, Qp, 'AbsTol', tc.TestData.tol);
end

function testAxisAngleRoundTrip(tc)
    for i = 1:size(tc.TestData.angles, 1)
        R = rbsim.math.rotBodyFixed('XYZ', tc.TestData.angles(i,:));
        [k, phi] = rbsim.math.rotToAxisAngle(R);
        tc.verifyEqual(rbsim.math.rotAxisAngle(k, phi), R, 'AbsTol', 1e-9);
        tc.verifyEqual(R * k, k, 'AbsTol', 1e-9);   % : k is fixed
    end
end

function testAxisAngleSpecialCases(tc)
    [k, phi] = rbsim.math.rotToAxisAngle(eye(3));
    tc.verifyEqual(phi, 0); tc.verifyEqual(norm(k), 1, 'AbsTol', 1e-12);
    kIn = [1; -1; 2] / sqrt(6);
    [k, phi] = rbsim.math.rotToAxisAngle(rbsim.math.rotAxisAngle(kIn, pi));
    tc.verifyEqual(phi, pi, 'AbsTol', 1e-9);
    tc.verifyEqual(abs(dot(k, kIn)), 1, 'AbsTol', 1e-9);   % axis sign is free at pi
end

function testEigenvaluesOfR(tc)
%: eigenvalues +1, exp(+-i phi) with phi = acos((trace-1)/2).
    R = rbsim.math.rotBodyFixed('XYZ', [0.2 0.9 -0.4]);
    phi = acos((trace(R) - 1)/2);
    ev = sort(eig(R), 'ComparisonMethod', 'real');
    tc.verifyEqual(abs(ev), ones(3,1), 'AbsTol', 1e-12);
    tc.verifyEqual(sort(angle(ev)), [-phi; 0; phi], 'AbsTol', 1e-12);
end

% ---------- inverse Euler  ----------
function testEulerXYZRoundTrip(tc)
    for i = 1:size(tc.TestData.angles, 1)
        th = tc.TestData.angles(i,:);
        R = rbsim.math.rotBodyFixed('XYZ', th);
        thBack = rbsim.math.rotToEulerXYZ(R);
        tc.verifyEqual(rbsim.math.rotBodyFixed('XYZ', thBack), R, 'AbsTol', 1e-10);
    end
end

function testEulerXYZRecoversKnownAngles(tc)
    th = deg2rad([60 45 -75]);
    tc.verifyEqual(rbsim.math.rotToEulerXYZ(rbsim.math.rotBodyFixed('XYZ', th)), th, 'AbsTol', 1e-12);
end

function testEulerXYZSingular(tc)
%: theta2 = +-pi/2 -> theta3 = 0 convention, R still reproduced.
    for t2 = [pi/2, -pi/2]
        R = rbsim.math.rotBodyFixed('XYZ', [0.4 t2 0.3]);
        th = rbsim.math.rotToEulerXYZ(R);
        tc.verifyEqual(th(3), 0);
        tc.verifyEqual(rbsim.math.rotBodyFixed('XYZ', th), R, 'AbsTol', 1e-10);
    end
end

function testEulerZYZRoundTrip(tc)
    for i = 1:size(tc.TestData.angles, 1)
        th = tc.TestData.angles(i,:);
        R = rbsim.math.rotBodyFixed('ZYZ', th);
        tc.verifyEqual(rbsim.math.rotBodyFixed('ZYZ', rbsim.math.rotToEulerZYZ(R)), R, 'AbsTol', 1e-10);
    end
end

function testEulerZYZSingular(tc)
%: r33 = +-1 cases.
    R = rbsim.math.rotBodyFixed('ZYZ', [0.5 0 0.2]);
    th = rbsim.math.rotToEulerZYZ(R);
    tc.verifyEqual(th(1:2), [0 0]); tc.verifyEqual(th(3), 0.7, 'AbsTol', 1e-12);
    R = rbsim.math.rotBodyFixed('ZYZ', [0 pi 0.2]);
    tc.verifyEqual(rbsim.math.rotBodyFixed('ZYZ', rbsim.math.rotToEulerZYZ(R)), R, 'AbsTol', 1e-12);
end

% ---------- numerical hygiene ----------
function testOrthonormalizeRestoresSO3(tc)
    R = rbsim.math.rotBodyFixed('XYZ', [0.3 0.2 0.1]);
    Rn = rbsim.math.orthonormalize(R + 1e-3 * [1 2 0; -1 0 3; 2 1 -1]);
    tc.verifyTrue(rbsim.math.isRotationMatrix(Rn, 1e-12));
    tc.verifyLessThan(norm(Rn - R, 'fro'), 1e-2);
    tc.verifyEqual(rbsim.math.orthonormalize(R), R, 'AbsTol', 1e-12);
end

function testIsRotationRejectsReflection(tc)
    tc.verifyFalse(rbsim.math.isRotationMatrix(diag([1 1 -1])));
    tc.verifyFalse(rbsim.math.isRotationMatrix(2 * eye(3)));
    tc.verifyFalse(rbsim.math.isRotationMatrix(eye(2)));
end
