function tests = test_inertia
%TEST_INERTIA Level 3 validation of mass properties (L14-15).
%   Formula results are compared with a direct discretisation of the lecture
%   definition ^A[I]_C = -sum m_k [r_k][r_k] (L14-15/s6), which independently
%   checks the sign convention of products of inertia.
tests = functiontests(localfunctions);
end

function [I, pts, mk] = gridInertia(a, b, c, m, n, R, O)
% Midpoint-rule discretisation of a uniform box (n^3 cells), rotated by R and
% offset so that the inertia is taken about point O (both in {A}).
[ix, iy, iz] = ndgrid(((1:n) - 0.5)/n - 0.5);
P = [a*ix(:), b*iy(:), c*iz(:)].';
pts = R * P;
mk = m / size(P, 2);
I = zeros(3);
for k = 1:size(pts, 2)
    S = skew(pts(:, k) - O);
    I = I - mk * (S * S);
end
end

function testCubeIsotropic(tc)
verifyEqual(tc, inertiaCube(0.006, 0.016), (0.006*0.016^2/6) * eye(3), 'RelTol', 1e-14);
end

function testBoxMatchesDefinition(tc)
% Midpoint rule error for x^2 over a cell is O(1/n^2) relative.
m = 2; a = 0.3; b = 0.2; c = 0.1;
Igrid = gridInertia(a, b, c, m, 40, eye(3), [0;0;0]);
verifyEqual(tc, Igrid, inertiaBox(m, a, b, c), 'RelTol', 1e-3, 'AbsTol', 1e-12);
end

function testRotationRuleMatchesDefinition(tc)
% L14-15/s8: ^A[I] = R ^B[I] R'. Rotated box gives non-zero products of inertia
% whose SIGN must match -sum m x y.
m = 2; a = 0.3; b = 0.2; c = 0.1;
R = rotBodyFixed('XYZ', [0.4 -0.3 0.9]);
Igrid = gridInertia(a, b, c, m, 40, R, [0;0;0]);
IA = inertiaRotate(inertiaBox(m, a, b, c), R);
verifyEqual(tc, IA, Igrid, 'AbsTol', 1e-3 * max(abs(IA(:))));
verifyNotEqual(tc, sign(IA(1,2)), 0);
end

function testParallelAxisMatchesDefinition(tc)
m = 2; a = 0.3; b = 0.2; c = 0.1; O = [0.5; -0.2; 0.3];
Igrid = gridInertia(a, b, c, m, 40, eye(3), O);
IO = inertiaParallelAxis(inertiaBox(m, a, b, c), m, O);
verifyEqual(tc, IO, Igrid, 'AbsTol', 1e-3 * max(abs(IO(:))));
% Corrected (3,3) entry (SD-05): Izz,O = Izz,C + m (dx^2 + dy^2)
Ic = inertiaBox(m, a, b, c);
verifyEqual(tc, IO(3,3), Ic(3,3) + m*(O(1)^2 + O(2)^2), 'RelTol', 1e-14);
verifyEqual(tc, IO(1,2), -m*O(1)*O(2), 'RelTol', 1e-14);
end

function testPerpendicularAxisThinPlate(tc)
% L14-15/s8: I_zz = I_xx + I_yy for a planar body (c -> 0).
I = inertiaBox(1.5, 0.4, 0.25, 1e-9);
verifyEqual(tc, I(3,3), I(1,1) + I(2,2), 'RelTol', 1e-12);
end

function testSymmetricPositiveDefinite(tc)
% L14-15/s17
I = inertiaRotate(inertiaBox(3, 0.5, 0.2, 0.1), rotBodyFixed('ZYZ', [1 2 3]));
verifyEqual(tc, I, I.', 'AbsTol', 1e-15);
verifyGreaterThan(tc, min(eig(I)), 0);
verifyEqual(tc, sort(eig(I)), sort(diag(inertiaBox(3, 0.5, 0.2, 0.1))), 'RelTol', 1e-12);
end
