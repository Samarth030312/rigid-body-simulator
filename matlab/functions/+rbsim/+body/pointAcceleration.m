function aA = pointAcceleration(aO_A, alpha, omegaS, R_AB, pB, vRelB, aRelB)
%POINTACCELERATION Acceleration of a point on/in a moving body, components in {A}.
%
%   aA = pointAcceleration(aO_A, alpha, omegaS, R_AB, pB)
%   aA = pointAcceleration(aO_A, alpha, omegaS, R_AB, pB, vRelB, aRelB)
%
%   ^A a_p = ^A a_OB                         (acceleration of origin)
%          + alpha x r                       (tangential)
%          + w x (w x r)                     (centripetal)
%          + 2 w x (^A_B[R] ^B V_p)          (Coriolis)
%          + ^A_B[R] ^B a_p                  (relative acceleration in {B})
%   with r = ^A_B[R] ^B p, w = ^A w_B^s, alpha = d/dt(^A w_B^s).
%
%   Inputs: vectors in {A} except pB, vRelB, aRelB which are in {B}. Units SI.
%   Output: aA 3xN, m/s^2.
%
%   Source:  (five-term expression, term names on s24).
arguments
    aO_A (3,1) double {mustBeFinite}
    alpha (3,1) double {mustBeFinite}
    omegaS (3,1) double {mustBeFinite}
    R_AB (3,3) double {mustBeFinite}
    pB (3,:) double {mustBeFinite}
    vRelB (3,:) double {mustBeFinite} = zeros(3, size(pB, 2))
    aRelB (3,:) double {mustBeFinite} = zeros(3, size(pB, 2))
end
n = size(pB, 2);
r = R_AB * pB;
W = repmat(omegaS, 1, n);
aA = aO_A + cross(repmat(alpha, 1, n), r) + cross(W, cross(W, r)) ...
    + 2 * cross(W, R_AB * vRelB) + R_AB * aRelB;
end
