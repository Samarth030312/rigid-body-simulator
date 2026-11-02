function vA = pointVelocity(vO_A, omegaS, R_AB, pB, vRelB)
%POINTVELOCITY  Velocity of a point on/in a moving body, components in {A}.
%
%   vA = pointVelocity(vO_A, omegaS, R_AB, pB)          % point fixed in body
%   vA = pointVelocity(vO_A, omegaS, R_AB, pB, vRelB)   % point moving in {B}
%
%   ^A V_p = ^A V_OB + ^A w_B^s x (^A_B[R] ^B p) + ^A_B[R] ^B V_p
%
%   Inputs
%     vO_A: 3x1 velocity of origin O_B in {A}, m/s.
%     omegaS: 3x1 space-fixed angular velocity in {A}, rad/s.
%     R_AB: 3x3 ^A_B[R].
%     pB: 3xN point coordinates in {B}, m.
%     vRelB: 3xN velocity of the point relative to {B}, in {B}, m/s (default 0).
%   Output
%     vA: 3xN, m/s.
%
%   A common slip is to write the middle term as w x ^A p, with ^A p measured
%   from the origin of {A}. The operand is the vector from O_B to the point,
%   expressed in {A}, i.e. ^A_B[R] ^B p: only that part of the position turns
%   with the body. test_angular_velocity pins this against finite differences.
    arguments
        vO_A (3,1) double {mustBeFinite}
        omegaS (3,1) double {mustBeFinite}
        R_AB (3,3) double {mustBeFinite}
        pB (3,:) double {mustBeFinite}
        vRelB (3,:) double {mustBeFinite} = zeros(3, size(pB, 2))
    end
    r = R_AB * pB;
    vA = vO_A + cross(repmat(omegaS, 1, size(r, 2)), r) + R_AB * vRelB;
end
