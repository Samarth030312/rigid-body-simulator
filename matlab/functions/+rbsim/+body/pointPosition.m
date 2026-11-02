function pA = pointPosition(O_A, R_AB, pB)
%POINTPOSITION  World position of body points: ^A p = ^A O_B + ^A_B[R] ^B p.
%
%   pA = pointPosition(O_A, R_AB, pB)
%
%   Inputs
%     O_A: 3x1 position of the body-frame origin O_B, components in {A}, m.
%     R_AB: 3x3 ^A_B[R].
%     pB: 3xN body-fixed point coordinates in {B}, m.
%   Output
%     pA: 3xN point positions in {A}, m.
%
    arguments
        O_A (3,1) double {mustBeFinite}
        R_AB (3,3) double {mustBeFinite}
        pB (3,:) double {mustBeFinite}
    end
    pA = O_A + R_AB * pB;
end
