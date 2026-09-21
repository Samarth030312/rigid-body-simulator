function I_A = inertiaRotate(I_B, R_AB)
%INERTIAROTATE Express an inertia matrix in another frame with the same origin.
%
%   I_A = inertiaRotate(I_B, R_AB)
%
%   ^A[I] = ^A_B[R] ^B[I] ^A_B[R]^T
%
%   Inputs
%     I_B  : 3x3 inertia about point C, components in {B}, kg*m^2.
%     R_AB : 3x3 ^A_B[R].
%   Output
%     I_A  : 3x3 inertia about the same point C, components in {A}.
%
%   Source: [LECTURE] L14-15/s8, L16/s4.
arguments
    I_B (3,3) double {mustBeFinite}
    R_AB (3,3) double {mustBeFinite}
end
I_A = R_AB * I_B * R_AB.';
I_A = 0.5 * (I_A + I_A.');   % remove round-off asymmetry
end
