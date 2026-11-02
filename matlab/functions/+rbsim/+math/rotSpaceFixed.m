function R = rotSpaceFixed(seq, angles)
%ROTSPACEFIXED Orientation from successive rotations about space-fixed axes of {A}.
%
%   R = rotSpaceFixed(seq, angles)
%
%   Inputs
%     seq    : axis letters, e.g. 'XYZ'. angles(1) about seq(1) is applied first.
%     angles : vector (rad), same length as seq.
%   Output
%     R      : 3x3 ^A_B[R]. Space-fixed rotations PRE-multiply:
%                R = R(seq(n),angles(n)) * ... * R(seq(1),angles(1))
%              e.g. space-fixed X-Y-Z -> R = R(Z,t3) R(Y,t2) R(X,t1).
%
%   Source: [LECTURE] L2-4/s32-36 (the lecture derives this with the (k,phi) form
%   by expressing each fixed axis in the current body frame; final result s36).
%   Equivalence with body-fixed reversed sequence: L2-4/s59.
%   See also ROTBODYFIXED.
arguments
    seq {mustBeTextScalar}
    angles (1,:) double {mustBeFinite}
end
seq = upper(char(seq));
R = rbsim.math.rotBodyFixed(fliplr(seq), fliplr(angles));
end
