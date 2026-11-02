function R = rotBodyFixed(seq, angles)
%ROTBODYFIXED Orientation from successive rotations about body-fixed (moving) axes.
%
%   R = rotBodyFixed(seq, angles)
%
%   Inputs
%     seq    : char/string of axis letters, e.g. 'XYZ' (X-Y-Z Euler angles) or
%              'ZYZ' (classical Euler angles). Letters from {X,Y,Z}; consecutive
%              letters must differ.
%     angles : vector of the same length as seq, rad. angles(1) is applied first.
%   Output
%     R      : 3x3 ^A_B[R]. Body-fixed rotations POST-multiply:
%                R = R(seq(1),angles(1)) * R(seq(2),angles(2)) * ...
%
%   Source: [LECTURE] L2-4/s21 (order of multiplication), s24-25 (X-Y-Z), s30 (Z-Y-Z).
%   See also ROTSPACEFIXED.
arguments
    seq {mustBeTextScalar}
    angles (1,:) double {mustBeFinite}
end
seq = upper(char(seq));
validateSequence(seq, angles);
R = eye(3);
for i = 1:numel(seq)
    R = R * simpleRotation(seq(i), angles(i));
end
end

function validateSequence(seq, angles)
if numel(seq) ~= numel(angles)
    error('rbsim:math:sizeMismatch', 'seq and angles must have equal length.');
end
if any(~ismember(seq, 'XYZ'))
    error('rbsim:math:badAxis', 'Axis letters must be X, Y or Z.');
end
if any(seq(1:end-1) == seq(2:end))
    error('rbsim:math:repeatedAxis', 'Consecutive rotation axes must differ.');
end
end

function R = simpleRotation(ax, phi)
switch ax
    case 'X', R = rbsim.math.rotX(phi);
    case 'Y', R = rbsim.math.rotY(phi);
    case 'Z', R = rbsim.math.rotZ(phi);
end
end
