function theta = rotToEulerZYZ(R)
%ROTTOEULERZYZ Body-fixed Z-Y-Z Euler angles from a rotation matrix.
%
%   theta = rotToEulerZYZ(R)
%
%   Input
%     R: 3x3 ^A_B[R] = R(Z,t1) R(Y,t2) R(Z,t3).
%   Output
%     theta: 1x3 [t1 t2 t3], rad. '+sqrt' branch (sin t2 >= 0) engineering choice of branch.
%
%   Source:, including the singular cases
%     r33 = 1: t1 = t2 = 0, t3 = atan2(-r12, r11)
%     r33 = -1: t1 = 0, t2 = pi, t3 = atan2(r12, -r11)
arguments
    R (3,3) double {mustBeFinite}
end
r33 = R(3,3);
tolSing = 1e-12;
if abs(abs(r33) - 1) > tolSing
    t2 = atan2(sqrt(R(3,1)^2 + R(3,2)^2), r33);
    s2 = sin(t2);
    t1 = atan2(R(2,3) / s2, R(1,3) / s2);
    t3 = atan2(R(3,2) / s2, -R(3,1) / s2);
elseif r33 > 0
    t1 = 0; t2 = 0; t3 = atan2(-R(1,2), R(1,1));
else
    t1 = 0; t2 = pi; t3 = atan2(R(1,2), -R(1,1));
end
theta = [t1 t2 t3];
end
