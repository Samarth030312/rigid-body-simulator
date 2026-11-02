function theta = rotToEulerXYZ(R)
%ROTTOEULERXYZ Body-fixed X-Y-Z Euler angles from a rotation matrix.
%
%   theta = rotToEulerXYZ(R)
%
%   Input
%     R     : 3x3 ^A_B[R] = R(X,t1) R(Y,t2) R(Z,t3).
%   Output
%     theta : 1x3 [t1 t2 t3], rad, each in [-pi, pi]. The '+sqrt' branch is
%             returned (cos t2 >= 0) [EXTENSION: branch choice]; the lecture notes
%             that two solution sets exist.
%
%   Source: [LECTURE] L2-4/s26 (regular case), s27 (r13 = +-1 singular case:
%   t3 = 0, t1 = +-atan2(r21, r22)).
arguments
    R (3,3) double {mustBeFinite}
end
r13 = R(1,3);
tolSing = 1e-12;
if abs(abs(r13) - 1) > tolSing
    t2 = atan2(r13, sqrt(R(1,1)^2 + R(1,2)^2));
    c2 = cos(t2);
    t1 = atan2(-R(2,3) / c2, R(3,3) / c2);
    t3 = atan2(-R(1,2) / c2, R(1,1) / c2);
elseif r13 > 0
    t1 = atan2(R(2,1), R(2,2)); t2 = pi/2; t3 = 0;
else
    t1 = -atan2(R(2,1), R(2,2)); t2 = -pi/2; t3 = 0;
end
theta = [t1 t2 t3];
end
