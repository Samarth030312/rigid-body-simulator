function R = rotAxisAngle(k, phi)
%ROTAXISANGLE Rotation matrix [R(k,phi)] for a rotation phi about unit axis k.
%
%   R = rotAxisAngle(k, phi)
%
%   Inputs
%     k: 3x1 rotation axis (components in {A}). Normalised internally; must be
%           non-zero.
%     phi: rotation angle, rad (right-hand rule about k).
%   Output
%     R: 3x3 rotation matrix. R*Q equals Rodrigues' rotation of Q about k by phi.
%
%   Rodrigues' rotation formula.  (element formulas r11..r33 on s18).
%   See also ROTTOAXISANGLE.
arguments
    k (3,1) double {mustBeFinite}
    phi (1,1) double {mustBeFinite}
end
n = norm(k);
if n < eps
    error('rbsim:math:zeroAxis', 'Rotation axis must be non-zero.');
end
k = k / n;
kx = k(1); ky = k(2); kz = k(3);
c = cos(phi); s = sin(phi); v = 1 - c;
R = [kx*kx*v + c,     kx*ky*v - kz*s,  kz*kx*v + ky*s;
     kx*ky*v + kz*s,  ky*ky*v + c,     ky*kz*v - kx*s;
     kz*kx*v - ky*s,  ky*kz*v + kx*s,  kz*kz*v + c   ];
end
