function R = rotZ(phi)
%ROTZ Simple rotation about the Z axis, [R(Z,phi)].
%
%   R = rotZ(phi)
%
%   Input
%     phi: rotation angle, rad. Positive = right-hand rule about +Z.
%   Output
%     R: 3x3 rotation matrix; maps ^B r -> ^A r.
%
arguments
    phi (1,1) double {mustBeFinite}
end
c = cos(phi); s = sin(phi);
R = [c -s 0;
     s  c 0;
     0  0 1];
end
