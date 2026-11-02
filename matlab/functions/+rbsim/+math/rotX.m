function R = rotX(phi)
%ROTX Simple rotation about the X axis, [R(X,phi)].
%
%   R = rotX(phi)
%
%   Input
%     phi: rotation angle, rad. Positive = right-hand rule about +X.
%   Output
%     R: 3x3 rotation matrix ^A_B[R] of a frame {B} obtained by rotating {A}
%           about X_A by phi. Maps ^B r -> ^A r.
%
arguments
    phi (1,1) double {mustBeFinite}
end
c = cos(phi); s = sin(phi);
R = [1 0 0;
     0 c -s;
     0 s  c];
end
