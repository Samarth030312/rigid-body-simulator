function R = rotY(phi)
%ROTY Simple rotation about the Y axis, [R(Y,phi)].
%
%   R = rotY(phi)
%
%   Input
%     phi: rotation angle, rad. Positive = right-hand rule about +Y.
%   Output
%     R: 3x3 rotation matrix; maps ^B r -> ^A r.
%
arguments
    phi (1,1) double {mustBeFinite}
end
c = cos(phi); s = sin(phi);
R = [ c 0 s;
      0 1 0;
     -s 0 c];
end
