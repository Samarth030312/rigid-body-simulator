function S = skew(w)
%SKEW Skew-symmetric matrix [Omega] of a 3-vector, so that S*p = cross(w,p).
%
%   S = skew(w)
%
%   Input
%     w : 3x1 (or 1x3) vector, any units, components in some frame {F}.
%   Output
%     S : 3x3 skew-symmetric matrix in the same frame {F}:
%           S = [ 0   -wz   wy
%                 wz   0   -wx
%                -wy   wx   0 ]
%
%   Source: [LECTURE] L7-9/s6 (layout of the angular velocity matrix).
%   See also UNSKEW.
arguments
    w (3,1) double {mustBeFinite}
end
S = [ 0    -w(3)  w(2);
      w(3)  0    -w(1);
     -w(2)  w(1)  0   ];
end
