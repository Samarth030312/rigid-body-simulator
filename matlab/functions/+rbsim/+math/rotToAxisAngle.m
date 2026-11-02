function [k, phi] = rotToAxisAngle(R)
%ROTTOAXISANGLE  Equivalent axis k and angle phi of a rotation matrix.
%
%   [k, phi] = rotToAxisAngle(R)
%
%   Input
%     R: 3x3 rotation matrix.
%   Outputs
%     k: 3x1 unit axis (eigenvector of R for eigenvalue +1).
%     phi: angle in [0, pi], rad.
%
%   Split R into its symmetric and antisymmetric parts. The antisymmetric
%   part carries sin(phi)*k and the trace carries cos(phi):
%     v = [r32-r23; r13-r31; r21-r12]/2 = sin(phi) k,   s = |v| = sin(phi)
%     c = (r11+r22+r33-1)/2             = cos(phi)
%     phi = atan2(s, c),                k = v/s
%
%   The angle comes from atan2 rather than from acos(c), which loses half
%   the significant digits near phi = 0 and phi = pi. There c = -1+d^2/2 for
%   phi = pi-d, so a round-off e in the trace moves the recovered angle by
%   sqrt(2e): about 3e-8 for e = 4e-16. atan2 reads the angle off the sine
%   instead, whose round-off passes through unamplified, and the recovered
%   angle is good to a few eps across the whole range.
%
%   The axis still degenerates: v vanishes with sin(phi), so v/s has relative
%   error about eps/s, while at phi = pi the symmetric part gives
%     (R + I)/2 = k k'
%   whose axis is in error by about s (the leading term dropped from R). The
%   two are equally good at s = sqrt(eps), which is where this switches, and
%   both deliver the axis to about 1e-8 there. No choice of formula does
%   better: at phi = 0 the axis does not exist, and near it the rotation
%   itself barely distinguishes one axis from another.
%
%   Conventions, both engineering choices at the degeneracies:
%     phi ~ 0:  axis undefined; k = [0;0;1] is returned. The angle is not
%               snapped to zero with it: atan2 reports a small phi faithfully
%               and is exactly zero for R = I, so a caller measuring a drift
%               of 1e-8 rad still sees 1e-8 rad.
%     phi ~ pi: R(k) = R(-k), so the sign of k is free; the branch below
%               returns whichever sign the largest column of (R+I)/2 carries.
    arguments
        R (3,3) double {mustBeFinite}
    end
    v = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] / 2;   % sin(phi) k
    s = norm(v);                                                    % sin(phi)
    c = (trace(R) - 1) / 2;                                         % cos(phi)
    phi = atan2(s, c);                   % exactly 0 for R = I, no snapping needed
    if s > sqrt(eps)
        k = v / s;
    elseif c > 0                         % phi ~ 0: no axis to recover
        k = [0; 0; 1];
    else                                 % phi ~ pi: (R + I)/2 = k k'
        B = (R + eye(3)) / 2;
        [~, i] = max(diag(B));
        k = B(:, i) / sqrt(B(i, i));
    end
    k = k / norm(k);
end
