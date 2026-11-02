function [k, phi] = rotToAxisAngle(R)
%ROTTOAXISANGLE Equivalent axis k and angle phi of a rotation matrix.
%
%   [k, phi] = rotToAxisAngle(R)
%
%   Input
%     R: 3x3 rotation matrix.
%   Outputs
%     k: 3x1 unit axis (eigenvector of R for eigenvalue +1).
%     phi: angle in [0, pi], rad.
%
%   Regular case (phi not 0 or pi),:
%     phi = acos((r11+r22+r33-1)/2),  k = [r32-r23; r13-r31; r21-r12]/(2 sin phi)
%   Special cases, engineering choice (flags them as special without formulas):
%     phi ~ 0: axis undefined; k = [0;0;1] is returned by convention.
%     phi ~ pi: k from the largest diagonal of (R + I)/2 = k k'.
arguments
    R (3,3) double {mustBeFinite}
end
c = (trace(R) - 1) / 2;
c = min(1, max(-1, c));              % guard acos against round-off
phi = acos(c);
tolSmall = 1e-10;
if phi < tolSmall
    k = [0; 0; 1];
    phi = 0;
elseif pi - phi < 1e-6
    B = (R + eye(3)) / 2;            % = k k' when phi = pi
    [~, i] = max(diag(B));
    k = B(:, i) / sqrt(B(i, i));
    k = k / norm(k);
else
    k = [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)] / (2 * sin(phi));
    k = k / norm(k);
end
end
