function [q, qd, qdd] = reference(t, qStart, qEnd, T)
%REFERENCE Quintic point-to-point joint reference (rest-to-rest).
%
%   [q, qd, qdd] = rbsim.robot.reference(t, qStart, qEnd, T)
%
%   s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5, tau = t/T clipped to [0,1]; zero
%   velocity and acceleration at both ends. Units follow qStart/qEnd (rad).
%   Source: [EXTENSION] (motion planning is not covered by the lectures).
tau = min(max(t / T, 0), 1);
s = 10*tau^3 - 15*tau^4 + 6*tau^5;
sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
sdd = (60*tau - 180*tau^2 + 120*tau^3) / T^2;
if t < 0 || t > T, sd = 0; sdd = 0; end
dq = qEnd(:) - qStart(:);
q = qStart(:) + s * dq;
qd = sd * dq;
qdd = sdd * dq;
end
