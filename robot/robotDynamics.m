function [xdot, tauNet] = robotDynamics(t, x, p, ref)
%ROBOTDYNAMICS State derivative of the planar 3R: x = [q; qd] (rad, rad/s).
%
%   [xdot, tauNet] = robotDynamics(t, x, p, ref)
%
%   tauNet (3x1, N m) = actuator torque minus joint damping torque; the net
%   non-conservative power is tauNet' * qd (used for the energy balance).
%
%   M(q) qdd = tau - h(q, qd) - b .* qd,   h = C + G = RNE(q, qd, 0)
%   (L20-21/s10-11, s19 form; M from ROBOTMASSMATRIX; EXTENSION: forward dynamics).
q = x(1:3); qd = x(4:6);
tau = robotJointTorque(t, x, p, ref) - p.damping(:) .* qd;
h = robotNewtonEuler(q, qd, zeros(3, 1), p);
qdd = robotMassMatrix(q, p) \ (tau - h);
xdot = [qd; qdd];
tauNet = tau;
end
