function [xdot, tauNet] = dynamics(t, x, p, ref)
%DYNAMICS  State derivative of the planar 3R: x = [q; qd] (rad, rad/s).
%
%   [xdot, tauNet] = rbsim.robot.dynamics(t, x, p, ref)
%
%   tauNet (3x1, N m) = actuator torque minus joint damping torque; the net
%   non-conservative power is tauNet' * qd (used for the energy balance).
%
%   M(q) qdd = tau - h(q, qd) - b.* qd,   h = C + G = RNE(q, qd, 0)
%   (form; M from ROBOTMASSMATRIX; EXTENSION: forward dynamics).
    q = x(1:3); qd = x(4:6);
    tau = rbsim.robot.jointTorque(t, x, p, ref) - p.damping(:) .* qd;
    h = rbsim.robot.newtonEuler(q, qd, zeros(3, 1), p);
    qdd = rbsim.robot.massMatrix(q, p) \ (tau - h);
    xdot = [qd; qdd];
    tauNet = tau;
end
