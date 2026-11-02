function p = defaults()
%DEFAULTS  Default parameters of the planar 3R manipulator (SI units).
%
%   p = rbsim.robot.defaults
%
%   Planar 3R: frames {i} sit at joint O_i with X_i along link i, the joint
%   angles are relative and the joint axes point out of the page. Each link is
%   described by its length l_i, mass m_i, centre-of-mass offset r_i and the
%   inertia I_i about that centre of mass.
%
%   Fields
%.l      1x3 link lengths, m
%.m      1x3 link masses, kg
%.r      1x3 COM distance from joint i along X_i, m
%.width  1x3 link cross-section size, m (used only for the inertia default)
%.Izz    1x3 inertia of link i about its COM, about Z, kg m^2
%             default: slender box, m (l^2 + w^2)/12
%.g      gravity magnitude, m/s^2, acting along -Y_0
%.damping 1x3 viscous joint damping b_i, N m s/rad (torsional damper,
%              element; EXTENSION as a joint model)
%.q0,.qd0  1x3 initial joint angles (DEG) and rates (DEG/s) - user-facing
%.control struct, see rbsim.robot.jointTorque:.mode 'free' | 'constant' | 'pd' |
%             'trajectory',.tau (1x3 N m),.qTargetDeg,.Kp,.Kd,
%.gravityComp (logical),.duration (s, trajectory)
%.tFinal,.dt,.outputEvery  integration settings (RK4, ADR-0009)
    p.l = [0.5 0.4 0.3];
    p.m = [3.0 2.0 1.0];
    p.r = p.l / 2;
    p.width = [0.05 0.04 0.03];
    p.Izz = p.m .* (p.l.^2 + p.width.^2) / 12;
    p.g = 9.81;
    p.damping = [0 0 0];
    p.q0 = [30 -20 10];
    p.qd0 = [0 0 0];
    p.control = struct('mode', 'free', 'tau', [0 0 0], 'qTargetDeg', [90 -45 -30], ...
        'Kp', [200 120 40], 'Kd', [40 20 6], 'gravityComp', true, 'duration', 2);
    p.tFinal = 5;
    p.dt = 1e-3;
    p.outputEvery = 10;
end
