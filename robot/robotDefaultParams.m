function p = robotDefaultParams()
%ROBOTDEFAULTPARAMS Default parameters of the planar 3R manipulator (SI units).
%
%   p = robotDefaultParams()
%
%   Lecture model: planar 3R of L7-9/s49 (frames {i} at joint O_i, X_i along
%   link i, relative joint angles, joint axes out of the page) with the link
%   parameter set of L20-21/s13 (l_i, m_i, r_i, I_i).
%
%   Fields
%     .l      1x3 link lengths, m
%     .m      1x3 link masses, kg
%     .r      1x3 COM distance from joint i along X_i, m
%     .width  1x3 link cross-section size, m (used only for the inertia default)
%     .Izz    1x3 inertia of link i about its COM, about Z, kg m^2
%             default: slender box, m (l^2 + w^2)/12 (L14-15/s9)
%     .g      gravity magnitude, m/s^2, acting along -Y_0 (L20-21/s14)
%     .damping 1x3 viscous joint damping b_i, N m s/rad (torsional damper,
%             L10-13/s6 element; EXTENSION as a joint model)
%     .q0, .qd0  1x3 initial joint angles (DEG) and rates (DEG/s) - user-facing
%     .control struct, see ROBOTJOINTTORQUE: .mode 'free' | 'constant' | 'pd' |
%             'trajectory', .tau (1x3 N m), .qTargetDeg, .Kp, .Kd,
%             .gravityComp (logical), .duration (s, trajectory)
%     .tFinal, .dt, .outputEvery  integration settings (RK4, ADR-0009)
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
