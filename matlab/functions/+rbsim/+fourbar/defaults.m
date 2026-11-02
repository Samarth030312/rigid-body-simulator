function p = defaults()
%DEFAULTS Default four-bar mechanism parameters (SI units).
%
%   p = rbsim.fourbar.defaults
%
%   Geometry and notation of: ground link l0 from O_L (origin of {0})
%   to O_R = (l0, 0); crank l1 (link 1, input angle theta1 from X_L); coupler l2
%   (link 2); output/rocker l3 (link 3, angle phi1 from X_R). All angles CCW.
%
%   Fields
%.l          [l0 l1 l2 l3], m
%.m          [m1 m2 m3] link masses, kg
%.r          [r1 r2 r3] COM distance along each link from its first joint:
%                 link 1 from O_L, link 2 from A (crank pin), link 3 from O_R, m
%.width      [w1 w2 w3] m (inertia default only)
%.Izz        [I1 I2 I3] about each COM, kg m^2 (default slender box,)
%.g          gravity magnitude along -Y_0, m/s^2
%.branch     +1 'open' (configuration of), -1 'crossed'
%.theta0Deg,.thetaDot0Deg  initial crank angle (deg) and rate (deg/s)
%.drive      struct:.mode 'free' | 'torque' | 'speed'
%.torque (N m, constant crank torque, 'torque' mode)
%.speedDeg (deg/s, constant crank speed, 'speed' mode: kinematic
%                 drive, required torque from inverse dynamics)
%.loadTorque (N m on link 3, e.g. resisting load)
%.damping (N m s/rad, viscous at the crank bearing)
%.tFinal,.dt,.outputEvery
p.l = [0.40 0.12 0.36 0.28];
p.m = [0.30 0.80 0.60];
p.r = p.l(2:4) / 2;
p.width = [0.02 0.02 0.02];
p.Izz = p.m .* (p.l(2:4).^2 + p.width.^2) / 12;
p.g = 9.81;
p.branch = 1;
p.theta0Deg = 60;
p.thetaDot0Deg = 360;
p.drive = struct('mode', 'free', 'torque', 0.5, 'speedDeg', 360, ...
    'loadTorque', 0, 'damping', 0);
p.tFinal = 3;
p.dt = 1e-3;
p.outputEvery = 5;
end
