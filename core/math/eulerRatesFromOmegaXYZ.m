function thetaDot = eulerRatesFromOmegaXYZ(theta, omegaS)
%EULERRATESFROMOMEGAXYZ X-Y-Z Euler angle rates from space-fixed angular velocity.
%
%   thetaDot = eulerRatesFromOmegaXYZ(theta, omegaS)
%
%   Inverts the lecture relation omegaS = E(theta) * thetaDot (L7-9/s7, s14),
%   i.e. the "coupled non-linear differential equations" of L7-9/s14 that are
%   integrated in the numerical example of L7-9/s16.
%
%   Inputs
%     theta  : 3x1 X-Y-Z Euler angles, rad.
%     omegaS : 3x1 space-fixed angular velocity in {A}, rad/s.
%   Output
%     thetaDot : 3x1, rad/s.
%
%   Singular when cos(t2) = 0 (t2 = +-pi/2), the Euler-angle singularity of
%   L2-4/s27; an error is raised there. The closed-form inverse is an
%   [EXTENSION] derived from the lecture matrix E:
%     t3dot = (-s1*wy + c1*wz)/c2
%     t2dot =   c1*wy + s1*wz
%     t1dot =   wx - s2*t3dot
arguments
    theta (3,1) double {mustBeFinite}
    omegaS (3,1) double {mustBeFinite}
end
s1 = sin(theta(1)); c1 = cos(theta(1));
s2 = sin(theta(2)); c2 = cos(theta(2));
if abs(c2) < 1e-9
    error('eulerRatesFromOmegaXYZ:singular', ...
        'X-Y-Z Euler angle singularity: cos(theta2) = 0.');
end
t3dot = (-s1 * omegaS(2) + c1 * omegaS(3)) / c2;
t2dot = c1 * omegaS(2) + s1 * omegaS(3);
t1dot = omegaS(1) - s2 * t3dot;
thetaDot = [t1dot; t2dot; t3dot];
end
