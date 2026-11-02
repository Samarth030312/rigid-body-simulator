function thetaDot = eulerRatesFromOmegaXYZ(theta, omegaS)
%EULERRATESFROMOMEGAXYZ  X-Y-Z Euler angle rates from space-fixed angular velocity.
%
%   thetaDot = eulerRatesFromOmegaXYZ(theta, omegaS)
%
%   Inverts the relation omegaS = E(theta) * thetaDot,
%   a coupled non-linear system in the three angles, which is what gets
%   integrated when a body is propagated in Euler-angle coordinates.
%
%   Inputs
%     theta: 3x1 X-Y-Z Euler angles, rad.
%     omegaS: 3x1 space-fixed angular velocity in {A}, rad/s.
%   Output
%     thetaDot: 3x1, rad/s.
%
%   Singular when cos(t2) = 0 (t2 = +-pi/2), the gimbal-lock configuration of
%   the X-Y-Z set; an error is raised there. The closed-form inverse is an
%   engineering choice derived from the matrix E:
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
        error('rbsim:math:singular', ...
            'X-Y-Z Euler angle singularity: cos(theta2) = 0.');
    end
    t3dot = (-s1 * omegaS(2) + c1 * omegaS(3)) / c2;
    t2dot = c1 * omegaS(2) + s1 * omegaS(3);
    t1dot = omegaS(1) - s2 * t3dot;
    thetaDot = [t1dot; t2dot; t3dot];
end
