function [omegaS, omegaB] = eulerRatesXYZ(theta, thetaDot)
%EULERRATESXYZ Angular velocity from body-fixed X-Y-Z Euler angles and their rates.
%
%   [omegaS, omegaB] = eulerRatesXYZ(theta, thetaDot)
%
%   Inputs
%     theta: 3x1 [t1;t2;t3] X-Y-Z Euler angles, rad.
%     thetaDot: 3x1 Euler angle rates, rad/s.
%   Outputs
%     omegaS: 3x1 space-fixed angular velocity ^A w_B^s, components in {A}, rad/s.
%     omegaB: 3x1 body-fixed angular velocity ^A w_B^b = R' * omegaS, in {B}.
%
%   Source:  (omegaS formulas), s10 (omegaS = R omegaB).
%   See also EULERRATESFROMOMEGAXYZ.
arguments
    theta (3,1) double {mustBeFinite}
    thetaDot (3,1) double {mustBeFinite}
end
[E, R] = eulerXYZMaps(theta);
omegaS = E * thetaDot;
omegaB = R.' * omegaS;
end

function [E, R] = eulerXYZMaps(theta)
s1 = sin(theta(1)); c1 = cos(theta(1));
s2 = sin(theta(2)); c2 = cos(theta(2));
E = [1   0    s2;
     0   c1  -s1*c2;
     0   s1   c1*c2];
R = rbsim.math.rotBodyFixed('XYZ', theta.');
end
