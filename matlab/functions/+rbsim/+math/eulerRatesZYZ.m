function [omegaS, omegaB] = eulerRatesZYZ(theta, thetaDot)
%EULERRATESZYZ Angular velocity from body-fixed Z-Y-Z Euler angles and their rates.
%
%   [omegaS, omegaB] = eulerRatesZYZ(theta, thetaDot)
%
%   Inputs
%     theta    : 3x1 Z-Y-Z Euler angles, rad.
%     thetaDot : 3x1 rates, rad/s.
%   Outputs
%     omegaS : 3x1 space-fixed angular velocity, components in {A}, rad/s.
%     omegaB : 3x1 body-fixed angular velocity, components in {B}, rad/s.
%
%   Source: [LECTURE] L7-9/s8 and s10 (both component sets written explicitly).
arguments
    theta (3,1) double {mustBeFinite}
    thetaDot (3,1) double {mustBeFinite}
end
s1 = sin(theta(1)); c1 = cos(theta(1));
s2 = sin(theta(2)); c2 = cos(theta(2));
s3 = sin(theta(3)); c3 = cos(theta(3));
d1 = thetaDot(1); d2 = thetaDot(2); d3 = thetaDot(3);
omegaS = [d3*c1*s2 - d2*s1;
          d3*s1*s2 + d2*c1;
          d3*c2 + d1];
omegaB = [-d1*s2*c3 + d2*s3;
           d1*s2*s3 + d2*c3;
           d1*c2 + d3];
end
