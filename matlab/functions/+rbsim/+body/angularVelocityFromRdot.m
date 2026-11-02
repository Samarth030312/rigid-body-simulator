function [omegaS, omegaB] = angularVelocityFromRdot(R, Rdot)
%ANGULARVELOCITYFROMRDOT Space- and body-fixed angular velocity from R and dR/dt.
%
%   [omegaS, omegaB] = angularVelocityFromRdot(R, Rdot)
%
%   Inputs
%     R: 3x3 ^0_i[R].
%     Rdot: 3x3 time derivative of R, 1/s.
%   Outputs
%     omegaS: 3x1 from [Omega]_R = Rdot*R'  (space-fixed, components in {0}), rad/s.
%     omegaB: 3x1 from [Omega]_L = R'*Rdot  (body-fixed, components in {i}), rad/s.
%
%   Source:  ([Omega]_R), s9-10 ([Omega]_L).
arguments
    R (3,3) double {mustBeFinite}
    Rdot (3,3) double {mustBeFinite}
end
omegaS = rbsim.math.unskew(Rdot * R.');
omegaB = rbsim.math.unskew(R.' * Rdot);
end
