function Rdot = rotationRate(R, omegaB)
%ROTATIONRATE  Time derivative of the rotation matrix from body-fixed angular velocity.
%
%   Rdot = rotationRate(R, omegaB)
%
%   Inputs
%     R: 3x3 ^A_B[R].
%     omegaB: 3x1 body-fixed angular velocity ^A w_B^b, components in {B}, rad/s.
%   Output
%     Rdot: 3x3, Rdot = R * [omegaB x]  (= [omegaS x] * R).
%
%   From the definition [Omega]_L = R' Rdot, rearranged. The equivalent
%   space-fixed form is Rdot = [Omega]_R R, with [Omega]_R = Rdot R'.
    arguments
        R (3,3) double {mustBeFinite}
        omegaB (3,1) double {mustBeFinite}
    end
    Rdot = R * rbsim.math.skew(omegaB);
end
