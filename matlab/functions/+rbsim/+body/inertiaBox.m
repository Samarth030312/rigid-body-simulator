function I = inertiaBox(m, a, b, c)
%INERTIABOX  Inertia matrix of a uniform rectangular box about its centre of mass.
%
%   I = inertiaBox(m, a, b, c)
%
%   Inputs
%     m: mass, kg (> 0).
%     a, b, c: edge lengths along body X, Y, Z, m (> 0).
%   Output
%     I: 3x3 ^B[I]_C, kg*m^2, body frame {B} at C with axes along the edges:
%               diag(m(b^2+c^2), m(a^2+c^2), m(a^2+b^2)) / 12
%
    arguments
        m (1,1) double {mustBePositive, mustBeFinite}
        a (1,1) double {mustBePositive, mustBeFinite}
        b (1,1) double {mustBePositive, mustBeFinite}
        c (1,1) double {mustBePositive, mustBeFinite}
    end
    I = (m / 12) * diag([b^2 + c^2, a^2 + c^2, a^2 + b^2]);
end
