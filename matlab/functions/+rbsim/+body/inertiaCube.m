function I = inertiaCube(m, a)
%INERTIACUBE Inertia matrix of a uniform cube about its centre of mass.
%
%   I = inertiaCube(m, a)
%
%   Inputs
%     m: mass, kg.   a: edge length, m.
%   Output
%     I: 3x3 ^B[I]_C = (m a^2 / 6) [U], kg*m^2 (any orthonormal body axes
%         through C, because the cube's inertia is isotropic).
%
%   Source: box formula  with a = b = c.
arguments
    m (1,1) double {mustBePositive, mustBeFinite}
    a (1,1) double {mustBePositive, mustBeFinite}
end
I = rbsim.body.inertiaBox(m, a, a, a);
end
