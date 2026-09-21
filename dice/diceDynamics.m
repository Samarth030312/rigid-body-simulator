function xdot = diceDynamics(~, x, p, G, I_B)
%DICEDYNAMICS State derivative of the dice: Newton-Euler with gravity (+ contact).
%
%   xdot = diceDynamics(t, x, p, G, I_B)
%
%   I_B = inertiaCube(p.mass, p.edge), precomputed by the caller (inner loop).
%
%   External loads: weight m g at C (L10-13/s8; no moment about C) and, if enabled,
%   floor contact (DICECONTACTLOADS). Moments are converted to {B} for Euler's
%   equation: ^B M = R' ^A M (CONVENTIONS 6.2).
[Fc, Mc] = diceContactLoads(x, p, G);
R = reshape(x(7:15), 3, 3);
xdot = rigidBodyDerivative(x, p.mass, I_B, p.mass * p.g + Fc, R.' * Mc);
end
