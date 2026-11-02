function [x, corr] = projectState(x)
%PROJECTSTATE Re-orthonormalise the rotation block of an 18x1 rigid-body state.
%
%   [x, corr] = rbsim.body.projectState(x)
%
%   Intended as the INTEGRATERK4 PostStep for rigid bodies. corr is the Frobenius
%   norm of the change in R (a direct measure of integration drift off SO(3)).
%   Source: [EXTENSION] (ADR-0008).
arguments
    x (18,1) double
end
R = reshape(x(7:15), 3, 3);
Rn = rbsim.math.orthonormalize(R);
corr = norm(Rn - R, 'fro');
x(7:15) = Rn(:);
end
