function [r, v, R, omegaB] = unpack(x)
%UNPACK Split the 18x1 rigid-body state vector (see RBPACK for the layout).
%
%   [r, v, R, omegaB] = rbsim.body.unpack(x)
arguments
    x (18,1) double
end
r = x(1:3);
v = x(4:6);
R = reshape(x(7:15), 3, 3);
omegaB = x(16:18);
end
