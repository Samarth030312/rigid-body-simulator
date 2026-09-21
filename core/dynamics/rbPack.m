function x = rbPack(r, v, R, omegaB)
%RBPACK Assemble the 18x1 rigid-body state vector.
%
%   x = rbPack(r, v, R, omegaB)
%
%   Layout (docs/architecture/CONVENTIONS.md section 6.2):
%     x(1:3)   ^A r_C   centre-of-mass position in {A}, m
%     x(4:6)   ^A V_C   centre-of-mass velocity in {A}, m/s
%     x(7:15)  ^A_B[R]  rotation matrix, column-major (R(:))
%     x(16:18) ^A w_B^b body-fixed angular velocity, components in {B}, rad/s
%
%   See also RBUNPACK.
arguments
    r (3,1) double
    v (3,1) double
    R (3,3) double
    omegaB (3,1) double
end
x = [r; v; R(:); omegaB];
end
