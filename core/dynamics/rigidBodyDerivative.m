function xdot = rigidBodyDerivative(x, m, I_B, F_A, M_B)
%RIGIDBODYDERIVATIVE Newton-Euler state derivative of a single free rigid body.
%
%   xdot = rigidBodyDerivative(x, m, I_B, F_A, M_B)
%
%   Inputs
%     x   : 18x1 state [r_C; V_C; R(:); w_b] (see RBPACK).
%     m   : mass, kg.
%     I_B : 3x3 ^B[I]_C, inertia about the COM in body frame {B}, kg*m^2.
%     F_A : 3x1 resultant external force, components in {A}, N.
%     M_B : 3x1 resultant external moment about the COM C, components in {B}, N*m.
%   Output
%     xdot: 18x1 time derivative.
%
%   Equations
%     d/dt r_C = V_C                                   (definition, L7-9/s4)
%     m dV_C/dt = F_A                                  (Newton, L17-19/s6)
%     dR/dt = R [w_b x]                                (L7-9/s9)
%     ^B[I]_C dw_b/dt = M_B - w_b x (^B[I]_C w_b)      (Euler, L16/s6)
arguments
    x (18,1) double
    m (1,1) double {mustBePositive}
    I_B (3,3) double
    F_A (3,1) double
    M_B (3,1) double
end
[~, v, R, wb] = rbUnpack(x);
vdot = F_A / m;
Rdot = R * skew(wb);
wbdot = I_B \ (M_B - cross(wb, I_B * wb));
xdot = [v; vdot; Rdot(:); wbdot];
end
