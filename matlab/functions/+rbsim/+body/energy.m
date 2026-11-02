function [T, V, H_A, P_A] = energy(x, m, I_B, g_A)
%ENERGY  Kinetic/potential energy and momenta of a free rigid body.
%
%   [T, V, H_A, P_A] = rbsim.body.energy(x, m, I_B, g_A)
%
%   Inputs
%     x: 18x1 rigid-body state (see rbsim.body.pack).
%     m: mass, kg.  I_B: 3x3 ^B[I]_C, kg*m^2.
%     g_A: 3x1 gravitational acceleration in {A}, m/s^2 (e.g. [0;0;-9.81]).
%   Outputs
%     T: kinetic energy (1/2) m V_C.V_C + (1/2) w_b.(^B[I]_C w_b), J
%     V: gravitational potential energy -m g.r_C, J (datum at r = 0)
%     H_A: 3x1 angular momentum about C in {A}: ^A_B[R] ^B[I]_C w_b, kg*m^2/s
%           (= ^A[I]_C ^A w^s,,)
%     P_A: 3x1 linear momentum m V_C in {A}, kg*m/s
%
%   Note: w.[I]w is frame-independent, so the body-frame form is used.
    arguments
        x (18,1) double
        m (1,1) double {mustBePositive}
        I_B (3,3) double
        g_A (3,1) double
    end
    [r, v, R, wb] = rbsim.body.unpack(x);
    T = 0.5 * m * (v.' * v) + 0.5 * wb.' * (I_B * wb);
    V = -m * (g_A.' * r);
    H_A = R * (I_B * wb);
    P_A = m * v;
end
