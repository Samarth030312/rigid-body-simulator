function [T, V] = energy(x, p)
%ENERGY  Kinetic and potential energy of the planar 3R, J.
%
%   [T, V] = rbsim.robot.energy(x, p)
%
%   Computed link-by-link from COM velocities and angular velocities, NOT from
%   M(q), so it independently checks the dynamics:
%     T = sum 1/2 m_i V_Ci.V_Ci + 1/2 Izz_i w_i^2
%     V = sum m_i g y_Ci   (gravity along -Y_0, datum y = 0)
%   COM velocities by frame propagation: V_Ci = w_i x (C_i - O_1) summed along
%   the chain, evaluated here with the planar closed form of.
    q = x(1:3); qd = x(4:6);
    fk = rbsim.robot.forwardKinematics(q, p);
    T = 0; V = 0;
    w = 0; vO = zeros(3, 1);
    for i = 1:3
        w = w + qd(i);                               % ^0 w_i (z-component)
        rC = fk.C(:, i) - fk.O(:, i);
        vC = vO + cross([0; 0; w], rC);              %  with ^B p fixed
        T = T + 0.5 * p.m(i) * (vC.' * vC) + 0.5 * p.Izz(i) * w^2;
        V = V + p.m(i) * p.g * fk.C(2, i);
        vO = vO + cross([0; 0; w], fk.O(:, i+1) - fk.O(:, i));
    end
end
