function [T, V, Meff, dMeff, dV] = energy(theta1, thetaDot1, p)
%ENERGY Energy of the four-bar and its one-DOF effective inertia.
%
%   [T, V, Meff, dMeff, dV] = rbsim.fourbar.energy(theta1, thetaDot1, p)
%
%   T    = sum 1/2 m_i |v_Ci|^2 + 1/2 I_i w_i^2 = 1/2 Meff(theta1) theta1dot^2
%   V    = sum m_i g y_Ci   (datum y = 0, gravity -Y_0)
%   Meff = sum m_i |dC_i/dtheta1|^2 + I_i k_i^2   (kinematic coefficients)
%   dMeff, dV: d/dtheta1 of Meff and V by central differences (h = 1e-6 rad).
%   These give the independent one-DOF energy equation used for validation:
%     Meff th1dd + 1/2 dMeff th1dot^2 + dV = tau_net        (engineering choice)
%   (Lagrange/energy methods are listed for L23-26, which were not supplied.)
kin = rbsim.fourbar.kinematics(theta1, 1, 0, p);          % unit crank rate -> coefficients
Meff = sum(p.m .* sum(kin.vC.^2, 1)) + sum(p.Izz .* kin.k.^2);
T = 0.5 * Meff * thetaDot1^2;
V = p.g * sum(p.m .* kin.C(2, :));
if nargout > 3
    h = 1e-6;
    [~, Vp, Mp] = rbsim.fourbar.energy(theta1 + h, 0, p);
    [~, Vm, Mm] = rbsim.fourbar.energy(theta1 - h, 0, p);
    dMeff = (Mp - Mm) / (2*h); dV = (Vp - Vm) / (2*h);
end
end
