function M = massMatrix(q, p)
%MASSMATRIX  Joint-space mass matrix M(q) of the planar 3R, kg m^2.
%
%   M = rbsim.robot.massMatrix(q, p)
%
%   Column j is the RNE torque for qdd = e_j with qd = 0 and g = 0, because
%   tau = M(q) qdd + C(q,qd) + G(q), in which C and G vanish for that input.
%   The standard way to read M(q) out of an inverse-dynamics routine.
    M = zeros(3);
    E = eye(3);
    for j = 1:3
        M(:, j) = rbsim.robot.newtonEuler(q, zeros(3, 1), E(:, j), p, 0);
    end
    M = 0.5 * (M + M.');
end
