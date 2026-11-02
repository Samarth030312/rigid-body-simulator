function [sol, kin] = newtonEuler(theta1, thetaDot1, p, known)
%NEWTONEULER  Four-bar dynamics by per-link Newton-Euler with FBDs.
%
%   [sol, kin] = rbsim.fourbar.newtonEuler(theta1, thetaDot1, p, known)
%
%   known.type = 'torque': known.value = crank drive torque tau -> solve thetaDDot1
%   known.type = 'accel': known.value = thetaDDot1 -> solve required tau
%   Load torque p.drive.loadTorque acts on link 3; crank bearing damping
%   -b thetaDot1 acts on link 1 (both optional).
%
%   Free-body diagrams, one per link (EXTENSION, ADR-0011):
%     link 1:  F_OL - F_A + m1 g = m1 a_C1
%              tau - b th1dot + (O_L - C1) x F_OL + (A - C1) x (-F_A) = I1 alpha1
%     link 2:  F_A - F_B + m2 g = m2 a_C2
%              (A - C2) x F_A + (B - C2) x (-F_B) = I2 alpha2
%     link 3:  F_B + F_OR + m3 g = m3 a_C3
%              tauLoad + (B - C3) x F_B + (O_R - C3) x F_OR = I3 alpha3
%   F_OL: ground on link 1; F_A: link 1 on link 2; F_B: link 2 on link 3;
%   F_OR: ground on link 3 (all in {0}, N). g = [0; -g].
%   COM and angular accelerations are affine in thetaDDot1:
%     a_Ci = aK_i thetaDDot1 + aB_i,  alpha_i = k_i thetaDDot1 + bias_i
%   giving 9 linear equations in 9 unknowns [F_OL; F_A; F_B; F_OR; X], X = the
%   unknown thetaDDot1 or tau.
%
%   Output sol:.thetaDDot1,.tau,.F_OL,.F_A,.F_B,.F_OR (2x1, N)
    kin = rbsim.fourbar.kinematics(theta1, thetaDot1, 0, p);
    if kin.toggle
        error('rbsim:fourbar:singular', ['Toggle (singular) configuration at theta1 = ' ...
            '%.4f rad: coupler and rocker are collinear.'], theta1);
    end
    if ~kin.ok
        error('rbsim:fourbar:assembly', 'Loop cannot close at theta1 = %.4f rad.', theta1);
    end
    m = p.m; I = p.Izz; gv = [0; -p.g];
    dr = p.drive;
    % acceleration parts: aC(theta1ddot) = aK * th1dd + aB
    kin1 = rbsim.fourbar.kinematics(theta1, thetaDot1, 1, p);
    aB = kin.aC; aK = kin1.aC - kin.aC;
    alB = kin.alpha; alK = kin1.alpha - kin.alpha;
    cr = @(r) [-r(2), r(1)];            % r x F (z) = cr(r) * F
    C = kin.C;
    Az = zeros(9); bz = zeros(9, 1);
    % unknown order: F_OL(1:2) F_A(3:4) F_B(5:6) F_OR(7:8) X(9)
    iOL = 1:2; iA = 3:4; iB = 5:6; iOR = 7:8;
    % link 1 force
    Az(1:2, iOL) = eye(2); Az(1:2, iA) = -eye(2);
    bz(1:2) = -m(1) * gv;               % + m a moved below
    % link 1 moment
    Az(3, iOL) = cr(kin.OL - C(:,1)); Az(3, iA) = -cr(kin.A - C(:,1));
    bz(3) = dr.damping * thetaDot1;
    % link 2 force
    Az(4:5, iA) = eye(2); Az(4:5, iB) = -eye(2);
    bz(4:5) = -m(2) * gv;
    % link 2 moment
    Az(6, iA) = cr(kin.A - C(:,2)); Az(6, iB) = -cr(kin.B - C(:,2));
    % link 3 force
    Az(7:8, iB) = eye(2); Az(7:8, iOR) = eye(2);
    bz(7:8) = -m(3) * gv;
    % link 3 moment
    Az(9, iB) = cr(kin.B - C(:,3)); Az(9, iOR) = cr(kin.OR - C(:,3));
    bz(9) = -dr.loadTorque;
    % inertia terms  m a = m (aK X + aB),  I alpha = I (alK X + alB)
    rowsF = {1:2, 4:5, 7:8}; rowsM = [3, 6, 9];
    for i = 1:3
        bz(rowsF{i}) = bz(rowsF{i}) + m(i) * aB(:, i);
        bz(rowsM(i)) = bz(rowsM(i)) + I(i) * alB(i);
    end
    switch known.type
        case 'torque'
            tau = known.value;
            bz(3) = bz(3) - tau;
            for i = 1:3                      % move X-dependent inertia terms left
                Az(rowsF{i}, 9) = -m(i) * aK(:, i);
                Az(rowsM(i), 9) = -I(i) * alK(i);
            end
            u = Az \ bz;
            th1dd = u(9);
        case 'accel'
            th1dd = known.value;
            for i = 1:3
                bz(rowsF{i}) = bz(rowsF{i}) + m(i) * aK(:, i) * th1dd;
                bz(rowsM(i)) = bz(rowsM(i)) + I(i) * alK(i) * th1dd;
            end
            Az(3, 9) = 1;                    % tau enters link-1 moment equation
            u = Az \ bz;
            tau = u(9);
        otherwise
            error('rbsim:fourbar:param', 'known.type must be ''torque'' or ''accel''.');
    end
    sol.thetaDDot1 = th1dd; sol.tau = tau;
    sol.F_OL = u(iOL); sol.F_A = u(iA); sol.F_B = u(iB); sol.F_OR = u(iOR);
end
