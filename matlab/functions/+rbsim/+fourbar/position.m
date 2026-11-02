function [pos, ok] = position(theta1, p)
%POSITION  Position analysis of the four-bar for crank angle theta1.
%
%   [pos, ok] = rbsim.fourbar.position(theta1, p)
%
%   Solves the loop-closure constraint in Freudenstein form
%     eta1(theta1, phi1) = (l1 c1 - l0 - l3 cos phi1)^2 + (l1 s1 - l3 sin phi1)^2 - l2^2 = 0
%   in closed form by intersecting the circle of radius l2 about the crank pin A
%   with the circle of radius l3 about O_R (engineering choice of solution method):
%     e = A - O_R, psi = atan2(e_y, e_x), gamma = acos((l3^2 + |e|^2 - l2^2)/(2 l3 |e|))
%     phi1 = psi - branch * gamma
%   branch = +1 is the "open" assembly, -1 the "crossed" one.
%
%   Outputs
%     ok: false if the loop cannot close for this theta1 (|e| outside
%           [|l2-l3|, l2+l3]); pos fields are then NaN.
%     pos: struct with points O_L, A, B, O_R (2x1, m), angles theta1, theta2
%           (absolute coupler angle), phi1 (rad), relative angles phi2,
%           phi3 (in [0, 2pi)), residuals:
%.eta1 (m^2),.closure (2x1, m: the two components of the vector loop),
%.angleSum (rad: theta1 + phi2 + phi3 + (pi - phi1) - 4pi, wrapped),
%.transmission (rad: angle between coupler and rocker at B).
    l0 = p.l(1); l1 = p.l(2); l2 = p.l(3); l3 = p.l(4);
    OL = [0; 0]; OR = [l0; 0];
    A = l1 * [cos(theta1); sin(theta1)];
    e = A - OR;
    D = norm(e);
    c = (l3^2 + D^2 - l2^2) / (2 * l3 * D);
    ok = D > 0 && abs(c) <= 1 + 1e-12;
    if ~ok
        nanv = [NaN; NaN];
        pos = struct('OL', OL, 'A', A, 'B', nanv, 'OR', OR, 'theta1', theta1, ...
            'theta2', NaN, 'phi1', NaN, 'phi2', NaN, 'phi3', NaN, 'eta1', NaN, ...
            'closure', nanv, 'angleSum', NaN, 'transmission', NaN);
        return
    end
    gamma = acos(min(1, max(-1, c)));
    psi = atan2(e(2), e(1));
    phi1 = psi - p.branch * gamma;
    B = OR + l3 * [cos(phi1); sin(phi1)];
    theta2 = atan2(B(2) - A(2), B(1) - A(1));
    pos.OL = OL; pos.A = A; pos.B = B; pos.OR = OR;
    pos.theta1 = theta1; pos.theta2 = theta2; pos.phi1 = phi1;
    % Relative joint angles: phi2 from the extension of link 1 to link 2,
    % phi3 from the extension of link 2 to the direction B -> O_R.
    pos.phi2 = mod(theta2 - theta1, 2*pi);
    pos.phi3 = mod(phi1 + pi - theta2, 2*pi);
    pos.eta1 = (l1*cos(theta1) - l0 - l3*cos(phi1))^2 + (l1*sin(theta1) - l3*sin(phi1))^2 - l2^2;
    t12 = theta1 + pos.phi2; t123 = t12 + pos.phi3;
    pos.closure = [l1*cos(theta1) + l2*cos(t12) + l3*cos(t123) - l0;
                   l1*sin(theta1) + l2*sin(t12) + l3*sin(t123)];
    pos.angleSum = angdiff0(theta1 + pos.phi2 + pos.phi3 + (pi - phi1) - 4*pi);
    u2 = (B - A) / l2; u3 = (B - OR) / l3;
    mu = acos(max(-1, min(1, u2.' * u3)));      % angle between coupler and rocker
    pos.transmission = min(mu, pi - mu);         % folded into [0, pi/2]
end

function d = angdiff0(a)
    d = mod(a + pi, 2*pi) - pi;
end
