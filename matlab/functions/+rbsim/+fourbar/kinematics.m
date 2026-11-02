function kin = kinematics(theta1, thetaDot1, thetaDDot1, p)
%KINEMATICS Position, velocity and acceleration analysis of the four-bar.
%
%   kin = rbsim.fourbar.kinematics(theta1, thetaDot1, thetaDDot1, p)
%
%   Vector loop (the loop-closure constraint):
%     l1 e(theta1) + l2 e(theta2) - l3 e(phi1) - l0 X = 0,  e(a) = [cos a; sin a]
%   Differentiating the loop-closure constraint, with
%   n(a) = [-sin a; cos a] and J = [l2 n(theta2), -l3 n(phi1)]:
%     J [theta2dot; phi1dot]   = -l1 n1 theta1dot
%     J [theta2ddot; phi1ddot] = -l1 n1 theta1ddot + l1 e1 theta1dot^2
%                                + l2 e2 theta2dot^2 - l3 e3 phi1dot^2
%   det J = l2 l3 sin(phi1 - theta2): singular when coupler and rocker are
%   collinear (toggle position).
%
%   Output struct kin: all fields of FOURBARPOSITION plus
%.ok (false if the loop cannot close OR at a toggle),.toggle,.omega (1x3: theta1dot, theta2dot, phi1dot),.alpha (1x3),
%.k (1x3 kinematic coefficients d(angle)/d theta1),.kdot (1x3: d/dt of k,
%     per unit theta1dot; alpha = k theta1ddot + kdot theta1dot... see below),
%.C (2x3 COM positions),.vC (2x3),.aC (2x3),.detJ
%   COM positions: C1 = r1 e1, C2 = A + r2 e2, C3 = O_R + r3 e3.
[kin, ok] = rbsim.fourbar.position(theta1, p);
kin.ok = ok;
kin.toggle = false;
if ~ok
    kin.omega = nan(1,3); kin.alpha = nan(1,3); kin.k = nan(1,3); kin.detJ = NaN;
    kin.C = nan(2,3); kin.vC = nan(2,3); kin.aC = nan(2,3); kin.bias = nan(1,3);
    return
end
l0 = p.l(1); l1 = p.l(2); l2 = p.l(3); l3 = p.l(4); %#ok<NASGU>
t1 = theta1; t2 = kin.theta2; f1 = kin.phi1;
e = @(a) [cos(a); sin(a)]; n = @(a) [-sin(a); cos(a)];
J = [l2 * n(t2), -l3 * n(f1)];
kin.detJ = det(J);
if abs(kin.detJ) < 1e-9 * l2 * l3          % toggle: velocities undefined
    kin.ok = false;
    kin.omega = nan(1,3); kin.alpha = nan(1,3); kin.k = nan(1,3);
    kin.C = nan(2,3); kin.vC = nan(2,3); kin.aC = nan(2,3); kin.bias = nan(1,3);
    kin.toggle = true;
    return
end
kin.toggle = false;
% kinematic coefficients (velocity per unit crank rate)
k23 = J \ (-l1 * n(t1));
kin.k = [1, k23.'];
w = kin.k * thetaDot1;
% acceleration = k * theta1ddot + bias (bias collects the velocity-squared terms)
b23 = J \ (l1 * e(t1) * w(1)^2 + l2 * e(t2) * w(2)^2 - l3 * e(f1) * w(3)^2);
kin.bias = [0, b23.'];
kin.omega = w;
kin.alpha = kin.k * thetaDDot1 + kin.bias;
r1 = p.r(1); r2 = p.r(2); r3 = p.r(3);
a = kin.alpha;
kin.C  = [r1 * e(t1), kin.A + r2 * e(t2), kin.OR + r3 * e(f1)];
kin.vC = [r1 * n(t1) * w(1), l1 * n(t1) * w(1) + r2 * n(t2) * w(2), r3 * n(f1) * w(3)];
kin.aC = [r1 * (n(t1) * a(1) - e(t1) * w(1)^2), ...
          l1 * (n(t1) * a(1) - e(t1) * w(1)^2) + r2 * (n(t2) * a(2) - e(t2) * w(2)^2), ...
          r3 * (n(f1) * a(3) - e(f1) * w(3)^2)];
end
