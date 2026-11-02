function [tau, out] = newtonEuler(q, qd, qdd, p, g, fTool)
%NEWTONEULER Recursive Newton-Euler inverse dynamics of the planar 3R.
%
%   tau = rbsim.robot.newtonEuler(q, qd, qdd, p)            % gravity p.g, no tool load
%   [tau, out] = rbsim.robot.newtonEuler(q, qd, qdd, p, g, fTool)
%
%   Inputs
%     q, qd, qdd: 3x1 joint angles (rad), rates (rad/s), accelerations (rad/s^2)
%     p: parameters (l, m, r, Izz)
%     g: gravity magnitude along -Y_0, m/s^2 (default p.g)
%     fTool: 3x1 force applied BY the tool ON the environment, in {Tool}, N
%                  (enters as ^{N+1}f_{N+1}; default 0)
%   Outputs
%     tau: 3x1 joint torques, N m (positive CCW about Z_i)
%     out: per-link kinematics/forces, all in link frame {i}:
%.w,.wd (3x3 angular velocity/acceleration),.Vd (origin accel.),
%.VdC (COM accel.),.F,.N (inertial force/moment),.f,.n (joint
%           force/moment exerted on link i by link i-1)
%
%   Algorithm exactly as  (R joints, joint axis Z_i = (0 0 1)^T):
%   outward i = 0..N-1
%     ^{i+1}w     = R' ^i w + qd_{i+1} Z
%     ^{i+1}wd    = R' ^i wd + (R' ^i w) x qd_{i+1} Z + qdd_{i+1} Z
%     ^{i+1}Vd    = R' (^i Vd + ^i wd x ^i p_{i+1} + ^i w x (^i w x ^i p_{i+1}))
%     ^{i+1}Vd_C  = ^{i+1}Vd + wd x p_C + w x (w x p_C)
%     F = m Vd_C,  N = ^C[I] wd + w x ^C[I] w
%   inward i = N..1
%     ^i f = R ^{i+1}f + ^i F
%     ^i n = R ^{i+1}n + ^i p_{i+1} x R ^{i+1}f + ^i p_C x ^i F + ^i N
%     tau_i = ^i n. Z
%   with R = ^i_{i+1}[R] = R(Z, q_{i+1}) and gravity via ^0 Vd_0 = (0, g, 0)^T.
%   Link inertia ^C[I]_i = diag(0, Izz, Izz)-type terms do not affect planar
%   motion; only Izz enters, so ^C[I] = diag(0, 0, Izz) is used.
if nargin < 5 || isempty(g), g = p.g; end
if nargin < 6 || isempty(fTool), fTool = zeros(3, 1); end
Z = [0; 0; 1];
w = zeros(3, 1); wd = zeros(3, 1);
Vd = [0; g; 0];                          % base "accelerating upward with 1 g"
pPrev = zeros(3, 1);                     % ^0 p_1 = 0 (O_1 at the origin of {0})
Rl = cell(1, 4);                         % Rl{i} = ^{i-1}_i[R]
out.w = zeros(3); out.wd = zeros(3); out.Vd = zeros(3); out.VdC = zeros(3);
out.F = zeros(3); out.N = zeros(3); out.f = zeros(3); out.n = zeros(3);
for i = 1:3
    Rl{i} = rbsim.math.rotZ(q(i));
    Rt = Rl{i}.';
    wPrev = w;
    w = Rt * wPrev + qd(i) * Z;
    wdNew = Rt * wd + cross(Rt * wPrev, qd(i) * Z) + qdd(i) * Z;
    Vd = Rt * (Vd + cross(wd, pPrev) + cross(wPrev, cross(wPrev, pPrev)));
    wd = wdNew;
    pC = [p.r(i); 0; 0];
    VdC = Vd + cross(wd, pC) + cross(w, cross(w, pC));
    Ic = diag([0, 0, p.Izz(i)]);
    out.w(:, i) = w; out.wd(:, i) = wd; out.Vd(:, i) = Vd; out.VdC(:, i) = VdC;
    out.F(:, i) = p.m(i) * VdC;
    out.N(:, i) = Ic * wd + cross(w, Ic * w);
    pPrev = [p.l(i); 0; 0];              % ^i p_{i+1}
end
Rl{4} = eye(3);                          % {Tool} parallel to {3}
f = fTool; n = zeros(3, 1);
tau = zeros(3, 1);
for i = 3:-1:1
    R = Rl{i + 1};                        % ^i_{i+1}[R]
    pNext = [p.l(i); 0; 0];
    pC = [p.r(i); 0; 0];
    fNew = R * f + out.F(:, i);
    n = R * n + cross(pNext, R * f) + cross(pC, out.F(:, i)) + out.N(:, i);
    f = fNew;
    out.f(:, i) = f; out.n(:, i) = n;
    tau(i) = n.' * Z;
end
end
