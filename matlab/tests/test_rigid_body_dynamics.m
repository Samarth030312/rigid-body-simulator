function tests = test_rigid_body_dynamics
%TEST_RIGID_BODY_DYNAMICS Level 3-4 validation of Newton-Euler + RK4 integration.
tests = functiontests(localfunctions);
end

function setupOnce(tc)
tc.TestData.g = [0; 0; -9.81];
end

function sol = simulate(x0, m, I, loads, T, dt)
% loads(t, x) -> [F_A, M_B]
f = @(t, x) rbf(t, x, m, I, loads);
sol = rbsim.solver.rk4(f, [0 T], x0, dt, PostStep=@rbsim.body.projectState);
end

function xd = rbf(t, x, m, I, loads)
[F, M] = loads(t, x);
xd = rbsim.body.derivative(x, m, I, F, M);
end

function [F, M] = noLoads(~, ~)
F = zeros(3,1); M = zeros(3,1);
end

% ---------- Level 3: derivative checks ----------
function testStationaryBodyStaysAtRest(tc)
x0 = rbsim.body.pack([1;2;3], zeros(3,1), rbsim.math.rotBodyFixed('XYZ', [0.1 0.2 0.3]), zeros(3,1));
sol = simulate(x0, 1, rbsim.body.inertiaBox(1, .1, .2, .3), @noLoads, 2, 0.01);
verifyEqual(tc, sol.x(:, end), x0, 'AbsTol', 1e-14);
end

function testPureTranslation(tc)
v0 = [0.3; -1; 2];
x0 = rbsim.body.pack([0;0;1], v0, eye(3), zeros(3,1));
sol = simulate(x0, 2, rbsim.body.inertiaCube(2, 0.1), @noLoads, 3, 0.01);
[r, v, R] = rbsim.body.unpack(sol.x(:, end));
verifyEqual(tc, r, [0;0;1] + v0*3, 'AbsTol', 1e-12);
verifyEqual(tc, v, v0, 'AbsTol', 1e-14);
verifyEqual(tc, R, eye(3), 'AbsTol', 1e-15);
end

function testGravityProjectileAnalytic(tc)
% Newton  with mg: r = r0 + v0 t + g t^2/2 (RK4 exact).
g = tc.TestData.g; m = 0.006;
r0 = [0; 0; 1]; v0 = [1; 0.5; 3];
x0 = rbsim.body.pack(r0, v0, eye(3), [5; -3; 8]);
sol = simulate(x0, m, rbsim.body.inertiaCube(m, 0.016), @(t,x) deal(m*g, zeros(3,1)), 0.7, 1e-3);
t = sol.t;
rExact = r0 + v0 .* t + 0.5 * g .* t.^2;
verifyEqual(tc, sol.x(1:3, :), rExact, 'AbsTol', 1e-12);
end

function testCubeTorqueFreeRotatesAboutFixedAxis(tc)
% Isotropic inertia => w x Iw = 0 => w_b constant and R(t) = R0 R(k, |w| t).
m = 0.006; a = 0.016; I = rbsim.body.inertiaCube(m, a);
R0 = rbsim.math.rotBodyFixed('XYZ', [0.2 -0.4 0.7]); wb = [4; -7; 11];
x0 = rbsim.body.pack([0;0;0], [0;0;0], R0, wb);
T = 1.3;
sol = simulate(x0, m, I, @noLoads, T, 1e-3);
[~, ~, R, wEnd] = rbsim.body.unpack(sol.x(:, end));
verifyEqual(tc, wEnd, wb, 'AbsTol', 1e-12);
Rexact = R0 * rbsim.math.rotAxisAngle(wb, norm(wb) * T);
% RK4 phase error bound ~ N (|w| dt)^5 / 120 = 1300 * 0.0136^5 / 120 ~ 5e-9.
verifyEqual(tc, R, Rexact, 'AbsTol', 2e-8);
end

function testSymmetricTopTorqueFreeAnalytic(tc)
%: Ixx = Iyy, wz constant, Omega = (Izz-Ixx)/Ixx wz,
% wx = a0 sin(Omega t + phi), wy = -a0 cos(Omega t + phi).
I = diag([2, 2, 3.5]) * 1e-3;
w0 = [0.6; -0.8; 5];
x0 = rbsim.body.pack([0;0;0], [0;0;0], eye(3), w0);
sol = simulate(x0, 1, I, @noLoads, 4, 1e-3);
Om = (I(3,3) - I(1,1)) / I(1,1) * w0(3);
a0 = hypot(w0(1), w0(2));
phi = atan2(w0(1), -w0(2));          % from wx(0)=a0 sin phi, wy(0)=-a0 cos phi
t = sol.t;
wx = a0 * sin(Om*t + phi); wy = -a0 * cos(Om*t + phi);
verifyEqual(tc, sol.x(16, :), wx, 'AbsTol', 1e-9);
verifyEqual(tc, sol.x(17, :), wy, 'AbsTol', 1e-9);
verifyEqual(tc, sol.x(18, :), w0(3) * ones(size(t)), 'AbsTol', 1e-12);
end

function testAsymmetricTorqueFreeConservation(tc)
% Torque-free: kinetic energy and ^A H_C constant.
I = diag([1, 2, 3]) * 1e-3;
x0 = rbsim.body.pack([0;0;0], [0;0;0], rbsim.math.rotBodyFixed('ZYZ', [0.3 1 -0.5]), [1; 6; 0.5]);
sol = simulate(x0, 1, I, @noLoads, 10, 1e-3);
[T0, ~, H0] = rbsim.body.energy(sol.x(:,1), 1, I, zeros(3,1));
maxdT = 0; maxdH = 0;
for k = 1:size(sol.x, 2)
    [Tk, ~, Hk] = rbsim.body.energy(sol.x(:,k), 1, I, zeros(3,1));
    maxdT = max(maxdT, abs(Tk - T0)); maxdH = max(maxdH, norm(Hk - H0));
end
verifyLessThan(tc, maxdT / T0, 1e-8);
verifyLessThan(tc, maxdH / norm(H0), 1e-8);
verifyLessThan(tc, sol.solver.maxCorrection, 1e-10);
end

function testEulerEquationEqualsRateOfAngularMomentum(tc)
% Newton-Euler consistency: d/dt(^A H_C) = ^A M = R ^B M.
I = [3 0.2 -0.1; 0.2 2 0.3; -0.1 0.3 4] * 1e-3;   % non-diagonal, SPD
MB = @(t) [0.01*sin(t); -0.02; 0.005*t];
loads = @(t, x) deal(zeros(3,1), MB(t));
x0 = rbsim.body.pack([0;0;0], [0;0;0], rbsim.math.rotBodyFixed('XYZ', [0.5 0.1 -0.3]), [2; -1; 3]);
sol = simulate(x0, 1, I, loads, 1, 1e-4);
k = 5000; h = sol.t(k+1) - sol.t(k);
[~, ~, Hp] = rbsim.body.energy(sol.x(:,k+1), 1, I, zeros(3,1));
[~, ~, Hm] = rbsim.body.energy(sol.x(:,k-1), 1, I, zeros(3,1));
[~, ~, R] = rbsim.body.unpack(sol.x(:,k));
verifyEqual(tc, (Hp - Hm) / (2*h), R * MB(sol.t(k)), 'AbsTol', 1e-7);
end

function testConstantTorqueAboutPrincipalAxis(tc)
% From rest, M about a principal axis: w = (M/I) t about that axis.
I = diag([1 2 3]) * 1e-3; M = [0; 0.006; 0];
x0 = rbsim.body.pack([0;0;0], [0;0;0], eye(3), [0;0;0]);
sol = simulate(x0, 1, I, @(t,x) deal(zeros(3,1), M), 1, 1e-3);
verifyEqual(tc, sol.x(16:18, end), [0; 3; 0], 'AbsTol', 1e-12);
[~, ~, R] = rbsim.body.unpack(sol.x(:, end));
verifyEqual(tc, R, rbsim.math.rotY(1.5), 'AbsTol', 1e-10);     % angle = (M/I) t^2 / 2
end

function testGravityEnergyConservation(tc)
m = 0.2; I = rbsim.body.inertiaBox(m, 0.1, 0.05, 0.02); g = tc.TestData.g;
x0 = rbsim.body.pack([0;0;2], [1;2;4], eye(3), [3; 5; -2]);
sol = simulate(x0, m, I, @(t,x) deal(m*g, zeros(3,1)), 1, 1e-3);
E = zeros(1, size(sol.x, 2));
for k = 1:numel(E)
    [T, V] = rbsim.body.energy(sol.x(:,k), m, I, g); E(k) = T + V;
end
verifyLessThan(tc, max(abs(E - E(1))) / abs(E(1)), 1e-9);
end

% ---------- Level 4: integration ----------
function testEulerAngleIntegrationMatchesExactRotation(tc)
%: X-Y-Z Euler angles driven by constant w^s = [1 2 3] rad/s from 0.
% (a) Integrate the Euler-angle ODE; (b) the exact rotation for constant
% w^s is R(t) = R(w^s/|w^s|, |w^s| t). Both must describe the same orientation.
ws = [1; 2; 3];
sol = rbsim.solver.rk4(@(t, th) rbsim.math.eulerRatesFromOmegaXYZ(th, ws), [0 5], zeros(3,1), 1e-4, ...
    OutputEvery=100);
for k = 1:10:numel(sol.t)
    Rth = rbsim.math.rotBodyFixed('XYZ', sol.x(:, k).');
    Rex = rbsim.math.rotAxisAngle(ws, norm(ws) * sol.t(k));
    verifyEqual(tc, Rth, Rex, 'AbsTol', 1e-8);
end
% The comparison with the exact rotation above is the authoritative check.
% Regression values of this implementation at t = 5 s:
verifyEqual(tc, sol.x(:, end), [-0.0420; -0.0732; 18.7347], 'AbsTol', 1e-3);
end

function testRK4FourthOrderConvergence(tc)
I = diag([1, 2, 3]) * 1e-3;
x0 = rbsim.body.pack([0;0;0], [0;0;0], eye(3), [1; 6; 0.5]);
f = @(t, x) rbsim.body.derivative(x, 1, I, zeros(3,1), zeros(3,1));
ref = rbsim.solver.rk4(f, [0 2], x0, 1e-4);
dts = [0.04 0.02 0.01];
err = zeros(size(dts));
for i = 1:numel(dts)
    s = rbsim.solver.rk4(f, [0 2], x0, dts(i));
    err(i) = norm(s.x(:, end) - ref.x(:, end));
end
order = log2(err(1:end-1) ./ err(2:end));
verifyGreaterThan(tc, order, 3.7);
verifyLessThan(tc, order, 4.3);
end

function testRK4MatchesODE45(tc)
I = [3 0.2 -0.1; 0.2 2 0.3; -0.1 0.3 4] * 1e-3; m = 0.5; g = [0;0;-9.81];
x0 = rbsim.body.pack([0;0;1], [1;0;2], rbsim.math.rotBodyFixed('XYZ', [0.1 0.2 0.3]), [3; -2; 5]);
f = @(t, x) rbsim.body.derivative(x, m, I, m*g, [0.001*cos(3*t); 0; 0]);
a = rbsim.solver.rk4(f, [0 2], x0, 1e-3, PostStep=@rbsim.body.projectState);
b = rbsim.solver.ode45ref(f, a.t, x0);
verifyEqual(tc, a.x, b.x, 'AbsTol', 1e-7);
end

function testStopFcnAndOutputDecimation(tc)
f = @(t, x) 1;
s = rbsim.solver.rk4(f, [0 1], 0, 0.1, OutputEvery=3, StopFcn=@(t, x) x >= 0.55);
verifyTrue(tc, s.solver.stoppedEarly);
verifyEqual(tc, s.t(end), 0.6, 'AbsTol', 1e-12);
s = rbsim.solver.rk4(f, [0 1], 0, 0.3);
verifyEqual(tc, s.t, [0 0.3 0.6 0.9 1], 'AbsTol', 1e-12);   % last step shortened
verifyEqual(tc, s.x(end), 1, 'AbsTol', 1e-12);
end
