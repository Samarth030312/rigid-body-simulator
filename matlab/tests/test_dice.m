function tests = test_dice
%TEST_DICE Level 5 validation of the dice application.
tests = functiontests(localfunctions);
end

function p = freeFlight()
p = rbsim.dice.defaults();
p.contact.enabled = false;
p.dt = 1e-3; p.outputEvery = 1; p.tFinal = 0.5;
end

% ---------- geometry and face layout ----------
function testFaceLayoutIsTheStandardDie(tc)
%: +X 4, -X 3, +Y 2, -Y 5, +Z 1, -Z 6.
G = rbsim.dice.geometry(0.02);
expect = containers.Map({'1 0 0','-1 0 0','0 1 0','0 -1 0','0 0 1','0 0 -1'}, ...
    {4, 3, 2, 5, 1, 6});
for f = 1:6
    key = strtrim(sprintf('%d ', G.normals(:, f)));
    verifyEqual(tc, G.values(f), expect(key));
end
verifyEqual(tc, sort(G.values), 1:6);
for f = 1:6   % opposite faces sum to 7
    opp = find(all(G.normals == -G.normals(:, f), 1));
    verifyEqual(tc, G.values(f) + G.values(opp), 7);
end
end

function testFaceWindingAndPips(tc)
a = 0.02; G = rbsim.dice.geometry(a);
for f = 1:6
    V = G.vertices(:, G.faces(f, :));
    n = cross(V(:,2) - V(:,1), V(:,3) - V(:,2));
    verifyEqual(tc, n / norm(n), G.normals(:, f), 'AbsTol', 1e-14);  % CCW outward
    P = G.pips{f};
    verifyEqual(tc, size(P, 2), G.values(f));                     % pip count
    verifyEqual(tc, G.normals(:, f).' * P, (a/2) * ones(1, size(P, 2)), 'AbsTol', 1e-15);
    inPlane = P - G.normals(:, f) * (a/2);
    verifyLessThan(tc, max(abs(inPlane(:))) + G.pipRadius, a/2);   % pips inside face
end
end

function testTopFaceRule(tc)
G = rbsim.dice.geometry(0.016);
verifyEqual(tc, rbsim.dice.topFace(eye(3), G), 1);
verifyEqual(tc, rbsim.dice.topFace(rbsim.math.rotX(pi), G), 6);
verifyEqual(tc, rbsim.dice.topFace(rbsim.math.rotY(pi/2), G), 3);     % body -X points up
verifyEqual(tc, rbsim.dice.topFace(rbsim.math.rotY(-pi/2), G), 4);
verifyEqual(tc, rbsim.dice.topFace(rbsim.math.rotX(pi/2), G), 2);     % R(X,90) carries body +Y onto +Z_A
verifyEqual(tc, rbsim.dice.topFace(rbsim.math.rotX(-pi/2), G), 5);    % body -Y points up
[~, al] = rbsim.dice.topFace(rbsim.math.rotX(0.3), G);
verifyEqual(tc, al, cos(0.3), 'AbsTol', 1e-14);
end

% ---------- initial state ----------
function testInitialOrientationBodyAndSpace(tc)
p = rbsim.dice.defaults();
p.orientation = struct('type', 'body', 'sequence', 'XYZ', 'anglesDeg', [60 45 -75]);
[~, ~, R] = rbsim.body.unpack(rbsim.dice.initialState(p));
verifyEqual(tc, R, [0.1830 0.6830 0.7071; -0.3245 0.7209 -0.6124; -0.9280 -0.1174 0.3536], ...
    'AbsTol', 5e-5);                                       %
p.orientation.type = 'space';
[~, ~, R] = rbsim.body.unpack(rbsim.dice.initialState(p));
verifyEqual(tc, R, [0.1830 0.6415 -0.7450; -0.6830 -0.4621 -0.5657; -0.7071 0.6124 0.3536], ...
    'AbsTol', 5e-5);                                       %
end

function testInitialAngularVelocityFrames(tc)
p = rbsim.dice.defaults();
p.orientation.anglesDeg = [30 -20 50]; p.omega0 = [1; 2; 3];
p.omegaFrame = 'space';
[~, ~, R, wb] = rbsim.body.unpack(rbsim.dice.initialState(p));
verifyEqual(tc, R * wb, [1; 2; 3], 'AbsTol', 1e-14);       % w^s = R w^b
p.omegaFrame = 'body';
[~, ~, ~, wb] = rbsim.body.unpack(rbsim.dice.initialState(p));
verifyEqual(tc, wb, [1; 2; 3]);
end

function testParameterValidation(tc)
p = rbsim.dice.defaults(); p.mass = -1;
verifyError(tc, @() rbsim.dice.simulate(p), 'rbsim:dice:param');
p = rbsim.dice.defaults(); p.orientation.sequence = 'XXY';
verifyError(tc, @() rbsim.dice.simulate(p), 'rbsim:dice:param');
p = rbsim.dice.defaults(); p.r0 = [0; 0; 0.001];
verifyError(tc, @() rbsim.dice.simulate(p), 'rbsim:dice:param');          % starts inside floor
p = rbsim.dice.defaults(); p.dt = 1e-2;
verifyError(tc, @() rbsim.dice.simulate(p), 'rbsim:dice:param');          % unstable step
end

% ---------- free flight: the closed-form model ----------
function testFreeFlightAnalytic(tc)
% Newton: projectile; Euler with isotropic cube inertia: w_b constant, rotation
% about a fixed axis (R = R0 R(w,|w|t)).
p = freeFlight();
res = rbsim.dice.simulate(p);
x0 = rbsim.dice.initialState(p);
[r0, v0, R0, wb] = rbsim.body.unpack(x0);
t = res.t;
verifyEqual(tc, res.x(1:3, :), r0 + v0 .* t + 0.5 * p.g .* t.^2, 'AbsTol', 1e-12);
[~, ~, R, wEnd] = rbsim.body.unpack(res.x(:, end));
verifyEqual(tc, wEnd, wb, 'AbsTol', 1e-12);
% RK4 phase-error bound N (|w| dt)^5 / 120 = 500 * 0.033^5 / 120 ~ 1.6e-7
verifyEqual(tc, R, R0 * rbsim.math.rotAxisAngle(wb, norm(wb) * t(end)), 'AbsTol', 5e-7);
verifyLessThan(tc, max(abs(res.diag.energy - res.diag.energy(1))), 1e-12);
verifyLessThan(tc, max(vecnorm(res.diag.H_A - res.diag.H_A(:, 1))), 1e-15);
verifyEqual(tc, res.diag.contactCount, zeros(size(t)));
end

% ---------- contact extension ----------
function testRestingOnFaceStaticEquilibrium(tc)
% Placed flat, no velocity: settles with 4 vertices each carrying m g / 4,
% i.e. COM height a/2 - m g/(4k); face 1 up.
p = rbsim.dice.defaults();
p.orientation.anglesDeg = [0 0 0]; p.v0 = [0;0;0]; p.omega0 = [0;0;0];
p.r0 = [0; 0; p.edge/2 * sqrt(3) + 1e-4];
p.stopWhenSettled = false; p.tFinal = 1.5;
res = rbsim.dice.simulate(p);
k = p.contact.stiffness; g = -p.g(3);
verifyEqual(tc, res.x(3, end), p.edge/2 - p.mass*g/(4*k), 'AbsTol', 1e-7);
verifyEqual(tc, res.summary.finalFace, 1);
verifyTrue(tc, res.summary.settled);
verifyEqual(tc, res.diag.contactCount(end), 4);
end

function testContactIsPassive(tc)
% Total energy (kinetic + gravity + contact springs) must never increase:
% dampers and friction only dissipate (power argument).
res = rbsim.dice.simulate(rbsim.dice.defaults());
dE = diff(res.diag.energy);
verifyLessThan(tc, max(dE), 1e-12 * res.diag.energy(1));
verifyGreaterThan(tc, res.summary.energyDissipated, 0);
verifyLessThan(tc, max(res.diag.orthError), 1e-12);
end

function testOutcomeEmergesFromDynamics(tc)
% Different initial spins must produce different outcomes; the face is read
% from the simulated orientation, never prescribed.
p = rbsim.dice.defaults(); p.tFinal = 3;
spins = [18 -25 12; -30 10 5; 5 40 -20; 25 25 25; -12 -35 8; 40 -5 -15; 0 0 30; 10 -10 -40];
faces = zeros(1, size(spins, 1));
for i = 1:size(spins, 1)
    p.omega0 = spins(i, :).';
    res = rbsim.dice.simulate(p);
    verifyTrue(tc, res.summary.settled, sprintf('throw %d did not settle', i));
    verifyGreaterThan(tc, res.summary.finalAlignment, 0.999);
    [~, ~, R] = rbsim.body.unpack(res.x(:, end));
    verifyEqual(tc, res.summary.finalFace, rbsim.dice.topFace(R, res.geometry));
    faces(i) = res.summary.finalFace;
end
verifyGreaterThanOrEqual(tc, numel(unique(faces)), 3);
end

function testDeterministic(tc)
p = rbsim.dice.defaults();
a = rbsim.dice.simulate(p); b = rbsim.dice.simulate(p);
verifyEqual(tc, a.x, b.x);
verifyEqual(tc, a.summary, b.summary);
end

function testTimeStepConvergence(tc)
% Halving dt must not change the outcome, and the pre-contact trajectory must
% agree closely.
p = rbsim.dice.defaults(); p.stopWhenSettled = false; p.tFinal = 1.0;
a = rbsim.dice.simulate(p);
p.dt = p.dt / 2; p.outputEvery = 2 * p.outputEvery;
b = rbsim.dice.simulate(p);
verifyEqual(tc, a.t, b.t, 'AbsTol', 1e-12);
verifyEqual(tc, a.summary.finalFace, b.summary.finalFace);
firstContact = find(a.diag.contactCount > 0, 1);
verifyEqual(tc, a.x(:, 1:firstContact-1), b.x(:, 1:firstContact-1), 'AbsTol', 1e-10);
end
