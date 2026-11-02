%% Rigid-body simulator - playground
% Edit any value below and run a section with Ctrl+Enter (Run Section).
% Each section: parameters -> simulate -> animated scene + dashboard.
% All units SI except the angles marked "Deg" (degrees, converted inside).
% Set ANIMATE = false to only show the final frame.
clear; close all;
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(root, 'matlab', 'setup_path.m'));
ANIMATE = true;

%% 1. DICE - try other orientations, spins, contact settings
p = rbsim.dice.defaults();
p.edge   = 0.016;                 % m   (cube edge a)
p.mass   = 1150 * p.edge^3;       % kg  (inertia = m a^2/6 is automatic)
p.r0     = [0; 0; 0.15];          % m   initial COM position
p.orientation.type      = 'body'; % 'body' (post-multiply) or 'space' (pre-multiply)
p.orientation.sequence  = 'XYZ';  % any valid sequence: 'ZYZ', 'ZYX', ...
p.orientation.anglesDeg = [20 -35 10];
p.v0     = [0.4; 0.1; 0.5];       % m/s
p.omega0 = [18; -25; 12];         % rad/s
p.omegaFrame = 'body';            % 'body' = w^b along {B}, 'space' = w^s along {A}
p.contact.enabled = true;         % false = pure free flight (no floor)
p.contact.friction = 0.35;
res = rbsim.dice.simulate(p);
fprintf('Dice: top face %d, settled = %d\n', res.summary.finalFace, res.summary.settled);
showResult(res, 'dice', ANIMATE);

%% 2. PLANAR 3R MANIPULATOR - modes: 'free' 'constant' 'pd' 'trajectory'
p = rbsim.robot.defaults();
p.l = [0.5 0.4 0.3];              % m    link lengths
p.m = [3.0 2.0 1.0];              % kg   link masses
p.r = p.l / 2;                    % m    COM distances from each joint
p.Izz = p.m .* (p.l.^2 + p.width.^2) / 12;   % keep consistent if you change l or m
p.q0 = [30 -20 10];               % deg  initial joint angles (relative)
p.control.mode = 'trajectory';
p.control.qTargetDeg = [90 -45 -30];
p.control.duration = 2;           % s    trajectory move time
p.damping = [0 0 0];              % N m s/rad
p.tFinal = 4;
res = rbsim.robot.simulate(p);
fprintf('3R: final tool = [%.4f %.4f] m\n', res.summary.finalToolPosition(1:2));
showResult(res, 'robot', ANIMATE);

%% 3. FOUR-BAR MECHANISM - drives: 'free' 'torque' 'speed'
p = rbsim.fourbar.defaults();
p.l = [0.40 0.12 0.36 0.28];      % m    [ground l0, crank l1, coupler l2, rocker l3]
p.r = p.l(2:4) / 2;
p.Izz = p.m .* (p.l(2:4).^2 + p.width.^2) / 12;
p.branch = 1;                     % +1 open, -1 crossed
p.theta0Deg = 60;
p.drive.mode = 'speed';
p.drive.speedDeg = 360;           % deg/s  crank speed (speed mode)
p.drive.torque = 0.5;             % N m    (torque mode)
p.tFinal = 2;
[g, type] = rbsim.fourbar.grashof(p.l);
fprintf('Four-bar: %s (Grashof = %d)\n', type, g);
res = rbsim.fourbar.simulate(p);
fprintf('Four-bar: max closure residual %.1e m\n', res.summary.maxClosureResidual);
showResult(res, 'fourbar', ANIMATE);

%% helper
function showResult(res, kind, animate)
    f = figure('Name', kind, 'Position', [60 60 1300 650], 'Color', rbsim.viz.theme().surface);
    tl = tiledlayout(f, 2, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [2 2]);
    switch kind
        case 'dice',    h = rbsim.viz.diceScene(ax, res);    upd = @rbsim.viz.diceUpdate;
        case 'robot',   h = rbsim.viz.robotScene(ax, res);   upd = @rbsim.viz.robotUpdate;
        case 'fourbar', h = rbsim.viz.fourbarScene(ax, res); upd = @rbsim.viz.fourbarUpdate;
    end
    cur = rbsim.viz.dashboard([nexttile(tl, 3), nexttile(tl, 4), nexttile(tl, 7), nexttile(tl, 8)], res, kind);
    ks = numel(res.t);
    if animate, ks = unique(round(linspace(1, numel(res.t), min(numel(res.t), 250)))); end
    for k = ks
        if ~isvalid(f), return; end
        upd(h, k); rbsim.viz.setCursor(cur, res.t(k)); drawnow limitrate;
    end
end
