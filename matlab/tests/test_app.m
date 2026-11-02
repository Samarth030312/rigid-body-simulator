function tests = test_app
%TEST_APP Level 7: GUI behaviour, driven headlessly through the app object.
tests = functiontests(localfunctions);
end

function setup(tc)
tc.TestData.app = RigidBodyLabApp('Visible', false);
end

function teardown(tc)
delete(tc.TestData.app);
end

function testFormDefaultsEqualCoreDefaults(tc)
% The GUI must not silently change the model: default form -> default params.
app = tc.TestData.app;
p = app.getParams('dice'); d = rbsim.dice.defaults();
verifyEqual(tc, p.edge, d.edge, 'RelTol', 1e-12);
verifyEqual(tc, p.mass, d.mass, 'RelTol', 1e-12);   % exact: bounces are sensitive to it
verifyEqual(tc, p.orientation, d.orientation);
verifyEqual(tc, p.omega0, d.omega0); verifyEqual(tc, p.omegaFrame, d.omegaFrame);
r = app.getParams('robot'); dr = rbsim.robot.defaults();
verifyEqual(tc, r.l, dr.l); verifyEqual(tc, r.r, dr.r, 'AbsTol', 1e-15);
verifyEqual(tc, r.Izz, dr.Izz, 'AbsTol', 1e-15); verifyEqual(tc, r.control, dr.control);
f = app.getParams('fourbar'); df = rbsim.fourbar.defaults();
verifyEqual(tc, f.l, df.l); verifyEqual(tc, f.drive, df.drive); verifyEqual(tc, f.branch, df.branch);
end

function testUnitConversionAtBoundary(tc)
app = tc.TestData.app;
app.setField('dice', 'edge', 20); app.setField('dice', 'mass', 8);
app.setField('dice', 'orientType', 'Space-fixed'); app.setField('dice', 'omegaFrame', 'Space {A}');
p = app.getParams('dice');
verifyEqual(tc, p.edge, 0.020, 'AbsTol', 1e-15);    % mm -> m
verifyEqual(tc, p.mass, 0.008, 'AbsTol', 1e-15);    % g -> kg
verifyEqual(tc, p.orientation.type, 'space');
verifyEqual(tc, p.omegaFrame, 'space');
end

function testRunDiceMatchesCore(tc)
% GUI result is identical to calling the core directly with the same params.
app = tc.TestData.app;
verifyTrue(tc, app.run('dice'));
res = app.Results.dice;
ref = rbsim.dice.simulate(app.getParams('dice'));
verifyEqual(tc, res.x, ref.x);
data = app.SummaryTable.dice.Data;
verifyEqual(tc, data{1, 2}, sprintf('%d', ref.summary.finalFace));
[r, ~, R] = rbsim.body.unpack(res.x(:, end));
verifyEqual(tc, app.SceneHandle.dice.tf.Matrix, [R r; 0 0 0 1], 'AbsTol', 1e-12);
end

function testRunRobotAndFourbar(tc)
app = tc.TestData.app;
app.setField('robot', 'mode', 'pd'); app.setField('robot', 'tFinal', 1);
verifyTrue(tc, app.run('robot'));
verifyEqual(tc, app.Results.robot.params.control.mode, 'pd');
app.setField('fourbar', 'mode', 'speed'); app.setField('fourbar', 'tFinal', 1);
verifyTrue(tc, app.run('fourbar'));
verifyLessThan(tc, app.Results.fourbar.summary.maxClosureResidual, 1e-12);
end

function testScrubbingShowsRequestedSample(tc)
app = tc.TestData.app;
app.setField('robot', 'tFinal', 1);
app.run('robot');
res = app.Results.robot; k = 17;
app.showSample('robot', k);
verifyEqual(tc, app.Slider.robot.Value, res.t(k), 'AbsTol', 1e-12);
fk = rbsim.robot.forwardKinematics(res.x(1:3, k), res.params);
verifyEqual(tc, app.SceneHandle.robot.joints.XData, fk.O(1, :), 'AbsTol', 1e-14);
verifyEqual(tc, app.Cursors.robot(1).Value, res.t(k), 'AbsTol', 1e-12);
end

function testInvalidInputReportedNotThrown(tc)
% Bad input: no exception escapes, no result stored, status explains the error.
app = tc.TestData.app;
app.setField('dice', 'mass', -5);
ok = app.run('dice');
verifyFalse(tc, ok);
verifyFalse(tc, isfield(app.Results, 'dice'));
verifyTrue(tc, startsWith(app.StatusLabel.dice.Text, 'Error: Mass'));
app.setField('fourbar', 'l23', [0.05 0.05]);      % cannot assemble
verifyFalse(tc, app.run('fourbar'));
verifyTrue(tc, contains(app.StatusLabel.fourbar.Text, 'cannot be assembled'));
end

function testResetRestoresDefaults(tc)
app = tc.TestData.app;
app.setField('robot', 'l', [1 1 1]);
app.run('robot');
app.resetForm('robot');
verifyEqual(tc, app.getParams('robot').l, rbsim.robot.defaults().l);
verifyFalse(tc, isfield(app.Results, 'robot'));
verifyEmpty(tc, app.SummaryTable.robot.Data);
end

function testPlaybackAdvancesAndStops(tc)
app = tc.TestData.app;
app.setField('fourbar', 'tFinal', 0.3);
app.run('fourbar');
app.showSample('fourbar', 1);
app.SpeedDrop.fourbar.Value = '2x';
app.play('fourbar');
t0 = tic;
while ~isempty(app.Playing) && toc(t0) < 20
    pause(0.05);
end
verifyEmpty(tc, app.Playing);                             % stopped by itself
verifyEqual(tc, app.Slider.fourbar.Value, app.Results.fourbar.t(end), 'AbsTol', 1e-12);
end

function testRandomThrowChangesInputsOnly(tc)
app = tc.TestData.app;
before = app.getParams('dice');
app.randomThrowForTest();
after = app.getParams('dice');
verifyNotEqual(tc, after.omega0, before.omega0);
verifyEqual(tc, after.edge, before.edge);
verifyFalse(tc, isfield(app.Results, 'dice'));            % nothing simulated yet
end
