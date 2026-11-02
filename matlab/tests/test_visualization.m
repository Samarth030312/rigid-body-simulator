function tests = test_visualization
%TEST_VISUALIZATION  Level 6: scenes and dashboards are driven by simulation state.
%   Runs headless (invisible figures). Verifies that drawn geometry equals the
%   simulated state at the displayed sample, not merely that plotting succeeds.
    tests = functiontests(localfunctions);
end

function setupOnce(tc)
    tc.TestData.fig = figure('Visible', 'off');
    p = rbsim.dice.defaults(); p.tFinal = 0.4; p.stopWhenSettled = false;
    tc.TestData.dice = rbsim.dice.simulate(p);
    p = rbsim.robot.defaults(); p.tFinal = 0.5;
    tc.TestData.robot = rbsim.robot.simulate(p);
    p = rbsim.fourbar.defaults(); p.tFinal = 0.5;
    tc.TestData.fourbar = rbsim.fourbar.simulate(p);
end

function teardownOnce(tc)
    close(tc.TestData.fig);
end

function testDiceTransformEqualsState(tc)
    res = tc.TestData.dice; ax = axes(clf(tc.TestData.fig));
    h = rbsim.viz.diceScene(ax, res);
    for k = [1, round(numel(res.t)/2), numel(res.t)]
        rbsim.viz.diceUpdate(h, k);
        [r, ~, R] = rbsim.body.unpack(res.x(:, k));
        tc.verifyEqual(h.tf.Matrix, [R r; 0 0 0 1], 'AbsTol', 1e-12);
        tc.verifyEqual(h.trail.XData(end), res.x(1, k));
        tc.verifyTrue(contains(h.label.String(1, :), sprintf('top face = %d', res.diag.topFace(k))));
    end
    % the drawn pips on the top face really point up: transform face-1 pips
    G = res.geometry; [~, ~, R] = rbsim.body.unpack(res.x(:, 1));
    [~, ~, f] = rbsim.dice.topFace(R, G);
    tc.verifyEqual(size(G.pips{f}, 2), res.diag.topFace(1));
end

function testDiceFollowCamera(tc)
    res = tc.TestData.dice; ax = axes(clf(tc.TestData.fig));
    h = rbsim.viz.diceScene(ax, res, true);
    k = numel(res.t); rbsim.viz.diceUpdate(h, k);
    tc.verifyEqual(mean(xlim(ax)), res.x(1, k), 'AbsTol', 1e-12);
end

function testRobotLinksEqualForwardKinematics(tc)
    res = tc.TestData.robot; ax = axes(clf(tc.TestData.fig));
    h = rbsim.viz.robotScene(ax, res);
    k = numel(res.t); rbsim.viz.robotUpdate(h, k);
    fk = rbsim.robot.forwardKinematics(res.x(1:3, k), res.params);
    for i = 1:3
        tc.verifyEqual([h.link(i).XData; h.link(i).YData], fk.O(1:2, i:i+1), 'AbsTol', 1e-14);
    end
    tc.verifyEqual([h.joints.XData; h.joints.YData], fk.O(1:2, :), 'AbsTol', 1e-14);
end

function testFourbarJointsEqualSolution(tc)
    res = tc.TestData.fourbar; ax = axes(clf(tc.TestData.fig));
    h = rbsim.viz.fourbarScene(ax, res);
    k = numel(res.t); rbsim.viz.fourbarUpdate(h, k);
    pos = rbsim.fourbar.position(res.x(1, k), res.params);
    P = [pos.OL pos.A pos.B pos.OR];
    tc.verifyEqual([h.joints.XData; h.joints.YData], P, 'AbsTol', 1e-14);
    tc.verifyEqual(norm(P(:,3) - P(:,2)), res.params.l(3), 'AbsTol', 1e-14);   % drawn coupler length
end

function testDashboardsBuildAndCursorsMove(tc)
    kinds = {'dice', 'robot', 'fourbar'};
    for i = 1:3
        f = clf(tc.TestData.fig);
        tl = tiledlayout(f, 2, 2);
        axs = [nexttile(tl), nexttile(tl), nexttile(tl), nexttile(tl)];
        res = tc.TestData.(kinds{i});
        cur = rbsim.viz.dashboard(axs, res, kinds{i});
        tc.verifyEqual(numel(cur), 4);
        rbsim.viz.setCursor(cur, res.t(end));
        tc.verifyEqual([cur.Value], repmat(res.t(end), 1, 4));
        for a = axs      % cursor never appears in a legend
            lg = a.Legend;
            if ~isempty(lg), tc.verifyFalse(any(contains(lg.String, 'data'))); end
        end
    end
end

function testThemeCategoricalOrderFixed(tc)
    th = rbsim.viz.theme();
    tc.verifyEqual(size(th.series), [8 3]);
    tc.verifyEqual(round(th.series(1, :) * 255), [42 120 214]);   % #2a78d6 slot 1
    tc.verifyEqual(th.link, th.series(1:3, :));                    % link i <-> slot i
end
