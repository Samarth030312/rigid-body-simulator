classdef RigidBodyLabApp < handle
    %RIGIDBODYSIMULATORAPP Interactive GUI for the ENR209 rigid-body simulator.
    %
    %   app = RigidBodyLabApp()           % visible window
    %   app = RigidBodyLabApp('Visible', false)   % headless (tests)
    %
    %   Presentation/control layer only (ADR-0002, ADR-0014). Callbacks collect
    %   inputs from the form, convert units at this boundary (CONVENTIONS 5),
    %   call rbsim.dice.simulate / rbsim.robot.simulate / rbsim.fourbar.simulate, and hand the result
    %   to the visualization layer. No mechanics is computed here.
    %
    %   Public methods usable programmatically (and by tests):
    %     run(kind)            kind = 'dice' | 'robot' | 'fourbar'
    %     showSample(kind, k)  display sample k of the last result
    %     setField(kind, key, value) / getParams(kind) / resetForm(kind)
    %     play(kind) / pause() / delete()

    properties (SetAccess = private)
        UIFigure
        TabGroup
        Tabs struct = struct()      % per kind: tab handle
        Forms struct = struct()      % per kind: containers.Map key -> field handles
        Scene struct = struct()      % per kind: scene axes
        Dash struct = struct()      % per kind: 1x4 dashboard axes
        SceneHandle struct = struct()      % per kind: viz scene handle struct
        Cursors struct = struct()      % per kind: dashboard cursor lines
        Results struct = struct()      % per kind: last result struct
        Slider struct = struct()
        SummaryTable struct = struct()
        StatusLabel struct = struct()
        PlayButton struct = struct()
        SpeedDrop struct = struct()
        Timer
        Playing = ''
        ValidationTable
        ValidationLabel
    end

    properties (Constant)
        Kinds = {'dice', 'robot', 'fourbar'}
        Titles = {'Dice', '3R Manipulator', 'Four-Bar Mechanism'}
    end

    methods
        function app = RigidBodyLabApp(opts)
            arguments
                opts.Visible (1,1) logical = true
            end
            run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'setup_path.m'));
            app.UIFigure = uifigure('Name', 'ENR209 Rigid Body Simulator', ...
                'Position', [60 60 1440 860], 'Visible', 'off', ...
                'Color', rbsim.viz.theme().surface, 'CloseRequestFcn', @(~,~) app.delete());
            % The categorical palette is validated for a light surface; pin the
            % light theme so axis text stays readable when MATLAB is in dark mode.
            try
                theme(app.UIFigure, 'light');
            catch
                % theme() unavailable in older releases: default is light there
            end
            g = uigridlayout(app.UIFigure, [1 1], 'Padding', 0);
            app.TabGroup = uitabgroup(g);
            app.buildApplicationTab('dice', diceForm());
            app.buildApplicationTab('robot', robotForm());
            app.buildApplicationTab('fourbar', fourbarForm());
            app.buildValidationTab();
            app.buildAboutTab();
            app.Timer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.04, ...
                'TimerFcn', @(~,~) app.onTick(), 'BusyMode', 'drop');
            app.UIFigure.Visible = opts.Visible;
        end

        function delete(app)
            if ~isempty(app.Timer) && isvalid(app.Timer)
                stop(app.Timer); delete(app.Timer);
            end
            if ~isempty(app.UIFigure) && isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end

        % ------------------------------------------------------------ API
        function p = getParams(app, kind)
            %GETPARAMS Parameter struct for KIND from the current form values.
            v = app.formValues(kind);
            switch kind
                case 'dice',    p = diceParamsFromForm(v);
                case 'robot',   p = robotParamsFromForm(v);
                case 'fourbar', p = fourbarParamsFromForm(v);
            end
        end

        function setField(app, kind, key, value)
            %SETFIELD Set a form field (numeric vector, char for dropdowns, logical).
            h = app.Forms.(kind)(key);
            if iscell(h)
                for i = 1:numel(h), h{i}.Value = value(i); end
            else
                h.Value = value;
            end
        end

        function ok = run(app, kind)
            %RUN Simulate KIND with the form parameters and display the result.
            app.pause();
            ok = false;
            app.setStatus(kind, 'Simulating...', 'busy');
            drawnow limitrate;
            try
                p = app.getParams(kind);
                switch kind
                    case 'dice',    res = rbsim.dice.simulate(p);
                    case 'robot',   res = rbsim.robot.simulate(p);
                    case 'fourbar', res = rbsim.fourbar.simulate(p);
                end
            catch err
                app.setStatus(kind, ['Error: ' err.message], 'error');
                if strcmp(app.UIFigure.Visible, 'on')
                    uialert(app.UIFigure, err.message, 'Cannot run simulation');
                end
                return
            end
            app.Results.(kind) = res;
            app.render(kind);
            app.setStatus(kind, sprintf('Done: %d steps, %.2f s wall time (%s).', ...
                res.solver.steps, res.solver.wallTime, res.solver.name), 'ok');
            ok = true;
        end

        function showSample(app, kind, k)
            %SHOWSAMPLE Display sample K (index into res.t) of the last result.
            if ~isfield(app.Results, kind), return; end
            res = app.Results.(kind);
            k = max(1, min(numel(res.t), round(k)));
            h = app.SceneHandle.(kind);
            switch kind
                case 'dice',    rbsim.viz.diceUpdate(h, k);
                case 'robot',   rbsim.viz.robotUpdate(h, k);
                case 'fourbar', rbsim.viz.fourbarUpdate(h, k);
            end
            rbsim.viz.setCursor(app.Cursors.(kind), res.t(k));
            app.Slider.(kind).Value = res.t(k);
        end

        function resetForm(app, kind)
            %RESETFORM Restore default parameters and clear the displays.
            app.pause();
            spec = formSpec(kind);
            for i = 1:size(spec, 1)
                app.setField(kind, spec{i, 1}, spec{i, 3});
            end
            if isfield(app.Results, kind)
                app.Results = rmfield(app.Results, kind);
            end
            cla(app.Scene.(kind), 'reset');
            for a = app.Dash.(kind), cla(a, 'reset'); end
            app.SummaryTable.(kind).Data = cell(0, 2);
            app.setStatus(kind, 'Parameters reset to defaults.', 'ok');
        end

        function play(app, kind)
            if ~isfield(app.Results, kind), return; end
            app.Playing = kind;
            res = app.Results.(kind);
            if app.Slider.(kind).Value >= res.t(end) - 1e-12
                app.showSample(kind, 1);
            end
            app.PlayButton.(kind).Text = 'Pause';
            if strcmp(app.Timer.Running, 'off'), start(app.Timer); end
        end

        function pause(app)
            if ~isempty(app.Timer) && isvalid(app.Timer) && strcmp(app.Timer.Running, 'on')
                stop(app.Timer);
            end
            if ~isempty(app.Playing)
                app.PlayButton.(app.Playing).Text = 'Play';
            end
            app.Playing = '';
        end

        function randomThrowForTest(app)
            %RANDOMTHROWFORTEST Same as the 'Random throw' button (test hook).
            app.randomThrow();
        end

        function results = runValidation(app)
            %RUNVALIDATION Run the full test suite and list the results.
            app.ValidationLabel.Text = 'Running test suite...';
            drawnow;
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            suite = matlab.unittest.TestSuite.fromFolder(fullfile(root, 'tests'), ...
                'IncludingSubfolders', true);
            results = run(matlab.unittest.TestRunner.withNoPlugins(), suite);
            status = repmat({'PASS'}, numel(results), 1);
            status([results.Failed]) = {'FAIL'};
            status([results.Incomplete]) = {'INCOMPLETE'};
            app.ValidationTable.Data = [{results.Name}.', status, ...
                num2cell(round([results.Duration].', 3))];
            app.ValidationLabel.Text = sprintf('%d tests: %d passed, %d failed, %d incomplete (MATLAB %s).', ...
                numel(results), nnz([results.Passed]), nnz([results.Failed]), ...
                nnz([results.Incomplete]), version('-release'));
        end
    end

    methods (Access = private)
        % ------------------------------------------------------ building
        function buildApplicationTab(app, kind, ~)
            idx = find(strcmp(app.Kinds, kind));
            tab = uitab(app.TabGroup, 'Title', app.Titles{idx});
            app.Tabs.(kind) = tab;
            g = uigridlayout(tab, [1 3], 'ColumnWidth', {330, '1x', '1.1x'}, ...
                'Padding', [8 8 8 8], 'ColumnSpacing', 8);
            % left: form + buttons + summary
            left = uigridlayout(g, [4 1], 'RowHeight', {'1x', 30, 22, 190}, 'Padding', 0);
            formPanel = uipanel(left, 'Title', 'Parameters', 'Scrollable', 'on');
            app.Forms.(kind) = app.buildForm(formPanel, formSpec(kind));
            btns = uigridlayout(left, [1 3], 'Padding', 0, 'ColumnSpacing', 6);
            uibutton(btns, 'Text', 'Run', 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.run(kind));
            uibutton(btns, 'Text', 'Reset', 'ButtonPushedFcn', @(~,~) app.resetForm(kind));
            if strcmp(kind, 'dice')
                uibutton(btns, 'Text', 'Random throw', ...
                    'ButtonPushedFcn', @(~,~) app.randomThrow());
            else
                uilabel(btns, 'Text', '');
            end
            app.StatusLabel.(kind) = uilabel(left, 'Text', 'Set parameters and press Run.', ...
                'FontColor', rbsim.viz.theme().textMuted, 'WordWrap', 'on');
            app.SummaryTable.(kind) = uitable(left, 'ColumnName', {'Result', 'Value'}, ...
                'ColumnWidth', {170, 'auto'}, 'RowName', {}, 'Data', cell(0, 2));
            % middle: scene + playback
            mid = uigridlayout(g, [2 1], 'RowHeight', {'1x', 34}, 'Padding', 0);
            app.Scene.(kind) = uiaxes(mid);
            title(app.Scene.(kind), 'Press Run to simulate', 'FontWeight', 'normal');
            pb = uigridlayout(mid, [1 4], 'ColumnWidth', {70, '1x', 60, 80}, 'Padding', 0);
            app.PlayButton.(kind) = uibutton(pb, 'Text', 'Play', ...
                'ButtonPushedFcn', @(~,~) app.togglePlay(kind));
            app.Slider.(kind) = uislider(pb, 'Limits', [0 1], 'MajorTicks', [], ...
                'MinorTicks', [], 'ValueChangingFcn', @(~, e) app.onSlide(kind, e.Value), ...
                'ValueChangedFcn', @(s, ~) app.onSlide(kind, s.Value));
            uilabel(pb, 'Text', 'speed', 'HorizontalAlignment', 'right');
            app.SpeedDrop.(kind) = uidropdown(pb, 'Items', {'0.1x', '0.25x', '0.5x', '1x', '2x'}, ...
                'Value', '1x');
            % right: dashboard
            dash = uigridlayout(g, [2 2], 'Padding', 0, 'RowSpacing', 6, 'ColumnSpacing', 6);
            ax = gobjects(1, 4);
            for i = 1:4, ax(i) = uiaxes(dash); end
            app.Dash.(kind) = ax;
        end

        function fields = buildForm(~, parent, spec)
            fields = containers.Map();
            n = size(spec, 1);
            g = uigridlayout(parent, [n 4], 'ColumnWidth', {118, '1x', '1x', '1x'}, ...
                'RowHeight', repmat({22}, 1, n), 'Padding', [4 4 4 4], 'RowSpacing', 4, ...
                'ColumnSpacing', 4, 'Scrollable', 'on');
            for i = 1:n
                [key, label, def, kindOf] = spec{i, 1:4};
                l = uilabel(g, 'Text', label, 'Tooltip', spec{i, 5});
                l.Layout.Row = i; l.Layout.Column = 1;
                switch kindOf
                    case 'num'
                        h = cell(1, numel(def));
                        for j = 1:numel(def)
                            h{j} = uieditfield(g, 'numeric', 'Value', def(j), ...
                                'Tooltip', spec{i, 5});
                            h{j}.Layout.Row = i; h{j}.Layout.Column = 1 + j;
                        end
                        fields(key) = h;
                    case 'drop'
                        h = uidropdown(g, 'Items', spec{i, 6}, 'Value', def, 'Tooltip', spec{i, 5});
                        h.Layout.Row = i; h.Layout.Column = [2 4];
                        fields(key) = h;
                    case 'check'
                        h = uicheckbox(g, 'Text', '', 'Value', def, 'Tooltip', spec{i, 5});
                        h.Layout.Row = i; h.Layout.Column = 2;
                        fields(key) = h;
                end
            end
        end

        function buildValidationTab(app)
            tab = uitab(app.TabGroup, 'Title', 'Validation');
            g = uigridlayout(tab, [3 1], 'RowHeight', {30, 22, '1x'}, 'Padding', [10 10 10 10]);
            uibutton(g, 'Text', 'Run full validation suite (about 1 minute)', ...
                'ButtonPushedFcn', @(~,~) app.runValidation());
            app.ValidationLabel = uilabel(g, 'Text', ...
                'Runs every automated test (mathematics, kinematics, dynamics, applications, visualization, GUI).');
            app.ValidationTable = uitable(g, 'ColumnName', {'Test', 'Result', 'Time (s)'}, ...
                'ColumnWidth', {'1x', 100, 80}, 'RowName', {}, 'Data', cell(0, 3));
        end

        function buildAboutTab(app)
            tab = uitab(app.TabGroup, 'Title', 'Conventions');
            g = uigridlayout(tab, [1 1], 'Padding', [12 12 12 12]);
            uitextarea(g, 'Editable', 'off', 'FontName', 'monospace', 'Value', aboutText());
        end

        % ------------------------------------------------------ helpers
        function v = formValues(app, kind)
            v = struct();
            m = app.Forms.(kind);
            keys = m.keys;
            for i = 1:numel(keys)
                h = m(keys{i});
                if iscell(h)
                    v.(keys{i}) = cellfun(@(c) c.Value, h);
                else
                    v.(keys{i}) = h.Value;
                end
            end
        end

        function render(app, kind)
            res = app.Results.(kind);
            switch kind
                case 'dice'
                    follow = app.Forms.dice('follow').Value;
                    app.SceneHandle.dice = rbsim.viz.diceScene(app.Scene.dice, res, follow);
                case 'robot'
                    app.SceneHandle.robot = rbsim.viz.robotScene(app.Scene.robot, res);
                case 'fourbar'
                    app.SceneHandle.fourbar = rbsim.viz.fourbarScene(app.Scene.fourbar, res);
            end
            app.Cursors.(kind) = rbsim.viz.dashboard(app.Dash.(kind), res, kind);
            app.Slider.(kind).Limits = [res.t(1), max(res.t(end), res.t(1) + eps)];
            app.SummaryTable.(kind).Data = summaryRows(kind, res);
            app.showSample(kind, numel(res.t));
        end

        function setStatus(app, kind, msg, level)
            th = rbsim.viz.theme();
            switch level
                case 'error', c = th.series(8, :);
                case 'busy',  c = th.series(1, :);
                otherwise,    c = th.textMuted;
            end
            app.StatusLabel.(kind).Text = msg;
            app.StatusLabel.(kind).FontColor = c;
        end

        function togglePlay(app, kind)
            if strcmp(app.Playing, kind), app.pause(); else, app.play(kind); end
        end

        function onSlide(app, kind, t)
            if ~isfield(app.Results, kind), return; end
            [~, k] = min(abs(app.Results.(kind).t - t));
            app.showSample(kind, k);
        end

        function onTick(app)
            kind = app.Playing;
            if isempty(kind) || ~isvalid(app.UIFigure), return; end
            res = app.Results.(kind);
            factor = sscanf(app.SpeedDrop.(kind).Value, '%fx');
            tNext = app.Slider.(kind).Value + factor * app.Timer.Period;
            if tNext >= res.t(end)
                app.showSample(kind, numel(res.t));
                app.pause();
                return
            end
            [~, k] = min(abs(res.t - tNext));
            if res.t(k) <= app.Slider.(kind).Value, k = min(k + 1, numel(res.t)); end
            app.showSample(kind, k);
            drawnow limitrate;
        end

        function randomThrow(app)
            % Random but reproducible initial orientation and spin (seed shown).
            seed = randi(1e6);
            s = RandStream('twister', 'Seed', seed);
            app.setField('dice', 'angles', round(360 * rand(s, 1, 3) - 180));
            app.setField('dice', 'omega0', round(60 * rand(s, 1, 3) - 30, 1));
            app.setField('dice', 'v0', [round(rand(s) - 0.5, 2), round(rand(s) - 0.5, 2), round(rand(s), 2)]);
            app.setStatus('dice', sprintf('Random throw generated (seed %d). Press Run.', seed), 'ok');
        end
    end
end

% ======================================================================
% Form specifications: {key, label, default, type, tooltip, items}
% Units shown in labels; converted in *ParamsFromForm (GUI boundary).
function spec = formSpec(kind)
switch kind
    case 'dice',    spec = diceForm();
    case 'robot',   spec = robotForm();
    case 'fourbar', spec = fourbarForm();
end
end

function spec = diceForm()
d = rbsim.dice.defaults();
seqs = {'XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX', 'XYX', 'XZX', 'YXY', 'YZY', 'ZXZ', 'ZYZ'};
spec = {
 'edge',    'Edge a (mm)',          d.edge * 1e3, 'num', 'Cube edge length', {}
 'mass',    'Mass m (g)',           d.mass * 1e3, 'num', 'Mass; inertia = m a^2/6 [U] (L14-15/s9)', {}
 'r0',      'Position r_C (m)',     d.r0.', 'num', 'Initial COM position in {A} (x, y, z)', {}
 'orientType', 'Rotation type',     'Body-fixed', 'drop', 'Body-fixed = post-multiply, space-fixed = pre-multiply (L2-4/s21-36)', {'Body-fixed', 'Space-fixed'}
 'sequence','Sequence',             d.orientation.sequence, 'drop', 'Order of the three simple rotations', seqs
 'angles',  'Angles (deg)',         d.orientation.anglesDeg, 'num', 'Rotation angles, first rotation first', {}
 'v0',      'Velocity V_C (m/s)',   d.v0.', 'num', 'Initial COM velocity in {A}', {}
 'omega0',  'Ang. vel. (rad/s)',    d.omega0.', 'num', 'Initial angular velocity components', {}
 'omegaFrame', 'omega components',  'Body {B}', 'drop', 'Body-fixed w^b (along {B}) or space-fixed w^s (along {A})', {'Body {B}', 'Space {A}'}
 'contact', 'Floor contact',        true, 'check', 'EXTENSION (ADR-0012): spring-damper-friction floor. Off = pure free flight (lecture model)', {}
 'stiffness','Stiffness k (N/m)',   d.contact.stiffness, 'num', 'Contact spring per vertex', {}
 'damping', 'Damping c (N s/m)',    d.contact.damping, 'num', 'Contact damper per vertex', {}
 'friction','Friction mu',          d.contact.friction, 'num', 'Coulomb friction coefficient', {}
 'tFinal',  'Duration (s)',         d.tFinal, 'num', 'Simulated time (stops early once at rest)', {}
 'dt',      'Time step (ms)',       d.dt * 1e3, 'num', 'RK4 step', {}
 'follow',  'Follow camera',        false, 'check', 'Zoom the 3D view onto the die', {}
};
end

function spec = robotForm()
d = rbsim.robot.defaults();
spec = {
 'l',     'Lengths l_i (m)',       d.l, 'num', 'Link lengths', {}
 'm',     'Masses m_i (kg)',       d.m, 'num', 'Link masses', {}
 'rFrac', 'COM r_i/l_i',           d.r ./ d.l, 'num', 'COM position along each link (0..1)', {}
 'width', 'Width w_i (m)',         d.width, 'num', 'For inertia I_i = m(l^2+w^2)/12', {}
 'q0',    'Initial q (deg)',       d.q0, 'num', 'Relative joint angles (L7-9/s49)', {}
 'qd0',   'Initial qdot (deg/s)',  d.qd0, 'num', 'Joint rates', {}
 'mode',  'Control mode',          'free', 'drop', 'free / constant torque / PD / computed-torque trajectory', {'free', 'constant', 'pd', 'trajectory'}
 'tau',   'Torque tau (N m)',      d.control.tau, 'num', 'Constant joint torques (constant mode)', {}
 'target','Target q (deg)',        d.control.qTargetDeg, 'num', 'PD / trajectory target', {}
 'Kp',    'Kp (N m/rad)',          d.control.Kp, 'num', 'Proportional gains', {}
 'Kd',    'Kd (N m s/rad)',        d.control.Kd, 'num', 'Derivative gains', {}
 'gcomp', 'Gravity compensation',  d.control.gravityComp, 'check', 'Add G(q) in PD mode', {}
 'duration','Trajectory time (s)', d.control.duration, 'num', 'Quintic move duration', {}
 'damping','Joint damping (N m s)',d.damping, 'num', 'Viscous damping per joint', {}
 'g',     'Gravity g (m/s^2)',     d.g, 'num', 'Acts along -Y_0', {}
 'tFinal','Duration (s)',          d.tFinal, 'num', 'Simulated time', {}
};
end

function spec = fourbarForm()
d = rbsim.fourbar.defaults();
spec = {
 'l',      'l0 l1 (m)',            d.l(1:2), 'num', 'Ground and crank length (L5-6/s20)', {}
 'l23',    'l2 l3 (m)',            d.l(3:4), 'num', 'Coupler and rocker length', {}
 'm',      'Masses m1..m3 (kg)',   d.m, 'num', 'Crank, coupler, rocker', {}
 'branch', 'Assembly branch',      'open', 'drop', 'open = configuration of L5-6/s20', {'open', 'crossed'}
 'theta0', 'Crank angle (deg)',    d.theta0Deg, 'num', 'Initial theta1', {}
 'thetaDot0','Crank rate (deg/s)', d.thetaDot0Deg, 'num', 'Initial crank rate (free/torque)', {}
 'mode',   'Drive',                'free', 'drop', 'free / constant torque / constant speed', {'free', 'torque', 'speed'}
 'torque', 'Drive torque (N m)',   d.drive.torque, 'num', 'torque mode', {}
 'speed',  'Crank speed (deg/s)',  d.drive.speedDeg, 'num', 'speed mode (kinematic drive)', {}
 'load',   'Load on rocker (N m)', d.drive.loadTorque, 'num', 'Torque applied to link 3', {}
 'damping','Crank damping (N m s)',d.drive.damping, 'num', 'Viscous damping at O_L', {}
 'g',      'Gravity g (m/s^2)',    d.g, 'num', 'Acts along -Y_0', {}
 'tFinal', 'Duration (s)',         d.tFinal, 'num', 'Simulated time', {}
};
end

% ---------------- form -> params (unit conversion at the GUI boundary)
function p = diceParamsFromForm(v)
p = rbsim.dice.defaults();
p.edge = v.edge * 1e-3; p.mass = v.mass * 1e-3;
p.r0 = v.r0(:); p.v0 = v.v0(:); p.omega0 = v.omega0(:);
types = struct('Body_fixed', 'body', 'Space_fixed', 'space');
p.orientation = struct('type', types.(strrep(v.orientType, '-', '_')), ...
    'sequence', v.sequence, 'anglesDeg', v.angles);
if startsWith(v.omegaFrame, 'Body'), p.omegaFrame = 'body'; else, p.omegaFrame = 'space'; end
p.contact.enabled = v.contact;
p.contact.stiffness = v.stiffness; p.contact.damping = v.damping; p.contact.friction = v.friction;
p.tFinal = v.tFinal; p.dt = v.dt * 1e-3;
if ~p.contact.enabled, p.dt = max(p.dt, 1e-3); p.outputEvery = 2; end
end

function p = robotParamsFromForm(v)
p = rbsim.robot.defaults();
p.l = v.l; p.m = v.m; p.width = v.width;
p.r = v.rFrac .* v.l;
p.Izz = p.m .* (p.l.^2 + p.width.^2) / 12;
p.q0 = v.q0; p.qd0 = v.qd0;
p.control = struct('mode', v.mode, 'tau', v.tau, 'qTargetDeg', v.target, 'Kp', v.Kp, ...
    'Kd', v.Kd, 'gravityComp', v.gcomp, 'duration', v.duration);
p.damping = v.damping; p.g = v.g; p.tFinal = v.tFinal;
end

function p = fourbarParamsFromForm(v)
p = rbsim.fourbar.defaults();
p.l = [v.l, v.l23]; p.m = v.m;
p.r = p.l(2:4) / 2;
p.Izz = p.m .* (p.l(2:4).^2 + p.width.^2) / 12;
p.branch = 1 - 2 * strcmp(v.branch, 'crossed');
p.theta0Deg = v.theta0; p.thetaDot0Deg = v.thetaDot0;
p.drive = struct('mode', v.mode, 'torque', v.torque, 'speedDeg', v.speed, ...
    'loadTorque', v.load, 'damping', v.damping);
p.g = v.g; p.tFinal = v.tFinal;
end

% ---------------- summary tables
function rows = summaryRows(kind, res)
s = res.summary; d = res.diag;
fmt = @(x) sprintf('%.4g', x);
switch kind
    case 'dice'
        settled = 'no (still moving at end)'; if s.settled, settled = sprintf('yes, t = %.3f s', s.settleTime); end
        rows = {'Top face (outcome)', sprintf('%d', s.finalFace);
                'Settled', settled;
                'Face alignment', fmt(s.finalAlignment);
                'Contact episodes (sampled)', sprintf('%d', s.bounces);
                'Max height (m)', fmt(s.maxHeight);
                'Energy dissipated (J)', fmt(s.energyDissipated);
                'Max ||R''R - I||', fmt(max(d.orthError));
                'Max SO(3) correction', fmt(res.solver.maxCorrection);
                'RK4 steps / wall (s)', sprintf('%d / %.2f', res.solver.steps, res.solver.wallTime)};
    case 'robot'
        rows = {'Final tool position (m)', sprintf('[%.4f, %.4f]', s.finalToolPosition(1:2));
                'Final q (deg)', sprintf('[%.2f, %.2f, %.2f]', rad2deg(res.x(1:3, end)));
                'Max |tau| (N m)', fmt(max(abs(d.tau(:))));
                'Energy balance error (J)', fmt(s.maxEnergyBalanceError);
                'Tracking error (rad)', fmt(s.trackingErrorMax);
                'RK4 steps / wall (s)', sprintf('%d / %.2f', res.solver.steps, res.solver.wallTime)};
    case 'fourbar'
        rows = {'Grashof type', s.type;
                'Rocker range (deg)', sprintf('%.2f to %.2f', s.outputRangeDeg);
                'Min transmission angle (deg)', fmt(s.minTransmissionDeg);
                'Max drive torque (N m)', fmt(s.maxDriveTorque);
                'Max closure residual (m)', fmt(s.maxClosureResidual);
                'Energy balance error (J)', fmt(s.maxEnergyBalanceError);
                'RK4 steps / wall (s)', sprintf('%d / %.2f', res.solver.steps, res.solver.wallTime)};
end
end

function txt = aboutText()
txt = {
'ENR209 Rigid Body Simulator - conventions (docs/architecture/CONVENTIONS.md)'
''
'Frames      right-handed Cartesian; {A}/{0} fixed, {B} body-fixed at the COM'
'Rotation    ^A_B[R] = [^A X_B | ^A Y_B | ^A Z_B],  ^A r = ^A_B[R] ^B r      (L2-4/s8-9)'
'Body-fixed  R = R(ax1,t1) R(ax2,t2) R(ax3,t3)   (post-multiply)           (L2-4/s21,s25)'
'Space-fixed R = R(ax3,t3) R(ax2,t2) R(ax1,t1)   (pre-multiply)            (L2-4/s36)'
'Ang. vel.   [w^s x] = Rdot R''   (space-fixed),  [w^b x] = R'' Rdot (body)  (L7-9/s5-10)'
'Newton      m a_C = F                                                    (L17-19/s6)'
'Euler       ^B[I]_C wdot^b + w^b x ^B[I]_C w^b = ^B M                     (L16/s6)'
'Cube        ^B[I]_C = (m a^2 / 6) [U]                                     (L14-15/s9)'
'Gravity     9.81 m/s^2 along -Z_A (3D) and -Y_0 (planar)'
'Dice faces  +X 4, -X 3, +Y 2, -Y 5, +Z 1, -Z 6                             (L2-4/s13)'
'3R          planar, relative joint angles, recursive Newton-Euler           (L7-9/s49, L20-21/s10-11)'
'Four-bar    Freudenstein constraint eta1(theta1, phi1) = 0                 (L5-6/s22)'
'Integration fixed-step RK4 with SO(3) re-orthonormalisation                 (ADR-0009)'
''
'Engineering extensions (not in the lectures): dice floor contact, four-bar dynamics'
'formulation, controllers, Grashof classification, outcome rule. See docs/DECISIONS.md.'
};
end
