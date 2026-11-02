function out = runExamples(opts)
%RUNEXAMPLES  Reproducible demonstrations of every application WITHOUT the GUI.
%
%   out = runExamples               prints a report
%   out = runExamples(Quiet=true)     returns results only
%
%   Shows that the core mechanics run headlessly (architecture rule) and gives the
%   reference numbers quoted in docs/validation/VALIDATION_REPORT.md.
    arguments
        opts.Quiet (1,1) logical = false
    end
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    run(fullfile(root, 'matlab', 'setup_path.m'));
    say = @(varargin) fprintf(varargin{:});
    if opts.Quiet, say = @(varargin) []; end

    % 1. Rotation conventions: body-fixed and space-fixed triples
    ang = deg2rad([60 45 -75]);
    out.Rbody = rbsim.math.rotBodyFixed('XYZ', ang);
    out.Rspace = rbsim.math.rotSpaceFixed('XYZ', ang);
    say('Body-fixed  X-Y-Z (60, 45, -75) deg:\n'); say('%9.4f %9.4f %9.4f\n', out.Rbody.');
    say('Space-fixed X-Y-Z (60, 45, -75) deg:\n'); say('%9.4f %9.4f %9.4f\n', out.Rspace.');

    % 2. Die: free flight in closed form, then with the floor
    p = rbsim.dice.defaults(); p.contact.enabled = false; p.tFinal = 0.3; p.dt = 1e-3;
    out.diceFree = rbsim.dice.simulate(p);
    say('\nDice free flight 0.3 s: energy drift %.2e J, |dH_A| %.2e\n', ...
        max(abs(out.diceFree.diag.energy - out.diceFree.diag.energy(1))), ...
        max(vecnorm(out.diceFree.diag.H_A - out.diceFree.diag.H_A(:, 1))));
    out.dice = rbsim.dice.simulate(rbsim.dice.defaults());
    s = out.dice.summary;
    say('Dice throw (default): top face %d, settled at %.3f s after %d impacts\n', ...
        s.finalFace, s.settleTime, s.bounces);

    % 3. Planar 3R: passive swing and computed-torque move
    p = rbsim.robot.defaults(); p.tFinal = 2;
    out.robotFree = rbsim.robot.simulate(p);
    say('\n3R passive swing 2 s: max |E - E0| = %.2e J\n', ...
        max(abs(out.robotFree.diag.energy - out.robotFree.diag.energy(1))));
    p.control.mode = 'trajectory'; p.tFinal = 2.5;
    out.robotMove = rbsim.robot.simulate(p);
    say('3R computed-torque move: tracking error %.2e rad, final tool [%.4f %.4f] m\n', ...
        out.robotMove.summary.trackingErrorMax, out.robotMove.summary.finalToolPosition(1:2));

    % 4. Four-bar: constant crank speed (inverse dynamics)
    p = rbsim.fourbar.defaults(); p.drive.mode = 'speed'; p.tFinal = 1;
    out.fourbar = rbsim.fourbar.simulate(p);
    s = out.fourbar.summary;
    say('\nFour-bar (%s) at 360 deg/s: max closure %.1e m, torque %.3f N m peak, energy balance %.1e J\n', ...
        s.type, s.maxClosureResidual, s.maxDriveTorque, s.maxEnergyBalanceError);
end
