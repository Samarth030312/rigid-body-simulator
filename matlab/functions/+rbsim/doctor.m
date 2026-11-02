function report = doctor(opts)
%DOCTOR  Check that this computer can run the simulator, and say what is missing.
%
%   rbsim.doctor                 print the checks
%   r = rbsim.doctor('Quiet',true)   return them without printing
%
%   Each check is a struct with fields check, status (PASS / WARN / FAIL),
%   detail and fix. The simulator needs base MATLAB only: no toolbox is used,
%   and the tests state that explicitly (test_end_to_end).
    arguments
        opts.Quiet (1,1) logical = false
    end
    c = @(name, ok, detail, fix) struct('check', name, ...
        'status', string(ok), 'detail', detail, 'fix', fix);
    report = c('MATLAB release', relStatus(), ...
        sprintf('%s (%s)', version('-release'), computer('arch')), ...
        'R2021b or newer is required for the argument blocks and uifigure app.');
    report(end+1) = c('Repository on the path', ...
        ternary(~isempty(which('rbsim.body.derivative')), 'PASS', 'FAIL'), ...
        ternary(~isempty(which('rbsim.body.derivative')), 'rbsim package found', 'rbsim package not found'), ...
        'Run setup from the repository root.');
    report(end+1) = c('Graphics', ternary(usejava('desktop') || ~isempty(getenv('DISPLAY')) ...
        || ispc || ismac, 'PASS', 'WARN'), 'figures and the app need a display', ...
        'Without a display the simulations still run; only the app and figures need one.');
    report(end+1) = c('Toolboxes', 'PASS', 'none required (base MATLAB)', '');
    if ~opts.Quiet
        fprintf('\n  Rigid-body simulator  ·  %s\n', ...
            fileparts(fileparts(fileparts(mfilename('fullpath')))));
        for k = 1:numel(report)
            fprintf('  %-4s %-24s %s\n', report(k).status, report(k).check, report(k).detail);
        end
        fprintf('\n');
    end
end

function s = relStatus()
    v = sscanf(version('-release'), '%d');
    s = ternary(v >= 2021, 'PASS', 'WARN');
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
