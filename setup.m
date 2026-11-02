function out = setup(varargin)
%SETUP  Get the rigid-body simulator ready on this computer, and open the lab app.
%
%   setup            put the code on the path and check this computer
%   setup open       the same, then open the app (rigid_body_lab)
%   setup test       run the whole test suite and print a summary
%   setup figures    regenerate the figures in docs/img
%   setup save       keep the path for future MATLAB sessions (savepath)
%
%   Works on Windows, macOS and Linux and needs base MATLAB only: no toolbox,
%   no compiler, nothing to install. Nothing outside this folder is changed
%   except, with 'save', the MATLAB path.
    here = fileparts(mfilename('fullpath'));
    run(fullfile(here, 'matlab', 'setup_path.m'));
    a = lower(string(varargin));
    r = rbsim.doctor('Quiet', true);
    fprintf('\nRigid-body simulator  ·  %s\n', here);
    bad = r(string({r.status}) ~= "PASS");
    for k = 1:numel(bad)
        fprintf('  %-4s %s: %s\n', bad(k).status, bad(k).check, bad(k).detail);
        if ~isempty(bad(k).fix), fprintf('       -> %s\n', bad(k).fix); end
    end
    if isempty(bad), fprintf('  All checks passed (MATLAB %s, no toolbox needed).\n', version('-release')); end
    if any(a == "save"), savepath; fprintf('  Path saved for future sessions.\n'); end
    res = []; h = [];
    if any(a == "test"), res = runTests(here); end
    if any(a == "figures"), rigid_body_figures(); end
    if any(a == "open")
        h = rigid_body_lab();
    elseif ~any(a == "test")
        fprintf('  Open the app:  setup open    ·  run the tests:  setup test\n\n');
    end
    if nargout, out = struct('report', r, 'app', h, 'tests', res); end
end

function results = runTests(here)
    suite = matlab.unittest.TestSuite.fromFolder(fullfile(here, 'matlab', 'tests'), ...
        'IncludingSubfolders', true);
    results = run(matlab.unittest.TestRunner.withTextOutput('Verbosity', 1), suite);
    t = table(results);
    fprintf('\n  %d tests: %d passed, %d failed, %d incomplete, %.1f s\n\n', numel(results), ...
        nnz([results.Passed]), nnz([results.Failed]), nnz([results.Incomplete]), sum(t.Duration));
    if any(~[results.Passed])
        disp(t(~[results.Passed], {'Name', 'Passed', 'Failed', 'Incomplete'}));
        error('setup:testFailures', '%d test(s) failed.', nnz(~[results.Passed]));
    end
end
