function results = runAllTests(folder)
%RUNALLTESTS Run the project MATLAB test suite and print a summary.
%
%   results = runAllTests()            % every test under tests/
%   results = runAllTests('core')      % only tests/core
%
%   Uses matlab.unittest (base MATLAB, no toolbox). Returns the TestResult array.
%   Errors (non-zero exit under -batch) if any test fails, so shell scripts and CI
%   can gate on it.
arguments
    folder {mustBeTextScalar} = ''
end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'config'));
addProjectPaths();
target = fullfile(here, char(folder));
suite = matlab.unittest.TestSuite.fromFolder(target, 'IncludingSubfolders', true);
runner = matlab.unittest.TestRunner.withTextOutput('Verbosity', 1);
results = runner.run(suite);
t = table(results);
fprintf('\n=== TEST SUMMARY ===\n');
fprintf('Total: %d  Passed: %d  Failed: %d  Incomplete: %d  Time: %.2f s\n', ...
    numel(results), nnz([results.Passed]), nnz([results.Failed]), ...
    nnz([results.Incomplete]), sum(t.Duration));
if any([results.Failed]) || any([results.Incomplete])
    disp(t(~[results.Passed], {'Name', 'Passed', 'Failed', 'Incomplete'}));
    error('runAllTests:failures', 'Test failures detected.');
end
end
