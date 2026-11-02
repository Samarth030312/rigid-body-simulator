function [isGrashof, type] = grashof(l)
%GRASHOF Grashof condition and mechanism type for link lengths [l0 l1 l2 l3].
%
%   [isGrashof, type] = rbsim.fourbar.grashof(l)
%
%   Grashof: s + L <= p + q (s shortest, L longest). If Grashof, the type follows
%   from which link is shortest: ground -> 'double-crank', crank (l1) or rocker
%   (l3) -> 'crank-rocker' (the shortest link fully rotates), coupler ->
%   'double-rocker'. Otherwise 'triple-rocker (non-Grashof)'.
%   Source: [EXTENSION] (not in the supplied lectures).
arguments
    l (1,4) double {mustBePositive}
end
s = sort(l);
isGrashof = s(1) + s(4) <= s(2) + s(3) + 1e-12;
[~, iShort] = min(l);
if ~isGrashof
    type = 'triple-rocker (non-Grashof)';
elseif abs(s(1) + s(4) - s(2) - s(3)) < 1e-12
    type = 'change-point (Grashof equality)';
else
    names = {'double-crank', 'crank-rocker', 'double-rocker', 'rocker-crank'};
    type = names{iShort};
end
end
