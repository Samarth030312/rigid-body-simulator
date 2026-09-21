function p = fourbarValidateParams(p)
%FOURBARVALIDATEPARAMS Check four-bar parameters; error id 'fourbar:param'.
if numel(p.l) ~= 4 || any(~isfinite(p.l)) || any(p.l <= 0)
    error('fourbar:param', 'Four positive link lengths [l0 l1 l2 l3] are required.');
end
for f = {'m', 'r', 'Izz'}
    v = p.(f{1});
    if numel(v) ~= 3 || any(~isfinite(v)) || any(v < 0)
        error('fourbar:param', 'Field %s needs three non-negative values.', f{1});
    end
    p.(f{1}) = v(:).';
end
p.l = p.l(:).';
if any(p.r > p.l(2:4))
    error('fourbar:param', 'Each COM distance must lie on its link.');
end
if ~any(p.branch == [1 -1])
    error('fourbar:param', 'Branch must be +1 (open) or -1 (crossed).');
end
if ~any(strcmp(p.drive.mode, {'free', 'torque', 'speed'}))
    error('fourbar:param', 'Drive mode must be free, torque or speed.');
end
[~, ok] = fourbarPosition(deg2rad(p.theta0Deg), p);
if ~ok
    error('fourbar:param', ['The loop cannot be assembled at the initial crank ' ...
        'angle %.1f deg with these link lengths.'], p.theta0Deg);
end
if ~isscalar(p.tFinal) || p.tFinal <= 0 || ~isscalar(p.dt) || p.dt <= 0 || p.dt > p.tFinal
    error('fourbar:param', 'Require 0 < dt <= tFinal.');
end
end
