function p = robotValidateParams(p)
%ROBOTVALIDATEPARAMS Check 3R parameters; error id 'robot:param' with a readable message.
%
%   p = robotValidateParams(p)   returns p with row-vector fields.
names = {'l', 'Link lengths'; 'm', 'Link masses'; 'r', 'COM distances'; ...
         'Izz', 'Link inertias'; 'damping', 'Joint damping'; 'q0', 'Initial angles'; ...
         'qd0', 'Initial rates'};
for i = 1:size(names, 1)
    v = p.(names{i, 1});
    if numel(v) ~= 3 || any(~isfinite(v(:)))
        error('robot:param', '%s must have three finite values.', names{i, 2});
    end
    p.(names{i, 1}) = v(:).';
end
if any(p.l <= 0), error('robot:param', 'Link lengths must be positive.'); end
if any(p.m < 0), error('robot:param', 'Link masses must be non-negative.'); end
if any(p.Izz < 0), error('robot:param', 'Link inertias must be non-negative.'); end
if any(p.r < 0 | p.r > p.l)
    error('robot:param', 'Each COM distance r_i must lie on its link (0 <= r_i <= l_i).');
end
if any(p.damping < 0), error('robot:param', 'Joint damping must be non-negative.'); end
if ~isscalar(p.g) || ~isfinite(p.g) || p.g < 0
    error('robot:param', 'Gravity must be a non-negative number.');
end
if p.m(3) + p.Izz(3) == 0 && p.m(2) + p.Izz(2) == 0 && p.m(1) + p.Izz(1) == 0
    error('robot:param', 'At least one link must have mass or inertia.');
end
modes = {'free', 'constant', 'pd', 'trajectory'};
if ~any(strcmp(p.control.mode, modes))
    error('robot:param', 'Control mode must be one of: %s.', strjoin(modes, ', '));
end
if ~isscalar(p.tFinal) || p.tFinal <= 0 || ~isscalar(p.dt) || p.dt <= 0 || p.dt > p.tFinal
    error('robot:param', 'Require 0 < dt <= tFinal.');
end
end
