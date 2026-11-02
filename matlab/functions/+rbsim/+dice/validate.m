function p = validate(p)
%VALIDATE Check dice parameters; error with a user-readable message.
%
%   p = rbsim.dice.validate(p)   returns p with normalised fields (column vectors,
%   upper-case sequence). Error identifiers start with 'rbsim:dice:param'.
mustPos(p.edge, 'Edge length');
mustPos(p.mass, 'Mass');
p.g = vec3(p.g, 'Gravity');
p.r0 = vec3(p.r0, 'Initial position');
p.v0 = vec3(p.v0, 'Initial velocity');
p.omega0 = vec3(p.omega0, 'Initial angular velocity');
if ~any(strcmp(p.omegaFrame, {'body', 'space'}))
    error('rbsim:dice:param', 'Angular velocity frame must be ''body'' or ''space''.');
end
o = p.orientation;
if ~any(strcmp(o.type, {'body', 'space'}))
    error('rbsim:dice:param', 'Orientation type must be ''body'' or ''space''.');
end
o.sequence = upper(char(o.sequence));
if numel(o.sequence) ~= 3 || any(~ismember(o.sequence, 'XYZ')) || ...
        any(o.sequence(1:2) == o.sequence(2:3))
    error('rbsim:dice:param', ['Rotation sequence must be three axes from X, Y, Z ' ...
        'with no two consecutive axes equal (e.g. XYZ, ZYZ).']);
end
if numel(o.anglesDeg) ~= 3 || any(~isfinite(o.anglesDeg))
    error('rbsim:dice:param', 'Three finite orientation angles are required.');
end
p.orientation = o;
c = p.contact;
if c.enabled
    mustPos(c.stiffness, 'Contact stiffness');
    mustNonneg(c.damping, 'Contact damping');
    mustNonneg(c.friction, 'Friction coefficient');
    mustPos(c.frictionVelocity, 'Friction regularisation velocity');
    if p.r0(3) - c.floorZ < p.edge * sqrt(3) / 2
        error('rbsim:dice:param', ['Initial height must keep the whole cube above the ' ...
            'floor (COM height >= %.4g m).'], c.floorZ + p.edge*sqrt(3)/2);
    end
end
mustPos(p.tFinal, 'Simulation time');
if ~isfield(p, 'stopWhenSettled'), p.stopWhenSettled = false; end
mustPos(p.dt, 'Time step');
if p.dt > p.tFinal
    error('rbsim:dice:param', 'Time step must not exceed the simulation time.');
end
if c.enabled
    % Explicit RK4 stability: keep w_n dt well below 2.8 for the stiffest
    % vertex mode (translation) of a vertex spring carrying the whole mass.
    wn = sqrt(c.stiffness / p.mass);
    if wn * p.dt > 0.5
        error('rbsim:dice:param', ['Time step too large for the contact stiffness ' ...
            '(w_n*dt = %.2f > 0.5). Use dt <= %.2g s.'], wn*p.dt, 0.5/wn);
    end
end
end

function mustPos(v, name)
if ~isscalar(v) || ~isfinite(v) || v <= 0
    error('rbsim:dice:param', '%s must be a positive finite number.', name);
end
end

function mustNonneg(v, name)
if ~isscalar(v) || ~isfinite(v) || v < 0
    error('rbsim:dice:param', '%s must be a non-negative finite number.', name);
end
end

function v = vec3(v, name)
if numel(v) ~= 3 || any(~isfinite(v(:)))
    error('rbsim:dice:param', '%s must have three finite components.', name);
end
v = v(:);
end
