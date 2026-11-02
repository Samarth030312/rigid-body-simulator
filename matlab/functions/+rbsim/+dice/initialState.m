function x0 = initialState(p)
%INITIALSTATE 18x1 initial rigid-body state from user-facing dice parameters.
%
%   x0 = rbsim.dice.initialState(p)
%
%   Orientation: body-fixed (post-multiplied) or space-fixed (pre-multiplied)
%   rotations of the given sequence, angles in degrees.
%   Angular velocity: if given in 'space' components (^A w^s) it is converted to
%   the body-fixed state variable with ^A w^b = R' ^A w^s.
p = rbsim.dice.validate(p);
o = p.orientation;
ang = deg2rad(o.anglesDeg(:).');
if strcmp(o.type, 'body')
    R0 = rbsim.math.rotBodyFixed(o.sequence, ang);
else
    R0 = rbsim.math.rotSpaceFixed(o.sequence, ang);
end
if strcmp(p.omegaFrame, 'space')
    wb = R0.' * p.omega0;
else
    wb = p.omega0;
end
x0 = rbsim.body.pack(p.r0, p.v0, R0, wb);
end
