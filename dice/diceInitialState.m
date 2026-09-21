function x0 = diceInitialState(p)
%DICEINITIALSTATE 18x1 initial rigid-body state from user-facing dice parameters.
%
%   x0 = diceInitialState(p)
%
%   Orientation: body-fixed (post-multiplied) or space-fixed (pre-multiplied)
%   rotations of the given sequence (L2-4/s21-36), angles in degrees.
%   Angular velocity: if given in 'space' components (^A w^s) it is converted to
%   the body-fixed state variable with ^A w^b = R' ^A w^s (L7-9/s10).
p = diceValidateParams(p);
o = p.orientation;
ang = deg2rad(o.anglesDeg(:).');
if strcmp(o.type, 'body')
    R0 = rotBodyFixed(o.sequence, ang);
else
    R0 = rotSpaceFixed(o.sequence, ang);
end
if strcmp(p.omegaFrame, 'space')
    wb = R0.' * p.omega0;
else
    wb = p.omega0;
end
x0 = rbPack(p.r0, p.v0, R0, wb);
end
