function p = defaults()
%DEFAULTS Default parameters for the dice simulation (SI units).
%
%   p = rbsim.dice.defaults
%
%   Fields (all SI; angles in the orientation spec are DEGREES because they are a
%   user-facing input, converted once in DICEINITIALSTATE):
%.edge          cube edge length a, m
%.mass          mass m, kg
%.g             gravitational acceleration in {A}, m/s^2 (CONVENTIONS 1)
%.r0            initial COM position ^A r_C, m
%.orientation   struct:.type 'body' | 'space',
%.sequence e.g. 'XYZ' or 'ZYZ',.anglesDeg 1x3
%.v0            initial COM velocity ^A V_C, m/s
%.omega0        initial angular velocity, rad/s
%.omegaFrame    'body' (components along {B}, ^A w^b) or 'space' (along {A})
%.contact       struct (EXTENSION, ADR-0012):
%.enabled,.floorZ (m),.stiffness k (N/m per vertex),
%.damping c (N s/m per vertex),.friction mu (-),
%.frictionVelocity v_eps (m/s, Coulomb regularisation)
%.tFinal        simulated duration, s
%.dt            RK4 step, s
%.outputEvery   store every k-th step
%.stopWhenSettled  stop once at rest for 0.1 s (contact only)
%
%   Default dice: 16 mm cube of density ~1150 kg/m^3 (typical acrylic) -> 4.7 g.
p.edge = 0.016;
p.mass = 1150 * p.edge^3;
p.g = [0; 0; -9.81];
p.r0 = [0; 0; 0.15];
p.orientation = struct('type', 'body', 'sequence', 'XYZ', 'anglesDeg', [20 -35 10]);
p.v0 = [0.4; 0.1; 0.5];
p.omega0 = [18; -25; 12];
p.omegaFrame = 'body';
p.contact = struct('enabled', true, 'floorZ', 0, 'stiffness', 2e4, ...
    'damping', 4, 'friction', 0.35, 'frictionVelocity', 5e-3);
p.tFinal = 3;
p.dt = 1e-4;
p.outputEvery = 20;
p.stopWhenSettled = true;
end
