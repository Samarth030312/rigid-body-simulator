function app = rigid_body_lab()
%RIGID_BODY_LAB  Throw a die, swing a 3R arm, turn a four-bar: simulate, watch, measure.
%
%   setup open          (from the repository root; or: run('matlab/setup_path.m'); rigid_body_lab)
%
%   TABS       Die - a cube in free flight and on the floor, from its mass, inertia and
%              initial state; 3R arm - a planar three-link manipulator, free or driven;
%              Four-bar - a crank-rocker and its relatives, driven by torque or speed.
%   CONTROLS   Every parameter of the model is an input box or a dropdown. Run simulates,
%              Play and the time slider replay the result, Reset restores the defaults.
%   OUTPUT     A scene drawn from the simulated state, four diagnostic plots sharing a
%              time cursor, and a table of the run's measured outcomes.
%   CHECKS     The Validation tab runs the repository's whole test suite in the app.
%
%   The app computes no mechanics of its own: it reads the forms, calls
%   rbsim.dice.simulate, rbsim.robot.simulate or rbsim.fourbar.simulate, and draws
%   what comes back (rbsim.viz).
        app = RigidBodyLabApp();
end
