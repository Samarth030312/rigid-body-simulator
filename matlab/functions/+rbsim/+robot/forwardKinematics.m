function fk = forwardKinematics(q, p)
%FORWARDKINEMATICS Position and orientation of every frame of the planar 3R.
%
%   fk = rbsim.robot.forwardKinematics(q, p)
%
%   Inputs
%     q : 3x1 joint angles, rad (relative, CCW, L7-9/s49).
%     p : robot parameters (ROBOTDEFAULTPARAMS).
%   Output struct (all in the fixed frame {0}, metres)
%     .R      1x4 cell: ^0_i[R] for i = 1,2,3 and {Tool}
%     .O      3x4 origins O_1, O_2, O_3, O_Tool
%     .C      3x3 centres of mass C_1..C_3
%     .tool   3x1 tool point, .phi tool orientation angle theta1+theta2+theta3 (rad)
%
%   Composition (L5-6/s13, L7-9/s44):
%     ^0_i[R]   = ^0_{i-1}[R] ^{i-1}_i[R],   ^{i-1}_i[R] = R(Z, theta_i)
%     ^0 O_{i+1} = ^0 O_i + ^0_i[R] ^i O_{i+1},  ^i O_{i+1} = [l_i; 0; 0]
%     ^0 C_i     = ^0 O_i + ^0_i[R] [r_i; 0; 0]
%   {Tool} is at the end of link 3, parallel to {3} (L7-9/s49).
R = eye(3);
O = zeros(3, 1);
fk.R = cell(1, 4);
fk.O = zeros(3, 4);
fk.C = zeros(3, 3);
for i = 1:3
    R = R * rbsim.math.rotZ(q(i));
    fk.R{i} = R;
    fk.O(:, i) = O;
    fk.C(:, i) = O + R * [p.r(i); 0; 0];
    O = O + R * [p.l(i); 0; 0];
end
fk.R{4} = R;
fk.O(:, 4) = O;
fk.tool = O;
fk.phi = sum(q);
end
