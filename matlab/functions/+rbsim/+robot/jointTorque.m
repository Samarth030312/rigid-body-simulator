function tau = jointTorque(t, x, p, ref)
%JOINTTORQUE Actuator torques for the selected control mode, N m.
%
%   tau = rbsim.robot.jointTorque(t, x, p, ref)
%
%   Modes (p.control.mode):
%     'free'       tau = 0 (passive triple pendulum under gravity)
%     'constant'   tau = p.control.tau
%     'pd'         tau = Kp (q* - q) - Kd qd [+ G(q) if gravityComp]  (engineering choice)
%     'trajectory' computed torque: tau = M(q)(qdd_d + Kd e_d + Kp e) + C + G
%                  tracking the smooth reference in REF (engineering choice)
%   Joint viscous damping -b_i qd_i is applied separately in ROBOTDYNAMICS.
q = x(1:3); qd = x(4:6);
c = p.control;
switch c.mode
    case 'free'
        tau = zeros(3, 1);
    case 'constant'
        tau = c.tau(:);
    case 'pd'
        qt = deg2rad(c.qTargetDeg(:));
        tau = c.Kp(:) .* (qt - q) - c.Kd(:) .* qd;
        if c.gravityComp
            tau = tau + rbsim.robot.newtonEuler(q, zeros(3,1), zeros(3,1), p);
        end
    case 'trajectory'
        [qr, qdr, qddr] = ref(t);
        v = qddr + c.Kd(:) .* (qdr - qd) + c.Kp(:) .* (qr - q);
        tau = rbsim.robot.newtonEuler(q, qd, v, p);   % = M v + C + G in one RNE call
end
end
