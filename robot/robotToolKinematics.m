function [v0, a0, vT, aT] = robotToolKinematics(q, qd, qdd, p)
%ROBOTTOOLKINEMATICS Tool-point velocity and acceleration by lecture propagation.
%
%   [v0, a0, vT, aT] = robotToolKinematics(q, qd, qdd, p)
%
%   Outputs: v0, a0 in {0}; vT, aT in {Tool} (3x1, m/s and m/s^2).
%   Uses the R-joint propagation of L7-9/s44-47 (L20-21/s8-9) from {0} to {Tool}
%   with a fixed base (0 w_0 = 0, 0 V_0 = 0, 0 a_0 = 0; L7-9/s49). Gravity is
%   NOT included (pure kinematics).
Z = [0; 0; 1];
w = zeros(3,1); wd = zeros(3,1); V = zeros(3,1); Vd = zeros(3,1);
pPrev = zeros(3,1);
R0 = eye(3);
for i = 1:4
    if i <= 3, Ri = rotZ(q(i)); qdi = qd(i); qddi = qdd(i);
    else,      Ri = eye(3);     qdi = 0;     qddi = 0;      end   % {Tool}
    Rt = Ri.';
    Vnew  = Rt * (V + cross(w, pPrev));                              % s45
    Vdnew = Rt * (Vd + cross(wd, pPrev) + cross(w, cross(w, pPrev))); % s47
    wdnew = Rt * wd + cross(Rt * w, qdi * Z) + qddi * Z;             % s47
    w = Rt * w + qdi * Z;                                            % s44
    V = Vnew; Vd = Vdnew; wd = wdnew;
    R0 = R0 * Ri;
    if i <= 3, pPrev = [p.l(i); 0; 0]; end
end
vT = V; aT = Vd;
v0 = R0 * vT; a0 = R0 * aT;
end
