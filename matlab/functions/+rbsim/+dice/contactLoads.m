function [F_A, M_A, info] = contactLoads(x, p, G)
%CONTACTLOADS Floor contact forces on the cube vertices (EXTENSION, ADR-0012).
%
%   [F_A, M_A, info] = rbsim.dice.contactLoads(x, p, G)
%
%   Inputs: x 18x1 rigid-body state; p dice params; G = rbsim.dice.geometry(p.edge).
%   Outputs: F_A 3x1 resultant contact force in {A}, N
%            M_A 3x1 resultant contact moment about the COM C in {A}, N*m
%            info struct:.n (number of vertices in contact),.springEnergy (J),
%.normalForce (1x8, N)
%
%   Model, built from elementary force elements:
%     vertex i:  ^A p_i = ^A r_C + R ^B v_i
%                ^A V_i = ^A V_C + w^s x (R ^B v_i)
%     penetration  delta_i = floorZ - p_iz  (> 0 in contact)
%     normal       N_i = max(0, k delta_i - c V_iz)   spring + damper, no pull
%     friction     F_t = -mu N_i u_t / sqrt(|u_t|^2 + v_eps^2)   Coulomb |F| = mu N
%                  opposite to sliding velocity u_t (smoothly regularised)
%     moment       M_A = sum (R ^B v_i) x f_i
F_A = zeros(3, 1); M_A = zeros(3, 1);
wantInfo = nargout > 2;                % diagnostics only when requested
if wantInfo
    info = struct('n', 0, 'springEnergy', 0, 'normalForce', zeros(1, 8));
end
c = p.contact;
if ~c.enabled
    return
end
% Inner-loop function: no argument validation (rbsim.dice.simulate validates once).
R = reshape(x(7:15), 3, 3);
rv = R * G.vertices;                   % 3x8 COM->vertex vectors in {A}
delta = c.floorZ - (x(3) + rv(3, :));
idx = find(delta > 0);
if isempty(idx)
    return
end
wb = x(16:18);
ws = R * wb;                           % ^A w^s = R ^A w^b  ()
rc = rv(:, idx);
Vc = x(4:6) + [ws(2)*rc(3,:) - ws(3)*rc(2,:);      % V_C + w^s x r  (vectorised)
               ws(3)*rc(1,:) - ws(1)*rc(3,:);
               ws(1)*rc(2,:) - ws(2)*rc(1,:)];
N = max(0, c.stiffness * delta(idx) - c.damping * Vc(3, :));
ut = Vc(1:2, :);
scale = -c.friction * N ./ sqrt(sum(ut.^2, 1) + c.frictionVelocity^2);
f = [ut .* scale; N];                  % 3xn vertex forces in {A}
F_A = sum(f, 2);
M_A = sum([rc(2,:).*f(3,:) - rc(3,:).*f(2,:);
           rc(3,:).*f(1,:) - rc(1,:).*f(3,:);
           rc(1,:).*f(2,:) - rc(2,:).*f(1,:)], 2);
if wantInfo
    info.normalForce(idx) = N;
    info.springEnergy = 0.5 * c.stiffness * sum(delta(idx).^2);
    info.n = numel(idx);
end
end
