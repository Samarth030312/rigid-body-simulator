function h = diceScene(ax, res, follow)
%DICESCENE Build the 3D dice scene for a dice result; update with VIZDICEUPDATE.
%
%   h = rbsim.viz.diceScene(ax, res)
%   h = rbsim.viz.diceScene(ax, res, follow)   follow = true: camera box tracks the die
%
%   Everything drawn is driven by the simulated state: the cube (with pips) is a
%   child of an hgtransform whose matrix is built from ^A r_C and ^A_B[R] of the
%   sample being shown (graphics-only 4x4; the core never uses homogeneous
%   transforms, CONVENTIONS 3). Also drawn: floor, world frame {A}, body frame {B},
%   COM trajectory, velocity V_C and angular velocity w^s arrows.
if nargin < 3, follow = false; end
th = rbsim.viz.theme();
cla(ax, 'reset');
hold(ax, 'on');
G = res.geometry; a = res.params.edge;
P = res.x(1:3, :);
pad = 2 * a;
lo = min(P, [], 2) - pad; hi = max(P, [], 2) + pad;
lo(3) = min(lo(3), res.params.contact.floorZ);
span = max(hi - lo); ctr = (lo + hi) / 2;
lim = [ctr - span/2, ctr + span/2];
lim(3, :) = [max(lim(3,1), res.params.contact.floorZ - a) , lim(3,1) + span];
% floor
if res.params.contact.enabled
    z0 = res.params.contact.floorZ;
    h.floor = patch(ax, lim(1, [1 2 2 1]), lim(2, [1 1 2 2]), z0 * [1 1 1 1], th.floor, ...
        'EdgeColor', th.grid, 'FaceAlpha', 0.8);
end
% world frame {A}
L = 1.5 * a;
line(ax, [0 L], [0 0], [0 0], 'Color', th.axisX, 'LineWidth', 1.5);
line(ax, [0 0], [0 L], [0 0], 'Color', th.axisY, 'LineWidth', 1.5);
line(ax, [0 0], [0 0], [0 L], 'Color', th.axisZ, 'LineWidth', 1.5);
text(ax, L, 0, 0, ' X_A', 'Color', th.textMuted, 'FontSize', 8, 'Clipping', 'on');
text(ax, 0, L, 0, ' Y_A', 'Color', th.textMuted, 'FontSize', 8, 'Clipping', 'on');
text(ax, 0, 0, L, ' Z_A', 'Color', th.textMuted, 'FontSize', 8, 'Clipping', 'on');
% trajectory
h.trail = line(ax, P(1,1), P(2,1), P(3,1), 'Color', th.neutral, 'LineWidth', 1);
% body
h.tf = hgtransform(ax);
patch(h.tf, 'Vertices', G.vertices.', 'Faces', G.faces, 'FaceColor', th.dieBody, ...
    'EdgeColor', th.dieEdge, 'LineWidth', 1, 'FaceLighting', 'gouraud');
[pv, pf] = pipMesh(G);
patch(h.tf, 'Vertices', pv, 'Faces', pf, 'FaceColor', th.pip, 'EdgeColor', 'none');
Lb = 0.9 * a;
line(h.tf, [0 Lb], [0 0], [0 0], 'Color', th.axisX, 'LineWidth', 2);
line(h.tf, [0 0], [0 Lb], [0 0], 'Color', th.axisY, 'LineWidth', 2);
line(h.tf, [0 0], [0 0], [0 Lb], 'Color', th.axisZ, 'LineWidth', 2);
h.vArrow = rbsim.viz.arrow3(ax, th.series(2, :), 1.5);
h.wArrow = rbsim.viz.arrow3(ax, th.series(7, :), 1.5);
h.label = text(ax, 0.02, 0.97, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
    'FontSize', 9, 'Color', th.text, 'FontName', 'monospace');
axis(ax, 'equal');
xlim(ax, lim(1, :)); ylim(ax, lim(2, :)); zlim(ax, lim(3, :));
view(ax, 35, 22);
xlabel(ax, 'X_A (m)'); ylabel(ax, 'Y_A (m)'); zlabel(ax, 'Z_A (m)');
rbsim.viz.styleAxes(ax, th);
light(ax, 'Position', [1 -1 2], 'Style', 'infinite');
h.res = res;
h.ax = ax;
h.follow = follow;
h.fullLim = lim;
h.scale = struct('v', 2 * a / max(1e-9, max(res.diag.speed)), ...
                 'w', 2 * a / max(1e-9, max(res.diag.spin)));
rbsim.viz.diceUpdate(h, 1);
end

function [V, F] = pipMesh(G)
% All pips as one patch of 16-gon discs, lifted slightly off the faces.
nSeg = 16; ang = (0:nSeg-1) * 2*pi / nSeg;
V = []; F = [];
for f = 1:6
    n = G.normals(:, f);
    if abs(n(3)) > 0.5, u = [1;0;0]; else, u = [0;0;1]; end
    u = u - (u.'*n)*n; u = u / norm(u); v = cross(n, u);
    for c = G.pips{f}
        pts = c + 0.002 * max(abs(G.vertices(:))) * n + G.pipRadius * (u * cos(ang) + v * sin(ang));
        F(end+1, :) = size(V, 1) + (1:nSeg); %#ok<AGROW>
        V = [V; pts.']; %#ok<AGROW>
    end
end
end
