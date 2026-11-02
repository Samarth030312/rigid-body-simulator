function h = fourbarScene(ax, res)
%FOURBARSCENE Four-bar scene (ground, links, joints, coupler-point curve).
th = rbsim.viz.theme();
cla(ax, 'reset'); hold(ax, 'on');
p = res.params; l0 = p.l(1);
A = res.diag.points.A; B = res.diag.points.B;
allX = [0, l0, A(1,:), B(1,:)]; allY = [0, 0, A(2,:), B(2,:)];
m = 0.15 * max(p.l);
% ground link with hatching
line(ax, [0 l0], [0 0], 'Color', th.neutral, 'LineWidth', 2, 'LineStyle', ':');
for x0 = [0 l0]
    for j = -2:2
        line(ax, x0 + 0.01*j + [0 -0.01], [-0.012 -0.025], 'Color', th.neutral);
    end
end
C2 = squeeze(res.diag.points.C(:, 2, :));
line(ax, C2(1, :), C2(2, :), 'Color', th.neutral, 'LineWidth', 1, 'LineStyle', '--');
h.curve = line(ax, nan, nan, 'Color', th.series(2, :), 'LineWidth', 1.2);
names = {'crank l_1', 'coupler l_2', 'rocker l_3'};
for i = 1:3
    h.link(i) = line(ax, nan, nan, 'Color', th.link(i, :), 'LineWidth', 6, 'DisplayName', names{i});
end
h.joints = line(ax, nan, nan, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', th.dieEdge, 'LineWidth', 1.5);
h.com = line(ax, nan, nan, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 5, ...
    'MarkerFaceColor', th.text, 'MarkerEdgeColor', 'none');
text(ax, 0, -0.04, 'O_L', 'HorizontalAlignment', 'center', 'Color', th.textMuted);
text(ax, l0, -0.04, 'O_R', 'HorizontalAlignment', 'center', 'Color', th.textMuted);
h.label = text(ax, 0.02, 0.98, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
    'FontSize', 9, 'Color', th.text, 'FontName', 'monospace');
axis(ax, 'equal');
xlim(ax, [min(allX) - m, max(allX) + m]); ylim(ax, [min([allY -0.05]) - m, max(allY) + m]);
xlabel(ax, 'X_0 (m)'); ylabel(ax, 'Y_0 (m)');
rbsim.viz.styleAxes(ax, th);
title(ax, sprintf('Four-bar (%s, %s branch)', res.summary.type, ...
    branchName(p.branch)), 'FontWeight', 'normal', 'Color', th.text, 'FontSize', 10);
h.res = res;
rbsim.viz.fourbarUpdate(h, 1);
end

function s = branchName(b)
if b > 0, s = 'open'; else, s = 'crossed'; end
end
