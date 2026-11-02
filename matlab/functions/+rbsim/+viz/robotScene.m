function h = robotScene(ax, res)
%ROBOTSCENE  Planar 3R scene (links, joints, frames {i}, tool path); see VIZROBOTUPDATE.
    th = rbsim.viz.theme();
    cla(ax, 'reset'); hold(ax, 'on');
    p = res.params; L = sum(p.l);
    line(ax, [-0.08 0.08] * L, [0 0], 'Color', th.dieEdge, 'LineWidth', 3);   % base
    h.path = line(ax, nan, nan, 'Color', th.neutral, 'LineWidth', 1);
    if any(strcmp(p.control.mode, {'pd', 'trajectory'}))
        fk = rbsim.robot.forwardKinematics(deg2rad(p.control.qTargetDeg(:)), p);
        h.target = line(ax, fk.O(1, :), fk.O(2, :), 'Color', th.neutral, 'LineStyle', '--', ...
            'LineWidth', 1.5, 'DisplayName', 'target');
    end
    for i = 1:3
        h.link(i) = line(ax, nan, nan, 'Color', th.link(i, :), 'LineWidth', 6, ...
            'DisplayName', sprintf('link %d', i));
        h.com(i) = line(ax, nan, nan, 'Marker', 'o', 'MarkerSize', 5, ...
            'MarkerFaceColor', th.text, 'MarkerEdgeColor', 'none', 'LineStyle', 'none');
        h.fx(i) = line(ax, nan, nan, 'Color', th.axisX, 'LineWidth', 1.2);
        h.fy(i) = line(ax, nan, nan, 'Color', th.axisY, 'LineWidth', 1.2);
    end
    h.joints = line(ax, nan, nan, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 9, ...
        'MarkerFaceColor', 'w', 'MarkerEdgeColor', th.dieEdge, 'LineWidth', 1.5);
    h.label = text(ax, 0.02, 0.98, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontSize', 9, 'Color', th.text, 'FontName', 'monospace');
    axis(ax, 'equal');
    xlim(ax, [-1.1 1.1] * L); ylim(ax, [-1.1 1.1] * L);
    xlabel(ax, 'X_0 (m)'); ylabel(ax, 'Y_0 (m)');
    rbsim.viz.styleAxes(ax, th);
    title(ax, 'Planar 3R — frames \{i\} at joints, gravity -Y_0', 'FontWeight', 'normal', ...
        'Color', th.text, 'FontSize', 10);
    h.res = res;
    rbsim.viz.robotUpdate(h, 1);
end
