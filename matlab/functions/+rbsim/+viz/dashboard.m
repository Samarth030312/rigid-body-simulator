function cursors = dashboard(axs, res, kind)
%DASHBOARD Plot the diagnostic dashboard for a result into 4 axes.
%
%   cursors = rbsim.viz.dashboard(axs, res, kind)    kind = 'dice' | 'robot' | 'fourbar'
%
%   axs: 1x4 axes handles. Returns 1x4 time-cursor lines (move them with
%   VIZSETCURSOR). One quantity family per panel, one y-axis per panel, legend
%   whenever there are >= 2 series; series colours follow the entity (joint/link i
%   always uses categorical slot i).
th = rbsim.viz.theme();
t = res.t; d = res.diag;
for a = axs, cla(a, 'reset'); hold(a, 'on'); end
switch kind
    case 'dice'
        plotSeries(axs(1), t, res.x(3, :), {'COM height z_C'}, 'z_C (m)', th, 1);
        if res.params.contact.enabled
            yline(axs(1), res.params.edge / 2, ':', 'a/2', 'Color', th.textMuted);
        end
        plotSeries(axs(2), t, [d.kinetic; d.potential + d.spring; d.energy], ...
            {'kinetic', 'potential + spring', 'total'}, 'energy (J)', th, 1:3);
        plotSeries(axs(3), t, d.speed, {'|V_C|'}, 'speed (m/s)', th, 2);
        plotSeries(axs(4), t, d.spin, {'|\omega|'}, 'angular speed (rad/s)', th, 7);
    case 'robot'
        plotSeries(axs(1), t, rad2deg(res.x(1:3, :)), {'\theta_1', '\theta_2', '\theta_3'}, ...
            'joint angle (deg)', th, 1:3);
        plotSeries(axs(2), t, d.tau, {'\tau_1', '\tau_2', '\tau_3'}, 'actuator torque (N m)', th, 1:3);
        plotSeries(axs(3), t, [d.kinetic; d.potential; d.energy], ...
            {'kinetic', 'potential', 'total'}, 'energy (J)', th, 1:3);
        plotSeries(axs(4), t, d.energyBalance, {'E - E_0 - W'}, 'energy balance (J)', th, 8);
    case 'fourbar'
        plotSeries(axs(1), t, rad2deg(unwrap([d.theta2; d.phi1], [], 2)), ...
            {'coupler \theta_2', 'rocker \phi_1'}, 'angle (deg)', th, 2:3);
        plotSeries(axs(2), t, d.tau, {'crank torque \tau'}, 'torque (N m)', th, 1);
        plotSeries(axs(3), t, [vecnorm(d.reactions.OL); vecnorm(d.reactions.A); ...
            vecnorm(d.reactions.B); vecnorm(d.reactions.OR)], {'|F_{OL}|', '|F_A|', '|F_B|', '|F_{OR}|'}, ...
            'joint reaction (N)', th, [1 2 3 4]);
        plotSeries(axs(4), t, max(d.closureResidual, 1e-18), {'loop-closure residual'}, ...
            'residual (m)', th, 8);
        set(axs(4), 'YScale', 'log');
end
cursors = gobjects(1, 4);
for i = 1:4
    xlabel(axs(i), 't (s)');
    xlim(axs(i), [t(1) max(t(end), t(1) + eps)]);
    cursors(i) = xline(axs(i), t(1), 'Color', th.textMuted, 'LineWidth', 1, ...
        'HandleVisibility', 'off');
    rbsim.viz.styleAxes(axs(i), th);
end
end

function plotSeries(ax, t, Y, names, ylab, th, slots)
for i = 1:size(Y, 1)
    plot(ax, t, Y(i, :), 'Color', th.series(slots(i), :), 'LineWidth', th.lineWidth, ...
        'DisplayName', names{i});
end
ylabel(ax, ylab);
if size(Y, 1) >= 2
    legend(ax, 'Location', 'best', 'Box', 'off', 'FontSize', 8, 'TextColor', th.text, ...
        'AutoUpdate', 'off');
else
    title(ax, names{1}, 'FontWeight', 'normal', 'FontSize', 9, 'Color', th.text);
end
end
