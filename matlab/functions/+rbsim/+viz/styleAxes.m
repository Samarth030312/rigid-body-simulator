function styleAxes(ax, th)
%STYLEAXES Recessive grid/axes styling shared by all dashboard panels.
if nargin < 2, th = rbsim.viz.theme(); end
set(ax, 'Color', th.surface, 'XColor', th.textMuted, 'YColor', th.textMuted, ...
    'GridColor', th.grid, 'GridAlpha', 1, 'FontSize', 9, 'Box', 'off', ...
    'TickDir', 'out', 'LineWidth', 0.75);
grid(ax, 'on');
end
