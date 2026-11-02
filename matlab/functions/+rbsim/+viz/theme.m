function th = theme()
%THEME Shared colours for scenes and dashboards (RGB triplets in [0,1]).
%
%   Categorical order is the validated reference palette of the data-viz method
%   (slot 1 blue, 2 orange, 3 aqua,... never cycled). Frame axes use the red,
%   green and blue slots for X, Y, Z. Text/axes use neutral inks.
hex = @(h) sscanf(h(2:end), '%2x%2x%2x', [1 3]) / 255;
th.series = [hex('#2a78d6'); hex('#eb6834'); hex('#1baf7a'); hex('#eda100'); ...
             hex('#e87ba4'); hex('#008300'); hex('#4a3aa7'); hex('#e34948')];
th.axisX = hex('#e34948'); th.axisY = hex('#008300'); th.axisZ = hex('#2a78d6');
th.surface = hex('#fcfcfb');
th.text = hex('#0b0b0b'); th.textMuted = hex('#52514e');
th.grid = hex('#d9d8d3');
th.neutral = hex('#8a8983');
th.dieBody = hex('#f4f1e8'); th.dieEdge = hex('#3a3936'); th.pip = hex('#1c1b19');
th.floor = hex('#e8e6df');
th.link = th.series(1:3, :);
th.lineWidth = 2;
end
