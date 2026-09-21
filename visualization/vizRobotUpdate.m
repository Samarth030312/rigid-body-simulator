function vizRobotUpdate(h, k)
%VIZROBOTUPDATE Show sample k of the 3R result in scene handle h.
res = h.res; p = res.params;
q = res.x(1:3, k);
fk = robotForwardKinematics(q, p);
s = 0.12 * sum(p.l);
for i = 1:3
    set(h.link(i), 'XData', fk.O(1, i:i+1), 'YData', fk.O(2, i:i+1));
    set(h.com(i), 'XData', fk.C(1, i), 'YData', fk.C(2, i));
    ex = fk.R{i}(:, 1) * s; ey = fk.R{i}(:, 2) * s; o = fk.O(:, i);
    set(h.fx(i), 'XData', o(1) + [0 ex(1)], 'YData', o(2) + [0 ex(2)]);
    set(h.fy(i), 'XData', o(1) + [0 ey(1)], 'YData', o(2) + [0 ey(2)]);
end
set(h.joints, 'XData', fk.O(1, :), 'YData', fk.O(2, :));
set(h.path, 'XData', res.diag.tool(1, 1:k), 'YData', res.diag.tool(2, 1:k));
set(h.label, 'String', sprintf(['t = %5.2f s\nq   = [%7.1f %7.1f %7.1f] deg\n' ...
    'tau = [%7.2f %7.2f %7.2f] N m\ntool = [%6.3f %6.3f] m'], res.t(k), rad2deg(q), ...
    res.diag.tau(:, k), fk.tool(1:2)));
end
