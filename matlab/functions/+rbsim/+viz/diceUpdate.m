function diceUpdate(h, k)
%DICEUPDATE Show sample k of the dice result stored in scene handle h.
res = h.res;
x = res.x(:, k);
[r, v, R, wb] = rbsim.body.unpack(x);
set(h.tf, 'Matrix', [R r; 0 0 0 1]);
if h.follow                                    % camera box of 8 edges around C
    w = 4 * res.params.edge;
    zlo = min(r(3) - w, h.fullLim(3, 1) + w);
    xlim(h.ax, r(1) + [-w w]); ylim(h.ax, r(2) + [-w w]); zlim(h.ax, zlo + [0 2*w]);
end
set(h.trail, 'XData', res.x(1, 1:k), 'YData', res.x(2, 1:k), 'ZData', res.x(3, 1:k));
rbsim.viz.setArrow3(h.vArrow, r, h.scale.v * v);
ws = R * wb;                                   % ^A w^s = R ^A w^b ()
rbsim.viz.setArrow3(h.wArrow, r, h.scale.w * ws);
th = rbsim.math.rotToEulerXYZ(R);
set(h.label, 'String', sprintf(['t = %6.3f s   top face = %d\n' ...
    'r_C = [%7.4f %7.4f %7.4f] m\n|V_C| = %6.3f m/s  |w| = %6.2f rad/s\n' ...
    'XYZ Euler = [%6.1f %6.1f %6.1f] deg'], res.t(k), res.diag.topFace(k), r, ...
    norm(v), norm(wb), rad2deg(th)));
end
