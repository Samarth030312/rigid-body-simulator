function vizFourbarUpdate(h, k)
%VIZFOURBARUPDATE Show sample k of the four-bar result in scene handle h.
res = h.res; d = res.diag; p = res.params;
A = d.points.A(:, k); B = d.points.B(:, k); OR = [p.l(1); 0];
P = [[0;0], A, B, OR];
for i = 1:3
    set(h.link(i), 'XData', P(1, i:i+1), 'YData', P(2, i:i+1));
end
set(h.joints, 'XData', P(1, :), 'YData', P(2, :));
C = d.points.C(:, :, k);
set(h.com, 'XData', C(1, :), 'YData', C(2, :));
C2 = squeeze(d.points.C(:, 2, 1:k));
set(h.curve, 'XData', C2(1, :), 'YData', C2(2, :));
set(h.label, 'String', sprintf(['t = %5.2f s\ntheta1 = %7.1f deg  phi1 = %7.1f deg\n' ...
    'tau = %7.3f N m   mu = %5.1f deg\nclosure residual = %.1e m'], res.t(k), ...
    mod(rad2deg(res.x(1, k)), 360), mod(rad2deg(d.phi1(k)), 360), d.tau(k), d.transmissionDeg(k), ...
    d.closureResidual(k)));
end
