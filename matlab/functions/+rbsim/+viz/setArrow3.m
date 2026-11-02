function setArrow3(h, origin, vec)
%SETARROW3  Place an arrow from ORIGIN along VEC (3x1, same units as the axes).
    L = norm(vec);
    if L < eps
        set([h.shaft h.head], 'XData', nan, 'YData', nan, 'ZData', nan); return
    end
    tip = origin + vec;
    u = vec / L;
    t = cross(u, [0; 0; 1]); if norm(t) < 1e-6, t = cross(u, [1; 0; 0]); end
    t = t / norm(t);
    hl = 0.2 * L;
    p1 = tip - hl * u + 0.5 * hl * t; p2 = tip - hl * u - 0.5 * hl * t;
    set(h.shaft, 'XData', [origin(1) tip(1)], 'YData', [origin(2) tip(2)], 'ZData', [origin(3) tip(3)]);
    set(h.head, 'XData', [p1(1) tip(1) p2(1)], 'YData', [p1(2) tip(2) p2(2)], 'ZData', [p1(3) tip(3) p2(3)]);
end
