function h = arrow3(parent, color, width)
%ARROW3  Create an updatable 3D arrow (shaft line + head line); see rbsim.viz.setArrow3.
    h.shaft = line(parent, nan, nan, nan, 'Color', color, 'LineWidth', width);
    h.head = line(parent, nan, nan, nan, 'Color', color, 'LineWidth', width);
end
