function h = vizArrow3(parent, color, width)
%VIZARROW3 Create an updatable 3D arrow (shaft line + head line); see VIZSETARROW3.
h.shaft = line(parent, nan, nan, nan, 'Color', color, 'LineWidth', width);
h.head = line(parent, nan, nan, nan, 'Color', color, 'LineWidth', width);
end
