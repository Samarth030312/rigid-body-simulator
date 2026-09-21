function vizSetCursor(cursors, t)
%VIZSETCURSOR Move dashboard time cursors to time t (s).
for c = cursors
    if isgraphics(c), c.Value = t; end
end
end
