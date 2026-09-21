function G = diceGeometry(a)
%DICEGEOMETRY Cube geometry and dice face layout in the body frame {B}.
%
%   G = diceGeometry(a)
%
%   Input : a  edge length, m. {B} origin at the centre of mass (CONVENTIONS 1).
%   Output struct:
%     .vertices   3x8 vertex coordinates in {B}, m (+-a/2 per axis)
%     .faces      6x4 vertex indices per face, counter-clockwise seen from outside
%     .normals    3x6 outward unit normals in {B}
%     .values     1x6 face value shown on each face
%     .pips       1x6 cell, each 3xN pip centres in {B} (N = face value), m
%     .pipRadius  pip radius, m
%
%   Face layout [LECTURE] L2-4/s13, s44: +X 4, -X 3, +Y 2, -Y 5, +Z 1, -Z 6.
%   Pip patterns (standard die) are an [EXTENSION] for display.
arguments
    a (1,1) double {mustBePositive}
end
h = a / 2;
[X, Y, Z] = ndgrid([-h h], [-h h], [-h h]);
V = [X(:) Y(:) Z(:)].';               % index = 1 + (x>0) + 2(y>0) + 4(z>0)
% faces: +X, -X, +Y, -Y, +Z, -Z  (CCW about the outward normal)
F = [2 4 8 6;    % +X
     1 5 7 3;    % -X
     3 7 8 4;    % +Y
     1 2 6 5;    % -Y
     5 6 8 7;    % +Z
     1 3 4 2];   % -Z
N = [1 -1 0 0 0 0; 0 0 1 -1 0 0; 0 0 0 0 1 -1];
G.vertices = V;
G.faces = F;
G.normals = N;
G.values = [4 3 2 5 1 6];
G.pipRadius = 0.09 * a;
s = 0.27 * a;                          % pip offset from the face centre
patterns = { [0 0], ...
             [-1 -1; 1 1], ...
             [-1 -1; 0 0; 1 1], ...
             [-1 -1; -1 1; 1 -1; 1 1], ...
             [-1 -1; -1 1; 0 0; 1 -1; 1 1], ...
             [-1 -1; -1 0; -1 1; 1 -1; 1 0; 1 1] };
G.pips = cell(1, 6);
for f = 1:6
    n = N(:, f);
    [u, v] = faceAxes(n);
    uv = patterns{G.values(f)} * s;
    G.pips{f} = n * h + u * uv(:, 1).' + v * uv(:, 2).';
end
end

function [u, v] = faceAxes(n)
% In-plane orthonormal axes (u, v, n right-handed).
if abs(n(3)) > 0.5
    u = [1; 0; 0];
else
    u = [0; 0; 1];
end
u = u - (u.' * n) * n; u = u / norm(u);
v = cross(n, u);
end
