function [value, alignment, faceIndex] = diceTopFace(R, G)
%DICETOPFACE Face value pointing up (+Z of {A}) for orientation ^A_B[R].
%
%   [value, alignment, faceIndex] = diceTopFace(R, G)
%
%   The world components of each outward face normal are ^A n = ^A_B[R] ^B n
%   (L2-4/s9). The top face is the one whose normal has the largest +Z_A
%   component; alignment = that component (1 = perfectly flat).
%   Outcome rule: [EXTENSION] (not defined in the lectures).
arguments
    R (3,3) double
    G struct
end
nz = R(3, :) * G.normals;         % Z_A component of every face normal
[alignment, faceIndex] = max(nz);
value = G.values(faceIndex);
end
