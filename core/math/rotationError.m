function [orthErr, detErr] = rotationError(R)
%ROTATIONERROR Deviation of a 3x3 matrix from the rotation group SO(3).
%
%   [orthErr, detErr] = rotationError(R)
%
%   Input
%     R       : 3x3 matrix.
%   Outputs
%     orthErr : Frobenius norm of (R'*R - I). Zero for an orthonormal matrix.
%     detErr  : |det(R) - 1|. Zero for a proper rotation.
%
%   Source: properties det = +1 and R'R = [U], [LECTURE] L2-4/s9. Metric choice
%   (Frobenius norm) is an [EXTENSION].
arguments
    R (3,3) double
end
orthErr = norm(R.' * R - eye(3), 'fro');
detErr = abs(det(R) - 1);
end
