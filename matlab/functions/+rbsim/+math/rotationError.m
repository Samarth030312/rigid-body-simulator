function [orthErr, detErr] = rotationError(R)
%ROTATIONERROR  Deviation of a 3x3 matrix from the rotation group SO(3).
%
%   [orthErr, detErr] = rotationError(R)
%
%   Input
%     R: 3x3 matrix.
%   Outputs
%     orthErr: Frobenius norm of (R'*R - I). Zero for an orthonormal matrix.
%     detErr: |det(R) - 1|. Zero for a proper rotation.
%
%   Measures the two defining properties of SO(3), R'R = I and det R = +1.
%   Using the Frobenius norm for the first is an engineering choice.
    arguments
        R (3,3) double
    end
    orthErr = norm(R.' * R - eye(3), 'fro');
    detErr = abs(det(R) - 1);
end
