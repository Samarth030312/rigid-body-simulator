function Rn = orthonormalize(R)
%ORTHONORMALIZE  Nearest proper rotation matrix (Frobenius sense) to R.
%
%   Rn = orthonormalize(R)
%
%   Numerical integration of Rdot = R*[omega_b] slowly violates R'R = I. This
%   projection restores the properties  after each step.
%   Method: SVD R = U S V', Rn = U diag(1,1,det(U V')) V'.
%
%   The standard polar (SVD) projection onto SO(3).
    arguments
        R (3,3) double {mustBeFinite}
    end
    [U, ~, V] = svd(R);
    D = diag([1, 1, sign(det(U * V.'))]);
    Rn = U * D * V.';
end
