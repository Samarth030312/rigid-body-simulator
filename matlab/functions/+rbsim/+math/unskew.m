function w = unskew(S)
%UNSKEW  Extract the 3-vector from a (near) skew-symmetric matrix.
%
%   w = unskew(S)
%
%   Input
%     S: 3x3 matrix. Only its rbsim.math.skew-symmetric part (S - S')/2 is used, so small
%         numerical symmetric errors (e.g. from Rdot*R' computed numerically) are
%         rejected rather than propagated.
%   Output
%     w: 3x1 vector such that rbsim.math.skew(w) = (S - S')/2.
%
%   Source: inverse of the layout in. Symmetric-part rejection
%   is an engineering choice.
%   See also SKEW.
    arguments
        S (3,3) double {mustBeFinite}
    end
    A = 0.5 * (S - S.');
    w = [A(3,2); A(1,3); A(2,1)];
end
