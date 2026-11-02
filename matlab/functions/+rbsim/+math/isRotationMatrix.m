function tf = isRotationMatrix(R, tol)
%ISROTATIONMATRIX  True if R is a proper rotation matrix within tolerance.
%
%   tf = isRotationMatrix(R)       % tol = 1e-9
%   tf = isRotationMatrix(R, tol)
%
%   Checks ||R'R - I||_F <= tol and |det R - 1| <= tol.
    arguments
        R double
        tol (1,1) double {mustBePositive} = 1e-9
    end
    tf = isequal(size(R), [3 3]) && all(isfinite(R(:)));
    if tf
        [o, d] = rbsim.math.rotationError(R);
        tf = (o <= tol) && (d <= tol);
    end
end
