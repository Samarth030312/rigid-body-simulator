function I_O = inertiaParallelAxis(I_C, m, d)
%INERTIAPARALLELAXIS Inertia about a point O from the inertia about the COM C.
%
%   I_O = inertiaParallelAxis(I_C, m, d)
%
%   [I]_O = [I]_C - m [d]^2 ,  [d] = skew(d)
%         = [I]_C + m ( (d'd) [U] - d d' )
%
%   Inputs
%     I_C : 3x3 inertia about C, kg*m^2 (frame {F}).
%     m   : mass, kg.
%     d   : 3x1 vector from C to O (either sign gives the same result), in {F}, m.
%   Output
%     I_O : 3x3 inertia about O, axes parallel to {F}.
%
%   Source: [LECTURE] L14-15/s8 (the printed (3,3) entry is corrected, SD-05).
arguments
    I_C (3,3) double {mustBeFinite}
    m (1,1) double {mustBeNonnegative, mustBeFinite}
    d (3,1) double {mustBeFinite}
end
D = skew(d);
I_O = I_C - m * (D * D);
end
