classdef RestDetector < handle
    %RESTDETECTOR Stateful "dice has come to rest" test for integrateRK4 StopFcn.
    %
    %   det = RestDetector(p, G, holdTime)
    %   stop = det.check(t, x)
    %
    %   Rest criterion (engineering choice): resting on a face (top-face alignment > 0.999),
    %   at least 3 vertices in contact, |V_C| < 5e-3 m/s and |w| < 0.2 rad/s,
    %   held continuously for holdTime seconds. Identical thresholds are used by
    %   rbsim.dice.simulate's summary.settled so the two never disagree.
    properties (SetAccess = private)
        p
        G
        holdTime
        restSince = NaN
    end
    methods
        function obj = RestDetector(p, G, holdTime)
            obj.p = p; obj.G = G; obj.holdTime = holdTime;
        end
        function stop = check(obj, t, x)
            if rbsim.dice.RestDetector.isAtRest(x, obj.p, obj.G)
                if isnan(obj.restSince), obj.restSince = t; end
                stop = (t - obj.restSince) >= obj.holdTime;
            else
                obj.restSince = NaN;
                stop = false;
            end
        end
    end
    methods (Static)
        function tf = isAtRest(x, p, G)
            tf = norm(x(4:6)) < 5e-3 && norm(x(16:18)) < 0.2;
            if tf
                R = reshape(x(7:15), 3, 3);
                [~, align] = rbsim.dice.topFace(R, G);
                [~, ~, info] = rbsim.dice.contactLoads(x, p, G);
                tf = align > 0.999 && info.n >= 3;
            end
        end
    end
end
