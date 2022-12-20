classdef trajectory
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        k; % b-spline order
        t; % knot sequence
        c; % control points
        p; % trajectory points 
        dp; % trajectory derivative
    end
    
    methods
        function obj = trajectory(k_, t_, c_,n)
            %TRAJECTORY Construct an instance of this class
            %   Detailed explanation goes here
            obj.k = k_;
            obj.t = t_;
            obj.c = c_;
            
            % plan trajectory
            obj.p = bspline_deboor(k_, t_, c_, n);
            [dt,dc] = bspline_deriv(k_, t_, c_);
            obj.dp = bspline_deboor(k_-1, dt, dc);
        end
        
    end
end

