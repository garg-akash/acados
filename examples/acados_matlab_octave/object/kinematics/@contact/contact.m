classdef contact < handle
    %CONTACT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        F_ = eye(4)          % contact frame w.r.t. body frame
        B_ = [];             % force selection
        Fc_ = [];            % contact forces
        x_hat_ = [];         % friction cone directions
        theta_ = 0;          % friction cone semi-aperture angle
        z_hat_ = [0,0,1]';   % contact normal
        alpha_ = [];
        n_hat_ = [0,0,0];
    end
    
    methods
        function obj = contact(F, type, mu)
            %CONTACT Construct an instance of this class
            %   F = contact frame
            %   type = 'pc', 'pcwf', ...
            obj.F_ = F;
            switch type
                case 'pc'
                    obj.B_ = [0 0 1 0 0 0]';
                case 'pcwf'
                    obj.B_ = [eye(3) zeros(3,3)]';
                case 'sc'
                    obj.B_ = [eye(4) zeros(3,4)]';    
            end
            
            obj.Fc_ = [0,0,0]';
            obj.theta_ = atan(mu);
            obj.x_hat_ = [rotx(obj.theta_)*obj.z_hat_, ...
                         rotx(-obj.theta_)*obj.z_hat_, ...
                         roty(obj.theta_)*obj.z_hat_, ...
                         roty(-obj.theta_)*obj.z_hat_];            
            
        end
    end
end

