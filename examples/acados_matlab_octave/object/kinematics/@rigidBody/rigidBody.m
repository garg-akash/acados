classdef rigidBody < matlab.mixin.Copyable
    %OBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mass = 1;           % mass
        I = eye(3);         % body inertia
        contacts = [];      % list of contacts
        Mb = zeros(6,6);    % body mass
        Cb = zeros(6,6);    % body C
        G = [];             % grasp matrix
        Go = [];            % orientation grasp matrix
        g = [];             % pose
        t = [];             % body twist
        a = [];             % body acceleration
        Fb = [];            % body wrench
        Fc_hat = [];        % composite friction cone versor
    end
    
    methods
        function obj = rigidBody(m, Ib)
            %OBJECT Construct an instance of this class
            %   m = mass
            %   I = body inertia (3x3) matrix
            obj.mass = m;
            obj.I = Ib;
            obj.Mb = [eye(3)*m, zeros(3,3);
                zeros(3,3) Ib];
        end
        
        function addContact(obj, c)
            %ADDCONCTACTS Add contacts to an object
            obj.contacts = [obj.contacts, c];
            
            % compute grasp matrix
            for i = 1:size(c,2)
                F_bc = c(i).F_;
                F_cb = [F_bc(1:3,1:3)', -F_bc(1:3,1:3)'*F_bc(1:3,4); ...
                    0 0 0 1];
                obj.G = [obj.G, ad(F_cb)'*c(i).B_];
                
                H = [zeros(3), eye(3)];
                Bo = [zeros(3); eye(2), [0;0]; 0,0,0];
                obj.Go = [obj.Go, H*ad(F_cb)'*Bo];
            end
            
            % recompute composite cone matrix
            obj.Fc_hat = blkdiag(obj.contacts(:).x_hat_);

        end
        
        function setGlobalPose(obj, g)
            obj.g = g;
        end
        
        function g = getGlobalPose(obj)
            g = obj.g;
        end
        
        function setBodyTwist(obj, t)
            obj.t = t;
        end
        
        function t = getBodyTwist(obj)
            t = obj.t;
        end
        
        function setBodyAcceleration(obj, a)
            obj.a = a;
        end
        
        function setContactForces(obj, Fc)
            z = [0,0,1]';
            for i = 1: size(obj.contacts,2)
                Fci = Fc(i*3-2 : i*3);
                obj.contacts(i).Fc_ = Fci;
                nomrFci = norm(Fci);
                alpha = acos(z'*Fci/nomrFci);
                if norm(alpha) == 0
                    nhat = [0,0,0]';
                    obj.contacts(i).alpha_ = alpha;
                    obj.contacts(i).n_hat_ = nhat;
                else
                    n = cross(z,Fci);
                    nhat = n/norm(n);
                    obj.contacts(i).alpha_ = alpha;
                    obj.contacts(i).n_hat_ = nhat;
                end
            end
        end
        
        [Fb_, Fc_, lambda_, flag_] = computeReactionForces(obj, Fb);
        [Fb_, Fc_, lambda_, flag_] = computeOptReactionForces(obj, p, pd, pdd, t);
        [lambda_, flag_] = computeOptLambda(obj, Fb);
        [Lambda, xddot, x] = computeOptTrajForces(obj, t);
        tau = computeOptTau(obj, alpha, alpha_dot);
        [tau, h] = computeTau(obj, alpha)
        alpha = getAlphaVector(obj);
        Fc = getContactForces(obj);
        C = computeC(obj, t);
        Fb = inverseDynamics(obj, p, dp, ddp);
        [pe,dpe,ddpe] = forwardDynamics(obj, Fb, dt);
        h = plotRB(obj, figHandle, T, cones, palm, Fb, Fc);
        [eo, eo_dot] = computeOrientationError(obj, Rd, omegad);
        reset(obj);
    end
end

