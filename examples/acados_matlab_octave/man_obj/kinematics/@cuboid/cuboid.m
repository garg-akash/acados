classdef cuboid < rigidBody
    %OBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = cuboid(l, w, h, m, I, mu)
            %OBJECT Construct an instance of this class
            %   l = lenght
            %   w = width
            %   h = height
            %   m = mass
            %   I = body inertia (3x3) matrix
            %   mu = friction
            
            
            h1 = trvec2tform([-l/2, w/2, -h/2]); % position of {C_i} in {B}
            h2 = trvec2tform([ l/2, w/2, -h/2]);
            h3 = trvec2tform([ l/2, -w/2, -h/2]);
            h4 = trvec2tform([-l/2, -w/2, -h/2]);
            c4 = contact(h4, 'pcwf', mu);
            c3 = contact(h3, 'pcwf', mu);
            c2 = contact(h2, 'pcwf', mu);
            c1 = contact(h1, 'pcwf', mu);
            
            % create rigid body
            
            obj = obj@rigidBody(m, I);
            obj.setMu(mu)
            obj.addContact([c1,c2,c3,c4]);
            clear h1 h2 h3 h4 c1 c2 c3 c4;
            obj.reset()
            
        end
    end
end

