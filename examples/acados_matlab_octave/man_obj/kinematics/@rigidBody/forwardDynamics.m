function [pe,dpe,ddpe] = forwardDynamics(obj, Fb, dt)
%FORWARDDYNAMICS Summary of this function goes here
%   Detailed explanation goes here

    dp = obj.getBodyTwist();
    obj.Cb = computeC(obj, dp);
    T = obj.getGlobalPose();
    R = T(1:3,1:3);
    %g = [R'*[0;0;0];0;0;0];
    g = [R'*[0; 0; 9.81]; 0; 0; 0];
    
    ddpe = obj.Mb\(-obj.Cb - obj.mass*g + Fb);
    dpe = obj.t + ddpe*dt;
    v = dpe(1:3);
    w = dpe(4:6);
    
    normw = norm(w);
    
    if (norm(w) == 0)
        e = [eye(3) v*dt; 0 0 0 1];
    else
        theta = normw*dt;
        w = w/normw;
        v = v/normw;        
        
        Ro = eye(3) + skew(w)*sin(theta) + ...
            (skew(w)*skew(w))*(1-cos(theta));
        

        A = eye(3)*theta + skew(w)*(1-cos(theta)) + ...
            (skew(w)*skew(w))*(theta-sin(theta));
        %A = (eye(3)-Ro)*(cross(w,v))+w*w'*theta
        e = [Ro A*v; 0 0 0 1];
    end 
    
    pe = T*e;
    obj.setBodyAcceleration(ddpe);
    obj.setBodyTwist(dpe);
    obj.setGlobalPose(pe);

end

