function Fb = inverseDynamics(obj, p, dp, ddp)
%INVERSEDYNAMICS Summary of this function goes here
%   Detailed explanation goes here

    t = obj.getBodyTwist();
    C = obj.computeC(t);
    T = obj.getGlobalPose();
    R = T(1:3,1:3);
    g = [R'*[0;0;9.81];0;0;0];
    Fb = + C + obj.mass*g + obj.Mb*ddp;
        
end

