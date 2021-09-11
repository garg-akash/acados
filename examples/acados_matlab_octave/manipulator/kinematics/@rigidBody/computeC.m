function C = computeC(obj, t)
%COMPUTEC Summary of this function goes here
%   Detailed explanation goes here

    v = t(1:3);
    omega = t(4:6);
    
    C = [cross(omega, obj.mass*v);
         cross(omega, obj.I*omega)];

end

