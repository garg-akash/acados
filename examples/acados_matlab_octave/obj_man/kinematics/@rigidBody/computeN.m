function N = computeN(obj, R)
%COMPUTEC Summary of this function goes here
%   Detailed explanation goes here

    g = [R'*[0;0;9.81];0;0;0];
    N = obj.mass*g;
    obj.Nb = N;
    
end

