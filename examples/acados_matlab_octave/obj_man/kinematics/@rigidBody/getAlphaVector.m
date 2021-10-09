function a = getAlphaVector(obj)
%COMPUTEALPHAVECTOR Summary of this function goes here
%   Detailed explanation goes here

a = [];

for i = 1 : size(obj.contacts,2)
    a = [a; obj.contacts(i).alpha_];
end

end
