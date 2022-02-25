function A = dRdzyx(x)

R = zyx2R(x);

for i=1:3
    A(:,:,i) = diff(R,x(i));
end

end