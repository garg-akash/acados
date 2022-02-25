function A = dRdphi(x)

R = angleAxis2R(x(1),x(2:end));

for i = 1:4
    A(:,:,i) = diff(R,x(i));
end

end

