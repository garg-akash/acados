function A = dEdphi(x)

P = E(x);

for i = 1:4
    A(:,:,i) = diff(P,x(i));
end

end
