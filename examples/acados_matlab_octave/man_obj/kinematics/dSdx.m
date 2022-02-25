function A = dSdx(x)

P = skew(x);

for i = 1:3
    A(:,:,i) = diff(P,x(i));
end

end
