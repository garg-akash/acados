function A = dEdzyx(x)

P = zyxE(x);

for i = 1:3
    A(:,:,i) = diff(P,x(i));
end

end