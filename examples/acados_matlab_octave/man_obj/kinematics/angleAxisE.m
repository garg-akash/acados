function A = angleAxisE(x)

t = x(1);
n = x(2:end);

A = [n'; -0.5*sin(t)/(1-cos(t))*skew(n).^2 - 0.5*skew(n)];

%%%%%% check skew(n).^2 or skew(x)*skew(n)

end

