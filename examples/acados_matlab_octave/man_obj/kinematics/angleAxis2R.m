function R = angleAxis2R(theta, axis)

for i = 1:size(theta,2)
    c_t = cos(theta(i));
    s_t = sin(theta(i));
    n = axis(:,i);

    R(1:3,1:3,i) = [n(1)^2*(1-c_t)+c_t,        n(1)*n(2)*(1-c_t)-n(3)*s_t, n(1)*n(3)*(1-c_t)+n(2)*s_t;
        n(1)*n(2)*(1-c_t)+n(3)*s_t, n(2)^2*(1-c_t)+c_t,         n(2)*n(3)*(1-c_t)-n(1)*s_t;
        n(1)*n(3)*(1-c_t)-n(2)*s_t, n(2)*n(3)*(1-c_t)+n(1)*s_t, n(3)^2*(1-c_t)+c_t];
end
end