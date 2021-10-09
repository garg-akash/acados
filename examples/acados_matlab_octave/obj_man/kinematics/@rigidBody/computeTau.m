function [tau_, h] = computeTau(obj, alpha)
%computeOptTau Summary of this function goes here
%   Detailed explanation goes here

h = 0;
dh_da = [];
for i = 1 : size(alpha,1)
    if(abs(alpha(i)) > obj.contacts(1).theta_)
        alpha(i) = (obj.contacts(1).theta_-1e-9)*sign(alpha(i));
    end
    h = h + H(alpha(i), obj.contacts(1).theta_, 5);
    % dh_da = [dh_da; dH_da(alpha(i), obj.contacts(1).theta_+0, 1)];
    dh_da = [dh_da; dH_da(alpha(i), obj.contacts(1).theta_, 10)*obj.contacts(i).n_hat_];
end

tau_ = obj.Go*dh_da;

end

