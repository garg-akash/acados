function q_ = toRodyman(q)
    q_ = [q(1:2,:); zeros(1,size(q,2)); q(3:end,:)];
end