function Fc = getContactForces(obj)
    Fc = [];
    for i = 1: size(obj.contacts, 2)
        Fc = [Fc; obj.contacts(i).Fc_];
    end
end