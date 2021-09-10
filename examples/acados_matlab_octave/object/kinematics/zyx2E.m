function E = zyx2E(chi)

% transforms time derivative of euler angles (zyx) in angular velocity (w)
% expressed in the base frame, i.e. w = E*zyx

E = [0, -sin(chi(2)), cos(chi(1))*cos(chi(2));
     0,  cos(chi(2)), cos(chi(1))*sin(chi(2));
     1,            0, -sin(chi(1))];

end