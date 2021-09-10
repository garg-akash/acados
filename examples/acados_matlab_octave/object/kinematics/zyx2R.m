function R = zyx2R(angles)
%     R = rotz(angles(1))*roty(angles(2))*rotx(angles(3));
    z = angles(1);
    y = angles(2);
    x = angles(3);
    c_x = cos(x);
    s_x = sin(x);
    c_y = cos(y);
    s_y = sin(y);
    c_z = cos(z);
    s_z = sin(z);
    
    rot_z = [c_z, -s_z, 0; s_z, c_z, 0; 0, 0, 1];
    rot_y = [c_y, 0, s_y; 0, 1, 0; -s_y, 0, c_y];
    rot_x = [1, 0, 0; 0, c_x, -s_x; 0, s_x, c_x];
    R = rot_z*rot_y*rot_x;
end