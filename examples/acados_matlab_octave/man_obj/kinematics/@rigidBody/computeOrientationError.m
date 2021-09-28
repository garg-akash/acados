function [eo, eo_dot] = computeOrientationError(obj, Rd, omegad)
%COMPUTEORIENTATIONERROR Summary of this function goes here
%   Detailed explanation goes here

Te = obj.getGlobalPose();
Re = Te(1:3, 1:3);
nd = Rd(1:3, 1);
ne = Re(1:3, 1);
sd = Rd(1:3, 2);
se = Re(1:3, 2);
ad = Rd(1:3, 3);
ae = Re(1:3, 3);

eo = 0.5*(cross(ne, nd) + cross(se, sd) + cross(ae, ad));
L = -0.5*(skew(nd)*skew(ne)+skew(sd)*skew(se)+skew(ad)*skew(ae));
t = obj.getBodyTwist();
omegae = t(4:6,1);

eo_dot = L'*omegad - L*omegae;


end

