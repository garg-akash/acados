function reset(obj)

% init object state
g = [eye(3), zeros(3,1);
    0 0 0 1];
obj.setGlobalPose(g);
obj.setBodyTwist(zeros(6,1));
obj.setBodyAcceleration(zeros(6,1));
%obj.computeReactionForces([0;0; obj.mass*9.81; 0;0;0]);

end