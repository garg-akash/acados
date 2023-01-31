load ("data_rodyman_16-Jan-2023 11:21:59")


coppelia = remApi('remoteApi');
coppelia.simxFinish(-1);
clientID = coppelia.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID>-1)
    disp('remote API server connected successfully!');
    handle_joint = zeros(9,1);
    [rtn, handle_joint(1)] = coppelia.simxGetObjectHandle( clientID, 'Joint_torso_1_ID17',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(2)] = coppelia.simxGetObjectHandle( clientID, 'Joint_torso_2_ID18',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(3)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_1_ID19', coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(4)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_2_ID5',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(5)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_3_ID6',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(6)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_4_ID7',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(7)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_5_ID8',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(8)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_6_ID9',  coppelia.simx_opmode_oneshot_wait);
    [rtn, handle_joint(9)] = coppelia.simxGetObjectHandle( clientID, 'Joint_arm_dx_7_ID10', coppelia.simx_opmode_oneshot_wait);
    disp('joint handles retrieved!');
    [rtn, handle_object] = coppelia.simxGetObjectHandle( clientID, 'Cub', coppelia.simx_opmode_oneshot_wait);
    disp('object handle retrieved!');
else
    disp('failed connecting to remote API server!');
    coppelia.simxFinish(clientID);  % close the line if still open
    coppelia.delete();              % call the destructor!
    return
end

q = toSim(q_log(:,1));
for i=1:size(handle_joint)
    [err]=coppelia.simxSetJointPosition(clientID,handle_joint(i),q(i),coppelia.simx_opmode_oneshot_wait);
    if (err~=coppelia.simx_return_ok)
        fprintf('failed to set the position of joint %d \n',i);
    end
end
%coppelia.simxSetObjectPosition(clientID,handle_object,-1,[0.7,-0.42,1.4376],coppelia.simx_opmode_oneshot_wait)
%coppelia.simxSetObjectOrientation(clientID,handle_object,-1,[0,0,0],coppelia.simx_opmode_oneshot_wait)



for ii = 1:size(q_log,2)-1
    q = toSim(q_log(:,ii));
    for i=1:size(handle_joint)
        [err]=coppelia.simxSetJointPosition(clientID,handle_joint(i),q(i),coppelia.simx_opmode_oneshot);
        if (err~=coppelia.simx_return_ok)
            fprintf('failed to set the position of joint %d \n',i);
        end
    end
    pause(0.008);
end


q = toSim(q_log(:,1));
for i=1:size(handle_joint)
    [err]=coppelia.simxSetJointPosition(clientID,handle_joint(i),q(i),coppelia.simx_opmode_oneshot);
    if (err~=coppelia.simx_return_ok)
        fprintf('failed to set the position of joint %d \n',i);
    end
end
%coppelia.simxSetObjectPosition(clientID,handle_object,-1,[0.7,-0.42,1.4376],coppelia.simx_opmode_oneshot_wait)
%coppelia.simxSetObjectOrientation(clientID,handle_object,-1,[0,0,0],coppelia.simx_opmode_oneshot_wait)
