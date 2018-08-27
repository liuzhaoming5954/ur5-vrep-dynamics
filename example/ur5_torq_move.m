function [ currentJoints ] = ur5_torq_move( vrep, id, handles, res, startingJoints, threshold, q, qd, qdd, ur5 )
%UR5_TORQ_MOVE - Liu Zhaoming
%   Move the UR5 robot with spatial vector method
%   vrep, id, handles, res - VREP related variables
%   startingJoints - the difference between joint values read from
%   remoteAPI and my calculating system
%   q - target position
%   qd - target velocity
%   qdd - target acceleration

%   Move the UR5 robot so that the end-effector is at joint vector theta
    targetJointsActual = reshape(q, [1,6]) + startingJoints;

% calculate torque
% tau = ID( ur5_6D, targetJointsActual, qd, qdd);
G = [0 0 9.81];
tau = ur5.rne(q',qd',qdd','gravity',G);

% set target torque
vrep.simxPauseCommunication(id, 1);
for i = 1:6
   res = vrep.simxSetJointTargetVelocity(id, handles.ur5Joints(i), sign(qd(i))*10, vrep.simx_opmode_oneshot);
   res = vrep.simxSetJointForce(id, handles.ur5Joints(i), abs(tau(i)), vrep.simx_opmode_oneshot);
   %vrep.simxSetJointForce(id, handles.ur5Joints(2), 1, vrep.simx_opmode_oneshot);
   vrchk(vrep, res, true);
end
vrep.simxPauseCommunication(id, 0);

    % Check if the robot arrives at the desired configuration
    currentJoints = zeros(1,6);
    reached = false;
    while ~reached
        % Get current joint angles for each joint
        for i = 1:6
            [res, currentJoints(i)] = vrep.simxGetJointPosition(id, handles.ur5Joints(i),...
                vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
        end
        Gcurrent = ur5fwdtrans(currentJoints-startingJoints, 6);
        movef( id, vrep, Gcurrent, handles.FrameEnd, handles.base);
        % Check whether all joints fall into the threshold
        % If yes, then we think the target is reached
        diffJoints = currentJoints - targetJointsActual;
        for j = 1:6
            if diffJoints(j) > pi
                diffJoints(j) = diffJoints(j) - 2*pi;
            elseif diffJoints(j) < -pi
                diffJoints(j) = diffJoints(j) + 2*pi;
            end
        end
        if max(abs(diffJoints)) < threshold
            reached = true;
        end
    end
    currentJoints = currentJoints - startingJoints;

end
