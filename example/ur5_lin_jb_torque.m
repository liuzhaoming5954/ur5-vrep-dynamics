function [ jointVecCurrent,jointVelocityCurrent ] = ur5_lin_jb_torque( vrep, id, handles, res, startingJoints, threshold, p1, p2, pRot, jointsVec,robot )
%UR5_LIN_JB - Long Qian
%   Move the UR5 robot linearly with jacobian method
%   vrep, id, handles, res - VREP related variables
%   startingJoints - the difference between joint values read from
%   remoteAPI and my calculating system
%   threshold - the maximum difference to say a point is close enough to
%   target
%   p1 - position of the start
%   p2 - position of the end
%   pRot - the rotation matrix of start point and end point
%   jointsVec - the current joint-space vector

    stopflag = false;
    % catesian space vector
    euler = eulerzyxinv(pRot);
    cartVecCurrent = [p1, euler]'; % 当前位姿，欧拉角表示
    jointVecCurrent = reshape(jointsVec, [6,1]);
    
    cartVecTarget = [p2, euler]'; % 目标位姿，欧拉角表示
    cartVecDif = cartVecTarget - cartVecCurrent;
    
    G = [0 0 9.81];
    
    j = 1;
    while ~stopflag
        jointVec = jointVecCurrent;
        jb_inv = ur5_jacobian_inv(jointVecCurrent);
%         jointVecTarget = jointVecCurrent + jb_inv * cartVecDif / norm(cartVecDif) * 0.001;
        jointSpeedCurrent = jb_inv * cartVecDif / norm(cartVecDif); % LZM
        jointVecTarget = jointVecCurrent + jointSpeedCurrent * 0.001;
%         jointVecCurrent = ur5_ptp_jnt( vrep, id, handles, res, startingJoints, threshold, jointVecTarget );
        [jointVecCurrent,jointVelocityCurrent(j,:)] = ur5_ptp_jnt_velocity( vrep, id, handles, res, startingJoints, threshold, jointVecTarget );
        jointVecCurrent = reshape(jointVecCurrent, [6,1]);
        cartVecCurrent = cartvec(ur5fwdtrans(jointVecCurrent, 6));
        cartVecDif = cartVecTarget - cartVecCurrent;
        
        % 计算力矩
%         jb_inv = ur5_jacobian_inv(jointVecCurrent); % LZM
%         jointSpeedNext = jb_inv * cartVecDif / norm(cartVecDif); % LZM
%         jointAccCurrent = (jointSpeedNext - jointSpeedCurrent) * 1000;
%         torque(j,:) = robot.rne(jointVec',jointSpeedCurrent',jointAccCurrent','gravity',G);
        % 计算力矩结束
      
        j = j+1;
        
        stopflag = true;
        
        for i = 4:6 
            while cartVecDif(i) > pi
                cartVecDif(i) = cartVecDif(i) - 2*pi;
            end
            while cartVecDif(i) < -pi
                cartVecDif(i) = cartVecDif(i) + 2*pi;
            end
        end
        for i = 1:6
            if abs(cartVecDif(i)) > threshold
                stopflag = false;
            end
        end
    end
    
end

