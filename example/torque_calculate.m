    
    p1 = [0.4800 0.1000 0.3000];
    p2 = [0.4800 -0.2000 0.3000];
    pRot = [0 -1 0; -1 0 0; 0 0 -1];
    jointVecCurrent = [-3.1607;-1.5366;1.8840;-1.9182;-1.5708;-0.0191];

    euler = eulerzyxinv(pRot);
    cartVecCurrent = [p1, euler]';
    cartVecTarget = [p2, euler]';
    cartVecDif = cartVecTarget - cartVecCurrent;
    
    for i=1:300
        jb_inv = ur5_jacobian_inv(jointVecCurrent);
        jointSpeedCurrent = jb_inv * cartVecDif / norm(cartVecDif); % 计算关节当前速度

        jointVecTarget = jointVecCurrent + jointSpeedCurrent * 0.001; % 频率1k
        jb_inv = ur5_jacobian_inv(jointVecTarget);
        jointSpeedNext = jb_inv * cartVecDif / norm(cartVecDif); % 计算关节当前速度
        jointAccCurrent = (jointSpeedNext - jointSpeedCurrent) * 1000;

        % 计算力矩
        torque(i) = ur5.rne(jointVecCurrent',jointSpeedCurrent',jointAccCurrent','gravity',G);
        jointVecCurrent = jointVecTarget;
    end
    
    jointVecCurrent = ur5_ptp_jnt( vrep, id, handles, res, startingJoints, threshold, jointVecTarget );
    jointVecCurrent = reshape(jointVecCurrent, [6,1]);
    cartVecCurrent = cartvec(ur5fwdtrans(jointVecCurrent, 6));
    cartVecDif = cartVecTarget - cartVecCurrent;
    stopflag = true;