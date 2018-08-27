function robot = ur5_model()  
    % UR5 with MDH
    deg = pi/180;
    
    % robot length values (metres)
    a = [0, 0, 0.425, 0.39225, 0, 0]';

    d = [0.089416, 0, 0, 0.10915, 0.09465, 0.0823]';

    alpha = [0, pi/2, 0, 0, pi/2, -pi/2]';
    
    theta = [0, 0, 0, 0, 0, 0]';
    
    DH = [theta d a alpha];
    
    for i=1:6
        L(i) = Link(DH(i,:),'modified');
    end

    mass = [3.7000, 8.3930, 2.275, 1.2190, 1.2190, 0.1897];

    center_of_mass = [
        0,-0.02561, 0.00193
        0.2125, 0, 0.11336
        0.15, 0, 0.0265
        0, -0.0018, 0.01634
        0, -0.0018, 0.01634
        0, 0, -0.001159];
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

%     robot = SerialLink(DH, ...
%         'name', 'UR5', 'manufacturer', 'Universal Robotics');
    robot = SerialLink(L,'name', 'UR5', 'manufacturer', 'Universal Robotics');

    % add the mass data, no inertia available
    links = robot.links;
    for i=1:6
        links(i).m = mass(i);
        links(i).r = center_of_mass(i,:);
    end

    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'ur5', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % vertical pose as per Fig 2
    end
end