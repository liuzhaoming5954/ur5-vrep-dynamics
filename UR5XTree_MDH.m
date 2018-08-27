function UR5_3D = UR5XTree_MDH()
%ur5_model;
% d1 = 0.089159;
% d2 = 0;
% d3 = 0;
% d4 = 0.10915;
% d5 = 0.09465;
% d6 = 0.0823;
% 
% a0 = 0;
% a1 = 0;
% a2 = 0.425;
% a3 = 0.39225;
% a4 = 0;
% a5 = 0;
% 
% alpha0 = 0;
% alpha1 = pi/2;
% alpha2 = 0;
% alpha3 = 0;
% alpha4 = pi/2;
% alpha5 = -pi/2;

% T01 = MDH(a0, alpha0, d1, 0);
% T12 = MDH(a1, alpha1, d2, 0);
% T23 = MDH(a2, alpha2, d3, 0);
% T34 = MDH(a3, alpha3, d4, 0);
% T45 = MDH(a4, alpha4, d5, 0);
% T56 = MDH(a5, alpha5, d6, 0);
% 
% T{1}=T01; T{2}=T12; T{3}=T23; T{4}=T34; T{5}=T45; T{6}=T56;

L1 = Link('d', 0.08916, 'a', 0, 'alpha',0,'modified');
L2 = Link( 'd',0, 'a',0, 'alpha', pi/2,'modified');
L3 = Link('d', 0,'a', 0.425,'alpha', 0,'modified');
L4 = Link('d', 0.10915,'a', 0.39225,'alpha', 0,'modified');
L5 = Link('d', 0.09456,'a', 0,'alpha', pi/2,'modified');
L6 = Link('d', 0.0823,'a', 0,'alpha', -pi/2,'modified');
ur5 = SerialLink([L1 L2 L3 L4 L5 L6],'name','ur');

T{1}=ur5.links(1).A(0); 
T{2}=ur5.links(2).A(pi/2); 
T{3}=ur5.links(3).A(0); 
T{4}=ur5.links(4).A(pi/2); 
T{5}=ur5.links(5).A(0); 
T{6}=ur5.links(6).A(0);

% ¹ßÁ¿¾ØÕó
%inertial.I{1} = [0.00443333156 0 0; 0 0.00443333156 0; 0 0 0.0072];
UR5_3D.inertial.I{1} = [0.010267495893 0 0; 0 0.010267495893 0; 0 0 0.00666]; UR5_3D.inertial.mass(1)=3.7; UR5_3D.inertial.CoM{1}=[0.0 0.0 0.0];
UR5_3D.inertial.I{2} = [0.22689067591 0 0; 0 0.22689067591 0; 0 0 0.0151074]; UR5_3D.inertial.mass(2)=8.393; UR5_3D.inertial.CoM{2}=[0.0 0.0 0.28];
UR5_3D.inertial.I{3} = [0.049443313556 0 0; 0 0.049443313556 0; 0 0 0.004095]; UR5_3D.inertial.mass(3)=2.275; UR5_3D.inertial.CoM{3}=[0.0 0.0 0.25];
UR5_3D.inertial.I{4} = [0.111172755531 0 0; 0 0.111172755531 0; 0 0 0.21942]; UR5_3D.inertial.mass(4)=1.219; UR5_3D.inertial.CoM{4}=[0.0 0.0 0.0];
UR5_3D.inertial.I{5} = [0.111172755531 0 0; 0 0.111172755531 0; 0 0 0.21942]; UR5_3D.inertial.mass(5)=1.219; UR5_3D.inertial.CoM{5}=[0.0 0.0 0.0];
UR5_3D.inertial.I{6} = [0.0171364731454 0 0; 0 0.0171364731454 0; 0 0 0.033822]; UR5_3D.inertial.mass(6)=0.1879; UR5_3D.inertial.CoM{6}=[0.0 0.0 0.0];

% for i=1:6
%     E{i} = T{i}(1:3,1:3);
%     r{i} = T{i}(1:3,4);
% end

for i=1:6
    UR5_3D.E{i} = [T{i}.n,T{i}.o,T{i}.a];
    UR5_3D.r{i} = T{i}.t;
end