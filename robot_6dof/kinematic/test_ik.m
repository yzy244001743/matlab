clear;
clc;
%% robot cfg
type_num = 1;
if type_num==1
    % for RM65
    d4 = 0.210;
    a2 = 0.256;
    l_tcp = 0.113;%0.144 - 0.031
    l_base = 0.2405;
    tcp_rpy = [0,0,pi];

else
    % for hwj_arm
    d4 = 0.2322;
    a2 = 0.260;
    l_tcp = 0.1382;
    l_base = 0.2035;
    tcp_rpy = [0,0,0];
end

%% 建立机器人DH参数，初始状态为竖直状态       
L1=Link('d',0,'a',0,'alpha',0,'modified'); 
L2=Link('d',0,'a',0,'alpha',-pi/2,'offset',-pi/2,'modified');
L3=Link('d',0,'a',a2,'alpha',0,'offset',-pi/2,'modified');
L4=Link('d',d4,'a',0,'alpha',-pi/2,'modified');
L5=Link('d',0,'a',0,'alpha',pi/2,'modified');
L6=Link('d',0,'a',0,'alpha',-pi/2,'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6')

%% 测试 joints 
% 正常
% Theta=[-30.0000  -30   30.0000  -30  -30.0000  -30.0000];
Theta = [2.3, 120.3, -15.5, 11.5, 13.8, 19.7];
% 奇异测试
% Theta=[39.0000  0   0.0000  11.5,  0.0000  19.7];% j1、j4、j6共轴奇异 + 边界奇异 
% Theta=[20.0000  0   0.0000  10  88.0000  20.0000];% j1、j4共轴奇异  + 边界奇异
% Theta = [2.3, 120.3, -15.5, 11.5, 0, 19.7];% j4、j6共轴奇异
% Theta = [2.3, 120.3, 0, 11.5, 13.8, 19.7];% 边界奇异

% 腕部球面中点位于J1轴线上奇异
j2 = 30*pi/180;%>0
j2_j3 = asin(a2*sin(j2)/d4)
j3 = -(j2_j3 + j2);
j3_deg =j3*180/pi
% Theta=[10  j2*180/pi   j3_deg  11.5,  10.0000  19.7]
%% viewer
Theta=Theta/180*pi;                     
forwarda=robot.fkine(Theta)             
W=[-1.0,+1.0,-1.0,+1.0,-1.0,+1.0];  
robot.plot(Theta,'tilesize',0.150,'workspace',W);  
robot.teach(forwarda,'rpy')              
theta_ik_numer = robot.ikine(forwarda)*180/pi

%% ik test
T = [forwarda.n(1,1), forwarda.o(1,1),forwarda.a(1,1),forwarda.t(1,1);
    forwarda.n(2,1), forwarda.o(2,1),forwarda.a(2,1),forwarda.t(2,1);
    forwarda.n(3,1), forwarda.o(3,1),forwarda.a(3,1),forwarda.t(3,1);
    0,0,0,1];

[~,theta_ik_all] = IK_MDH(T, type_num);
theta_ik_all_deg = theta_ik_all*180/pi

%% check fk using Tools
if 0
    forwarda_1 = robot.fkine(theta_ik_all(1,:)) 
    forwarda_2 = robot.fkine(theta_ik_all(2,:))

    forwarda_3 = robot.fkine(theta_ik_all(3,:)) 
    forwarda_4 = robot.fkine(theta_ik_all(4,:))

    forwarda_5 = robot.fkine(theta_ik_all(5,:)) 
    forwarda_6 = robot.fkine(theta_ik_all(6,:))

    forwarda_7 = robot.fkine(theta_ik_all(7,:)) 
    forwarda_8 = robot.fkine(theta_ik_all(8,:))
end

%% check fk using FK_MDH()
if 1
    fk_1 =FK_MDH(theta_ik_all(1,:),type_num) 
    fk_2 =FK_MDH(theta_ik_all(2,:), type_num)

    fk_3 =FK_MDH(theta_ik_all(3,:), type_num) 
    fk_4 =FK_MDH(theta_ik_all(4,:),type_num)

    fk_5 =FK_MDH(theta_ik_all(5,:),type_num) 
    fk_6 =FK_MDH(theta_ik_all(6,:),type_num)

    fk_7 =FK_MDH(theta_ik_all(7,:),type_num) 
    fk_8 =FK_MDH(theta_ik_all(8,:),type_num)
end

%% view
% [r,c ] = size(theta_ik_all);
% for i=1:r
%     joints = theta_ik_all(i,:);
% %     robot.teach(joints )
%     robot.plot(joints )
%     pause(1.0);
% end
% disp('test end ');

%% ik filter
input_angle = Theta
refer_angle = Theta - [0.1 -0.1 0.07 0.08 -0.05, 0.068]
[flag, ik_opt] = ik_filter(theta_ik_all, refer_angle);
if flag
    disp(ik_opt);
end


