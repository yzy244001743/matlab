clear,clc,close all;

%% robot cfg
type_num=1;
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

%% DH参数
L1=Link('d',0,'a',0,'alpha',0,'offset',0,'modified'); 
L2=Link('d',0,'a',0,'alpha',-pi/2,'offset',-pi/2,'modified');
L3=Link('d',0,'a',a2,'alpha',0,'offset',-pi/2,'modified');
L4=Link('d',d4,'a',0,'alpha',-pi/2,'modified');
L5=Link('d',0,'a',0,'alpha',pi/2,'modified');
L6=Link('d',0,'a',0,'alpha',-pi/2,'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6')

%% joints test
% 正常
% Theta=[-30.0000  -30   30.0000  -30  -30.0000  -30.0000];
% Theta = [2.3, 120.3, -15.5, 11.5, 13.8, 19.7];
% 奇异测试
% Theta=[39.0000  0   0.0000  11.5,  0.0000  19.7];% j1、j4、j6共轴奇异 + 边界奇异 -- rank(J0)=4
% Theta=[7.0000  0   0.0000  0,  0.0000  0];% j1、j4、j6共轴奇异 + 边界奇异 -- rank(J0)=3
% Theta=[20.0000  0   0.0000  10  88.0000  20.0000];% j1、j4共轴奇异  + 边界奇异
Theta = [2.3, 120.3, -15.5, 11.5, 0, 19.7];% j4、j6共轴奇异
% Theta = [2.3, 120.3, 0, 11.5, 13.8, 19.7];% 边界奇异
%
% 腕部球面中点位于J1轴线上奇异
j2 = 30*pi/180;%>0
j2_j3 = asin(a2*sin(j2)/d4)
j3 = -(j2_j3 + j2);
j3_deg =j3*180/pi
% Theta=[10  j2*180/pi   j3_deg  11.5,  10.0000  19.7]
%% 
Theta=Theta/180*pi;                      
forwarda=robot.fkine(Theta)            
% W=[-1.0,+1.0,-1.0,+1.0,-1.0,+1.0];
% robot.plot(Theta,'tilesize',0.15,'workspace',W); 
% robot.teach(forwarda,'rpy' )              %显示roll/pitch/yaw angles，GUI可调界面
% theta_ik = robot.ikine(forwarda)*180/pi

J0_d = robot.jacob0(Theta)
Je_d = robot.jacobe(Theta)
[J0, Je] = JacobMDH(Theta,type_num)
[J0_b, Je_t] = TCP_Jacob(Theta,type_num) % reter to base_frame and tool_frame

% s = svd(J0)
cond_num_d = cond(J0_d)
cond_num = cond(J0)
cond_num_b = cond(J0_b)
rank_j0_d = rank(J0_d)
rank_j0 = rank(J0)
rank_j0_b = rank(J0_b)
if cond_num == Inf
disp("cond_num == Inf");
end



