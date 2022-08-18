clc;clear;
%% robot cfg
type_num=0;
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

%% viewer
Theta=[0.0000  30   30.0000  -30,  30  30];
Theta=Theta/180*pi;                     
forwarda=robot.fkine(Theta)             
W=[-1.0,+1.0,-1.0,+1.0,-1.0,+1.0];  
 
%% limit
DEG2RAD = pi/180.0;
% for RM65
j1_l = -pi;
j1_u = pi;
j2_l = -130*DEG2RAD;
j2_u = 130*DEG2RAD;
j3_l = -135*DEG2RAD;
j3_u = 135*DEG2RAD;
j4_l = -pi;
j4_u = pi;
j5_l = -128*DEG2RAD;
j5_u = 128*DEG2RAD;
j6_l = -1*pi;% 2*pi
j6_u = 1*pi;

%% workspace plot
N= 8000;                                              
theta1= j1_l+(-j1_l +j1_u)*rand(N,1);
theta2= j2_l+(-j2_l +j2_u)*rand(N,1);
theta3= j3_l+(-j3_l +j3_u)*rand(N,1);  
theta4= j4_l+(-j4_l +j4_u)*rand(N,1); 
theta5= j5_l+(-j5_l +j5_u)*rand(N,1); 
theta6= j6_l+(-j6_l +j6_u)*rand(N,1); 

% robot.teach()
robot.plot(Theta,'tilesize',0.150,'workspace',W); 
robot.teach(forwarda,'rpy') 
hold on
for n=1:1:N
    % - using robotics Toolbox
    pp=robot.fkine([theta1(n) theta2(n) theta3(n) theta4(n) theta5(n) theta6(n)]);
    plot3(pp.t(1),pp.t(2),pp.t(3),'b.','MarkerSize',1.0);
    % - using FK_MDH or TCP_FK
%     pp = FK_MDH([theta1(n) theta2(n) theta3(n) theta4(n) theta5(n) theta6(n)],type_num);
%     plot3(pp(1,4),pp(2,4),pp(3,4),'b.','MarkerSize',1.0);
    hold on
end
disp("end");






