clear,clc,close all;
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

%% ����������DH��������ʼ״̬Ϊ��ֱ״̬
L1=Link('d',0,'a',0,'alpha',0,'offset',0,'modified'); 
L2=Link('d',0,'a',0,'alpha',-pi/2,'offset',-pi/2,'modified');
L3=Link('d',0,'a',a2,'alpha',0,'offset',-pi/2,'modified');
L4=Link('d',d4,'a',0,'alpha',-pi/2,'modified');
L5=Link('d',0,'a',0,'alpha',pi/2,'modified');
L6=Link('d',0,'a',0,'alpha',-pi/2,'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6')

% Theta=[-30.0000  -58.2807   30.0000  -61.7193  -30.0000  -30.0000];
Theta=[0.0000  90   -90.0000  0  0.0000  0.0000];
% Theta=[0.0000  0   0.0000  0  0.0000  0.0000];
Theta=Theta/180*pi;                      %����ɻ���
forwarda=robot.fkine(Theta)             %���������α任����
W=[-1.0,+1.0,-1.0,+1.0,-1.0,+1.0];  %����������ķ�Χ
robot.plot(Theta,'tilesize',0.15,'workspace',W);  %��ʾ��ά����
robot.teach(forwarda,'rpy' )              %��ʾroll/pitch/yaw angles��GUI�ɵ�����

theta_ik = robot.ikine(forwarda)*180/pi
