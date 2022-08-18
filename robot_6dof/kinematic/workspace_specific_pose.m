clc;clear;
%% param cfg
% for 睿尔曼: tcp到wrist_center距离0.144;清洁工具长度0.287;
type_num=0;
L_tool = 0.144+0.287;% 0; 0.113; 0.144+0.287;
theta = 60*pi/180; %Tool-Z与XOY面的夹角; 30; 0; -30; -60; -90;
%% robot cfg
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
L = 1.0;% 0.6
W=[-1.0*L,+1.0*L,-1.0*L,+1.0*L,-1.0*L,+1.0*L]; 

%% sample space 
% pose
rpy = [0, -(0.5*pi - theta), pi ];% f6相对于f0,有个固定offset:rz = pi

% space limit
x_l = 0;
x_u = a2+d4;
y_l = -(a2+d4);
y_u = (a2+d4);
z_l = 0;
z_l = -(a2+d4);
z_u = (a2+d4);% (a2+d4)
% z_u = 0;% (a2+d4)

% check have ik or not
i=1;
step = 0.02;
N_z = floor((z_u - z_l)/step) + 1;
N_y = floor((y_u - y_l)/step) + 1;
N_x = floor((x_u - x_l)/step) + 1;
N_MAX = N_z*N_y*N_x;
ex=zeros(1,N_MAX);
ey=zeros(1,N_MAX);
ez=zeros(1,N_MAX);
for z = z_l:step:z_u
     for y = y_l:step:y_u
         for x = x_l:step:x_u
             % cal ik and check
             pos = [x,y,z];
             T = rpyxyz2T([rpy,pos]);
             [flag_ik,ik_all] = IK_MDH(T,type_num);
             if flag_ik
                refer_angle = [0.0 0.0 0.0 0.0 -0.0 0.0];
                [flag, ~] = ik_filter(ik_all, refer_angle);
                 if flag
                     L_tool_z = L_tool*T(1:3, 3); % z-axis dir
                     ex(i)= x + L_tool_z(1);
                     ey(i)= y + L_tool_z(2);
                     ez(i)= z + L_tool_z(3);
                     i=i+1;
                 end
             end
         end
     end
end

%% plot
ex = ex(1:i);
ey = ey(1:i);
ez = ez(1:i);

% - using robotics Toolbox
robot.plot(Theta,'workspace',W); 
robot.teach(forwarda,'rpy') 
hold on
plot3(ex,ey,ez,'b.');

% - not using  robotics Toolbox
% plot3(ex,ey,ez,'.');

hold on
grid on
xlabel('X');ylabel('Y');zlabel('Z');














