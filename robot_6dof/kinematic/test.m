clc;clear;
%% basic test
% matlab�б������ļ�������˵������ȫ�ֱ�������������򣬿ɳ���if-else-end����;
if 0
    if 1
        a = 20;
    else
        a = 0;
    end
    b = a;
    if b==20
        disp(b);
    end
end

list_1 = rand(100,1);

%% LinkMatMDH
% a= 0;
% alp = 0.2;
% d = 0.5;
% th = 0.5;
% Ti = LinkMatMDH(a, alp, d, th)

%% FK_MDH
% Theta=[39.0000  0   0.0000  0  0.0000  0.0000];% j1��j4��������
% Theta=Theta/180*pi;                     
% forwarda=FK_MDH(Theta)    


% J{4} = eye(3)
% J{1} = eye(3)
% J{2} = eye(3)
% J{3} = eye(3)
% J{4} = eye(3)
% disp(J)

% J = []
% J{1} = eye(3)
% J{2} = eye(3)
% J{3} = eye(3)
% J{4} = eye(3)
% disp(J)
% size(J)




%% IK_MDH
% T06=[
%     0.2391   -0.0979    0.9660    0.4237;
%    -0.5088   -0.8600    0.0388    0.0170;
%     0.8270   -0.5008   -0.2554   -0.1828;
%          0         0         0    1.0000];
% ik_all = IK_MDH(T06)*180/pi

%% �������е�λ��J1������,j2 j3�����෴
% d4 = 0.210;
% a2 = 0.256;
% j2 = 30*pi/180
% j2_j3 = asin(a2*sin(j2)/d4)
% j3 = -(j2_j3 - j2)*180/pi

%% sym
% desc::���ڼ��㣬z����ˮƽ�汣��һ���н�ʱ��rx��ry֮��Ĺ�ϵ 
% ���ڻ���worspace,���滮�Բο�
if 0
    syms rx  ry  rz
    Rot_x = [
        1,     0,      0;
        0, cos(rx), -sin(rx);
        0, sin(rx), cos(rx);
        ]

    Rot_y = [
        cos(ry),  0, sin(ry);
        0,              1,              0;
        -sin(ry), 0, cos(ry);
        ]

    Rot_z =[
        cos(rz), -sin(rz), 0;
        sin(rz), cos(rz), 0;
        0, 0, 1;
        ]

    R = Rot_z*Rot_y*Rot_x

    z_axis = [0,0,1]
    z_axis_new = R*z_axis.'

    cos_theta = dot(z_axis, z_axis_new) % cos(rx)*cos(ry) = cos(theta)
end 

%% test rpyxyz2T()
if 1
    clc;clear;
    theta = 30*pi/180;
    rpy = [1,2,theta];
    rpyxyz = [rpy,0.11,0.22,0.33]

    rz = trotz(rpyxyz(3))
    ry = troty(rpyxyz(2))
    rx = trotx(rpyxyz(1))
    r_zyx = rz*ry*rx
    tt = transl(rpyxyz(4:6))
%     rz_ = rotz(rpyxyz(3))
    T1 = tt*r_zyx

    T = rpyxyz2T(rpyxyz)
end





