clear;
clc;

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
    tcp_rpy = [0,0,pi];
end


%% ≤‚ ‘ joints 
% ’˝≥£
% Theta=[-30.0000  -30   30.0000  -30  -30.0000  -30.0000];
% Theta = [2.3, 120.3, -15.5, 11.5, 13.8, 19.7];
% ∆Ê“Ï≤‚ ‘
Theta=[39.0000  0   0.0000  11.5,  0.0000  19.7];% j1°¢j4°¢j6π≤÷·∆Ê“Ï + ±ﬂΩÁ∆Ê“Ï 
% Theta=[20.0000  0   0.0000  10  88.0000  20.0000];% j1°¢j4π≤÷·∆Ê“Ï  + ±ﬂΩÁ∆Ê“Ï
% Theta = [2.3, 120.3, -15.5, 11.5, 0, 19.7];% j4°¢j6π≤÷·∆Ê“Ï
% Theta = [2.3, 120.3, 0, 11.5, 13.8, 19.7];% ±ﬂΩÁ∆Ê“Ï

%% tcp fk
% rotx_t =trotx(0) ;
% t = transl(1,2,20)

Theta=Theta/180*pi;
tcp_fk = TCP_FK(Theta,type_num)

%% tcp ik
[~, tcp_ik_all] = TCP_IK(tcp_fk,type_num);
tcp_ik_all = tcp_ik_all*180/pi

    






