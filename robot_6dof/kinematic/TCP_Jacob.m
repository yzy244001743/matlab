function [J0, Je] = TCP_Jacob(angles,type_num)
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

%% 
T6t = trotz(tcp_rpy(3))*troty(tcp_rpy(2))*trotx(tcp_rpy(1))*transl(0,0,l_tcp);
Tb0 = transl(0,0,l_base);

%% MDH param [a, alpha, d, offset]
a = [0 0 a2 0 0 0];
alp = [0 -pi/2 0 -pi/2 pi/2 -pi/2];
d = [0 0 0  d4 0 0 ];
offset = [0 -pi/2 -pi/2 0 0 0];
th = [0 0 0 0 0 0];
%% theta
for i=1:6
    th(i) = offset(i)+angles(i);
end

%% T0i && Ti6
T01 = LinkMatMDH(a(1), alp(1), d(1), th(1));
T12 = LinkMatMDH(a(2), alp(2), d(2), th(2));
T23 = LinkMatMDH(a(3), alp(3), d(3), th(3));
T34 = LinkMatMDH(a(4), alp(4), d(4), th(4));
T45 = LinkMatMDH(a(5), alp(5), d(5), th(5));
T56 = LinkMatMDH(a(6), alp(6), d(6), th(6));
% Tbi
Tb1 = Tb0*T01;
Tb2 = Tb1*T12;
Tb3 = Tb2*T23;
Tb4 = Tb3*T34;
Tb5 = Tb4*T45;
Tb6 = Tb5*T56;
Tbt = Tb6*T6t
% Tit
T5t = T6t*T56;
T4t = T45*T5t;
T3t = T34*T4t;
T2t = T23*T3t;
T1t = T12*T2t;
% Pit in base-frame
P1t = Tb1(1:3,1:3)*T1t(1:3,4);
P2t = Tb2(1:3,1:3)*T2t(1:3,4);
P3t = Tb3(1:3,1:3)*T3t(1:3,4);
P4t = Tb4(1:3,1:3)*T4t(1:3,4);
P5t = Tb5(1:3,1:3)*T5t(1:3,4);
P6t = Tb6(1:3,1:3)*T6t(1:3,4);
% Jv
Jv = zeros(3,6);
Jv(:,1) = cross(Tb1(1:3,3), P1t);
Jv(:,2) = cross( Tb2(1:3,3), P2t);
Jv(:,3) =  cross(Tb3(1:3,3), P3t );
Jv(:,4) =  cross( Tb4(1:3,3), P4t);
Jv(:,5) =  cross( Tb5(1:3,3), P5t);
Jv(:,6) =  cross( Tb6(1:3,3), P6t);
% Jw
Jw = zeros(3,6);
Jw(:,1) = Tb1(1:3,3);
Jw(:,2) = Tb2(1:3,3);
Jw(:,3) = Tb3(1:3,3);
Jw(:,4) = Tb4(1:3,3);
Jw(:,5) = Tb5(1:3,3);
Jw(:,6) = Tb6(1:3,3);

J0 = [Jv;Jw ];

Ad = zeros(6,6); 
Ad(1:3,1:3) = (Tbt(1:3,1:3)).';
Ad(4:6,4:6) = (Tbt(1:3,1:3)).';
Je = Ad*J0;
end

