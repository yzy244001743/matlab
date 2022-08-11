function [J0,Je]= JacobMDH(angles,type_num)
if type_num==1
    % for RM65
    d4 = 0.210;
    a2 = 0.256;
else
    % for hwj_arm
    d4 = 0.2322;
    a2 = 0.260;
end
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
% T0i
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;
% Ti6
T46 = T45*T56;
T36 = T34*T46;
T26 = T23*T36;
T16 = T12*T26;
T66 = eye(4);

% Pi6 in 0-frame
P16 = T01(1:3,1:3)*T16(1:3,4);
P26 = T02(1:3,1:3)*T26(1:3,4);
P36 = T03(1:3,1:3)*T36(1:3,4);
P46 = T04(1:3,1:3)*T46(1:3,4);
P56 = T05(1:3,1:3)*T56(1:3,4);
P66 = T06(1:3,1:3)*T66(1:3,4);

% Jv
Jv = zeros(3,6);
Jv(:,1) = cross(T01(1:3,3), P16);
Jv(:,2) = cross( T02(1:3,3), P26);
Jv(:,3) =  cross(T03(1:3,3), P36 );
Jv(:,4) =  cross( T04(1:3,3), P46);
Jv(:,5) =  cross( T05(1:3,3), P56);
Jv(:,6) =  cross( T06(1:3,3), P66);

% Jw
Jw = zeros(3,6);
Jw(:,1) = T01(1:3,3);
Jw(:,2) = T02(1:3,3);
Jw(:,3) = T03(1:3,3);
Jw(:,4) = T04(1:3,3);
Jw(:,5) = T05(1:3,3);
Jw(:,6) = T06(1:3,3);

J0 = [Jv;Jw ];

Ad = zeros(6,6); 
Ad(1:3,1:3) = (T06(1:3,1:3)).';
Ad(4:6,4:6) = (T06(1:3,1:3)).';
Je = Ad*J0;

end

