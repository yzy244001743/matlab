function T06 = FK_MDH(angles,type_num)
% MDH½¨Ä£
% theta:     Joint Angles (theta)
% d:     Link offset
% a:     Link Length
% alp:   Link Twist (alpha)

%% robot cfg
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

%% fk
T0_n = eye(4);
for i = 1 : 6
    Ti = LinkMatMDH(a(i), alp(i), d(i), th(i));
    T0_n = T0_n * Ti;
end
T06 = T0_n ;
end
