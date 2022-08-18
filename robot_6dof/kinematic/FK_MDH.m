function T06 = FK_MDH(angles,type_num)
% MDH½¨Ä£
%Theta, d, a, Alpha are lists (1 row matrix) with n inputs
%       Where n is the number of links in the arm.
%
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
% T{n} = zeros(4);
for i = 1 : 6
%     Ti= [cos(th(i))             -sin(th(i))               0            a(i);
%            sin(th(i))*cos(alp(i))  cos(th(i))*cos(alp(i))  -sin(alp(i))  -sin(alp(i))*d(i);
%            sin(th(i))*sin(alp(i))  cos(th(i))*sin(alp(i))  cos(alp(i))   cos(alp(i))*d(i);
%            0                        0                         0            1];
    Ti = LinkMatMDH(a(i), alp(i), d(i), th(i));
    T0_n = T0_n * Ti;
%     T{i} = T0_n;
end
T06 = T0_n ;
% Pos = T06(1: 3, 4)';

end
