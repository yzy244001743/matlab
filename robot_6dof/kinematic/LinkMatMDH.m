function T = LinkMatMDH(a, alp, d, theta)
% 改进D-H建模
%Theta, d, a, Alpha are lists (1 row matrix) with n inputs
%       Where n is the number of links in the arm.
%
% theta:     Joint Angles (theta)
% d:     Link offset
% a:     Link Length
% alp:   Link Twist (alpha)

T= [cos(theta)             -sin(theta)               0            a;
       sin(theta)*cos(alp)  cos(theta)*cos(alp)  -sin(alp)  -sin(alp)*d;
       sin(theta)*sin(alp)  cos(theta)*sin(alp)  cos(alp)   cos(alp)*d;
       0                        0                         0            1];
end
