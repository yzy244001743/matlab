function T = LinkMatMDH(a, alp, d, theta)
T= [cos(theta)             -sin(theta)               0            a;
       sin(theta)*cos(alp)  cos(theta)*cos(alp)  -sin(alp)  -sin(alp)*d;
       sin(theta)*sin(alp)  cos(theta)*sin(alp)  cos(alp)   cos(alp)*d;
       0                        0                         0            1];
end
