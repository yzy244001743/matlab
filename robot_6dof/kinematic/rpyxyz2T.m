function T = rpyxyz2T(rpyxyz)
Rot = rotz( rpyxyz(3) ) * roty( rpyxyz(2) ) * rotx( rpyxyz(1) );
Pos = rpyxyz(4:6);
T = eye(4);
T(1:3, 1:3) = Rot(1:3, 1:3);
T(1:3, 4) = Pos(1:3);
% T = trotz(rpyxyz(3))*troty(rpyxyz(2))*trotx(rpyxyz(1))*transl(rpyxyz(4:6));
end

