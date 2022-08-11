function [flag,ik]= ik_filter (ik_all, refer_angle)
%% limit
DEG2RAD = pi/180.0;

% for RM65 && hwj-arm
j1_l = -pi;
j1_u = pi;
j2_l = -130*DEG2RAD;
j2_u = 130*DEG2RAD;
j3_l = -135*DEG2RAD;
j3_u = 135*DEG2RAD;
j4_l = -pi;
j4_u = pi;
j5_l = -128*DEG2RAD;
j5_u = 128*DEG2RAD;
j6_l = -1*pi;% 2*pi
j6_u = 1*pi;

%% check limit
[r, c] = size(ik_all);
if c ~= 6
    disp("input date error");
    flag = false;
    ik = zeros(1,6);
    return
end
num = 0;
index_list = zeros(1,r);

for i=1:r
    ik_i = ik_all(i,:);
    if ( j1_l<ik_i(1,1) && j1_u>ik_i(1,1) && ...
        j2_l<ik_i(1,2) && j2_u>ik_i(1,2) && ...
        j3_l<ik_i(1,3) && j3_u>ik_i(1,3) && ...
        j4_l<ik_i(1,4) && j4_u>ik_i(1,4) && ...
        j5_l<ik_i(1,5) && j5_u>ik_i(1,5) && ...
        j6_l<ik_i(1,6) && j6_u>ik_i(1,6) )
    num = num+1;
    index_list(num) = i;
    end
end

if num == 0
    disp("no ik meet angle constraint");
    flag = false;
    ik = zeros(1,6);
    return
elseif num == 1
    flag = true;
    ik = ik_all(index_list(1),:);
    return;
end

%% cost
w = [2,2,2,1,1,1];
cost_min = 0;
opt_index = index_list(1);
ik_first = ik_all(index_list(opt_index),:);
for i=1:6
	cost_min = cost_min + w(i)*abs( ik_first(i) - refer_angle(i) );
end

for j=2:num
   ik_j = ik_all(index_list(j),:);
   cost_j = 0;
    for i=1:6
        cost_j = cost_j + w(i)*abs( ik_j(i) - refer_angle(i) );
    end
    if cost_j < cost_min
        opt_index = index_list(j);
    end
end

flag = true;
ik = ik_all(opt_index,:);

end

