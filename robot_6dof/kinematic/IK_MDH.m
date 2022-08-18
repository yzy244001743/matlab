% kerrick
% 20220804
% 适用于Modified DH参数法;hwj自研臂以及睿尔曼
% status : 0,1,2; 求解ok；奇异位型；求解失败
function [flag, ik_all] = IK_MDH(fk_T,type_num)
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

offset1 = 0; offset2 = -0.5*pi;offset3 = -0.5*pi;
offset4 = 0.0; offset5 = 0.0; offset6 = 0.0;

nx = fk_T(1, 1); ox = fk_T(1, 2); ax = fk_T(1, 3); px = fk_T(1, 4);
ny = fk_T(2, 1); oy = fk_T(2, 2); ay = fk_T(2, 3); py = fk_T(2, 4);
nz = fk_T(3, 1); oz = fk_T(3, 2); az = fk_T(3, 3); pz = fk_T(3, 4);

eps = 1e-4;%1e-6
%% check range
r = (px^2 + py^2 + pz^2)^0.5;
if r > (a2 + d4)
%     disp('Out of workspace Unable to solve');
    flag = false;
    ik_all = zeros(1,6);
    return; 
end

%% theta1有两个解
% d2=0;
% theta1_1 = atan2(py, px) - atan2(d2, (px^2+py^2-d2^2)^0.5);
% theta1_2 = atan2(py, px) - atan2(d2, -(px^2+py^2-d2^2)^0.5);
if abs(px)<eps && abs(py)<eps %奇异
   theta1_1 = 0;%0或取当前值
   theta1_2 = pi
else     
    theta1_1 = atan2(py, px);
    if theta1_1>0
        theta1_2 = atan2(py, px)-pi;
    else
        theta1_2 = atan2(py, px)+pi;
    end
end
%% theta2 有4个解
e1 = px*cos(theta1_1) + py*sin(theta1_1);
k1 = (e1^2 + pz^2 +a2^2 -d4^2)/(2*a2);
e2 = px*cos(theta1_2) + py*sin(theta1_2);
k2 = (e2^2 + pz^2 +a2^2 -d4^2)/(2*a2);
ek1 = e1^2 + pz^2 - k1^2;
ek2 = e2^2 + pz^2 - k2^2;

if ek1 < 0
    flag2_1_2 = false;
else
    flag2_1_2 = true;
    theta2_1 = atan2(e1, pz) - atan2(k1, (ek1)^0.5); %theta1_1  
    theta2_2 = atan2(e1, pz) - atan2(k1, -(ek1)^0.5);
end

if ek2 < 0
    flag2_3_4 = false;
else
    flag2_3_4 = true;
    theta2_3 = atan2(e2, pz) - atan2(k2, (ek2)^0.5); %theta1_2
    theta2_4 = atan2(e2, pz) - atan2(k2, -(ek2)^0.5);
end

if flag2_1_2==false && flag2_3_4==false
    disp('Out of workspace, Unable to cal theta2');
    flag = false;
    ik_all = zeros(1,6);
    return; 
end
%% theta3有4个解 
% theta3+theta2  = theta23 = atan2( -(pxc1 + pys1 - a2c2 ), -(pz + a2s2))
if flag2_1_2
    h1 =  -( px*cos(theta1_1) + py*sin(theta1_1) - a2*cos(theta2_1) );%theta1_1 theta2_1
    t1 = -( pz + a2*sin(theta2_1) );
    h2 =  -( px*cos(theta1_1) + py*sin(theta1_1) - a2*cos(theta2_2) );%theta1_1 theta2_2
    t2 = -( pz + a2*sin(theta2_2) );
    
    theta23_1 = atan2(h1,t1); 
    theta23_2 = atan2(h2,t2);     
end
    
if flag2_3_4
    h3 =  -( px*cos(theta1_2) + py*sin(theta1_2) - a2*cos(theta2_3) );%theta1_2 theta2_3
    t3 = -( pz + a2*sin(theta2_3) );
    h4 =  -( px*cos(theta1_2) + py*sin(theta1_2) - a2*cos(theta2_4) );%theta1_2 theta2_4
    t4 = -( pz + a2*sin(theta2_4) );
    
    theta23_3 = atan2(h3,t3); 
    theta23_4 = atan2(h4,t4);     
end 

% theta3
if flag2_1_2
    theta3_1 = theta23_1 -  theta2_1;%theta1_1 theta2_1
    theta3_2 = theta23_2 -  theta2_2;%theta1_1 theta2_2
    theta3_1 = joint_limit(theta3_1);
    theta3_2 = joint_limit(theta3_2);  
end 

if flag2_3_4
    theta3_3 = theta23_3 -  theta2_3;%theta1_2 theta2_3
    theta3_4 = theta23_4 -  theta2_4;%theta1_2 theta2_4
    theta3_3 = joint_limit(theta3_3);
    theta3_4 = joint_limit(theta3_4);  
end 

%% theta5 theta4 
%T23 = T36_d(2,3) = cos(theta5)
if flag2_1_2
    T23_1 = ax*(-sin(theta2_1)*cos(theta1_1)*cos(theta3_1) - sin(theta3_1)*cos(theta1_1)*cos(theta2_1))... 
        + ay*(-sin(theta1_1)*sin(theta2_1)*cos(theta3_1) - sin(theta1_1)*sin(theta3_1)*cos(theta2_1))...
        + az*(sin(theta2_1)*sin(theta3_1) - cos(theta2_1)*cos(theta3_1));% theta1_1 theta2_1 theta3_1 
    T23_2 = ax*(-sin(theta2_2)*cos(theta1_1)*cos(theta3_2) - sin(theta3_2)*cos(theta1_1)*cos(theta2_2))... 
        + ay*(-sin(theta1_1)*sin(theta2_2)*cos(theta3_2) - sin(theta1_1)*sin(theta3_2)*cos(theta2_2))...
        + az*(sin(theta2_2)*sin(theta3_2) - cos(theta2_2)*cos(theta3_2));% theta1_1 theta2_2 theta3_2
end
if flag2_3_4
    T23_3 = ax*(-sin(theta2_3)*cos(theta1_2)*cos(theta3_3) - sin(theta3_3)*cos(theta1_2)*cos(theta2_3))... 
        + ay*(-sin(theta1_2)*sin(theta2_3)*cos(theta3_3) - sin(theta1_2)*sin(theta3_3)*cos(theta2_3))...
        + az*(sin(theta2_3)*sin(theta3_3) - cos(theta2_3)*cos(theta3_3));% theta1_2 theta2_3 theta3_3
    T23_4 = ax*(-sin(theta2_4)*cos(theta1_2)*cos(theta3_4) - sin(theta3_4)*cos(theta1_2)*cos(theta2_4))... 
        + ay*(-sin(theta1_2)*sin(theta2_4)*cos(theta3_4) - sin(theta1_2)*sin(theta3_4)*cos(theta2_4))...
        + az*(sin(theta2_4)*sin(theta3_4) - cos(theta2_4)*cos(theta3_4));% theta1_2 theta2_4 theta3_4
end

% 
if flag2_1_2
    if T23_1 >0 && 1-abs(T23_1)<eps %theta5=0 奇异
        theta5_1 = 0;
        theta5_5 = -theta5_1;
        theta4_1 = 0;% 或当前值
        theta4_5 = pi;
    elseif  T23_1 <0 && 1-abs(T23_1)<eps
        theta5_1 = pi;
        theta5_5 = -theta5_1;
        theta4_1 = 0;% 或当前值
        theta4_5 = pi;
    else
        theta5_1 = acos(T23_1);
        theta5_5 = -theta5_1;

       % T33 = T36_d(3,3);T13 = T36_d(1, 3)
        T33_1 =  -ax*sin(theta1_1) + ay*cos(theta1_1);% theta1_1 theta2_1 theta3_1
        T13_1 =  ax*(-sin(theta2_1)*sin(theta3_1)*cos(theta1_1) + cos(theta1_1)*cos(theta2_1)*cos(theta3_1))... 
                +ay*(-sin(theta1_1)*sin(theta2_1)*sin(theta3_1)+ sin(theta1_1)*cos(theta2_1)*cos(theta3_1))...
                +az*(-sin(theta2_1)*cos(theta3_1) - sin(theta3_1)*cos(theta2_1)); 
        theta4_1 = atan2( T33_1/sin(theta5_1), -T13_1/sin(theta5_1) );
        theta4_5 = atan2( T33_1/sin(theta5_5), -T13_1/sin(theta5_5) ); 
    end
    
     if T23_2 >0 && 1-abs(T23_2)<eps %theta5=0
        theta5_2 = 0;
        theta5_6 = -theta5_2;
        theta4_2 = 0;% 或当前值
        theta4_6 = pi;
    elseif  T23_2 <0 && 1-abs(T23_2)<eps
        theta5_2 = pi;
        theta5_6 = -theta5_2;
        theta4_2 = 0;% 或当前值 
        theta4_6 = pi;
    else
        theta5_2 = acos(T23_2);
        theta5_6 = -theta5_2;    

        T33_2 =  -ax*sin(theta1_1) + ay*cos(theta1_1);% theta1_1 theta2_2 theta3_2
        T13_2 =  ax*(-sin(theta2_2)*sin(theta3_2)*cos(theta1_1) + cos(theta1_1)*cos(theta2_2)*cos(theta3_2))... 
                +ay*(-sin(theta1_1)*sin(theta2_2)*sin(theta3_2)+ sin(theta1_1)*cos(theta2_2)*cos(theta3_2))...
                +az*(-sin(theta2_2)*cos(theta3_2) - sin(theta3_2)*cos(theta2_2));  
        theta4_2 = atan2( T33_2/sin(theta5_2), -T13_2/sin(theta5_2) );
        theta4_6 = atan2( T33_2/sin(theta5_6), -T13_2/sin(theta5_6) );
    end    
end

if flag2_3_4
    if T23_3 >0 && 1-abs(T23_3)<eps  %theta5=0
        theta5_3 = 0;
        theta5_7 = -theta5_3;
        theta4_3 = 0;% 或当前值
        theta4_7 = pi;
    elseif  T23_3 <0 && 1-abs(T23_3)<eps
        theta5_3 = pi;
        theta5_7 = -theta5_3;
        theta4_3 = 0;% 或当前值 
        theta4_7 = pi;
    else
        theta5_3 = acos(T23_3);
        theta5_7 = -theta5_3;    

        T33_3 =  -ax*sin(theta1_2) + ay*cos(theta1_2);% theta1_2 theta2_3 theta3_3
        T13_3 =  ax*(-sin(theta2_3)*sin(theta3_3)*cos(theta1_2) + cos(theta1_2)*cos(theta2_3)*cos(theta3_3))... 
            +ay*(-sin(theta1_2)*sin(theta2_3)*sin(theta3_3)+ sin(theta1_2)*cos(theta2_3)*cos(theta3_3))...
            +az*(-sin(theta2_3)*cos(theta3_3) - sin(theta3_3)*cos(theta2_3));  
        theta4_3 = atan2( T33_3/sin(theta5_3), -T13_3/sin(theta5_3) ); 
        theta4_7 = atan2( T33_3/sin(theta5_7), -T13_3/sin(theta5_7) ); 
    end

    if T23_4 >0 && 1-abs(T23_4)<eps %theta5=0
        theta5_4 = 0;
        theta5_8 = -theta5_4;
        theta4_4 = 0;% 或当前值
        theta4_8 = pi;    
    elseif  T23_4 <0 && 1-abs(T23_4)<eps
        theta5_4 = pi;
        theta5_8 = -theta5_4;
        theta4_4 = 0;% 或当前值 
        theta4_8 = pi;
    else
        theta5_4 = acos(T23_4);
        theta5_8 = -theta5_4;  

        T33_4 =  -ax*sin(theta1_2) + ay*cos(theta1_2);% theta1_2 theta2_4 theta3_4
        T13_4 =  ax*(-sin(theta2_4)*sin(theta3_4)*cos(theta1_2) + cos(theta1_2)*cos(theta2_4)*cos(theta3_4))... 
                +ay*(-sin(theta1_2)*sin(theta2_4)*sin(theta3_4)+ sin(theta1_2)*cos(theta2_4)*cos(theta3_4))...
                +az*(-sin(theta2_4)*cos(theta3_4) - sin(theta3_4)*cos(theta2_4)); 
        theta4_4 = atan2( T33_4/sin(theta5_4), -T13_4/sin(theta5_4) );
        theta4_8 = atan2( T33_4/sin(theta5_8), -T13_4/sin(theta5_8) );
    end
end

%% theta6
if flag2_1_2 
    % theta1_1 theta2_1 theta3_1  theta4_1 theta5_1
    if abs(sin(theta5_1)) > eps %sin(theta5_1) ~= 0
        % T22 = T36_d(2,2);   T21 = T36_d(2,1);
        T22_1 = ox*(-sin(theta2_1)*cos(theta1_1)*cos(theta3_1) - sin(theta3_1)*cos(theta1_1)*cos(theta2_1))...
           +oy*(-sin(theta1_1)*sin(theta2_1)*cos(theta3_1) - sin(theta1_1)*sin(theta3_1)*cos(theta2_1))... 
          +oz*(sin(theta2_1)*sin(theta3_1) - cos(theta2_1)*cos(theta3_1));  
        T21_1 = nx*(-sin(theta2_1)*cos(theta1_1)*cos(theta3_1) - sin(theta3_1)*cos(theta1_1)*cos(theta2_1))...
                        + ny*(-sin(theta1_1)*sin(theta2_1)*cos(theta3_1) - sin(theta1_1)*sin(theta3_1)*cos(theta2_1))...
                        + nz*(sin(theta2_1)*sin(theta3_1) - cos(theta2_1)*cos(theta3_1));
        theta6_1 = atan2(-T22_1/sin(theta5_1), T21_1/sin(theta5_1));
        theta6_5 = atan2(-T22_1/sin(theta5_5), T21_1/sin(theta5_5));
    else 
        %T31=T36_d(3,1); T11=T36_d(1,1)
        T31_1 = -nx*sin(theta1_1) + ny*cos(theta1_1);
        T11_1 = nx*(-sin(theta2_1)*sin(theta3_1)*cos(theta1_1) + cos(theta1_1)*cos(theta2_1)*cos(theta3_1))... 
            + ny*(-sin(theta1_1)*sin(theta2_1)*sin(theta3_1) + sin(theta1_1)*cos(theta2_1)*cos(theta3_1))... 
            + nz*(-sin(theta2_1)*cos(theta3_1) - sin(theta3_1)*cos(theta2_1));
        sj6_1 = -( cos(theta4_1)*T31_1 + sin(theta4_1)*T11_1);
        cj6_1 = ( cos(theta4_1)*T11_1 - sin(theta4_1)*T31_1);
        theta6_1 = atan2(sj6_1, cj6_1);
        if theta6_1<0
            theta6_5 = pi + theta6_1;
        else
            theta6_5 = theta6_1- pi;
        end     
    end

    % theta1_1 theta2_2 theta3_2  theta4_2 theta5_2
    if abs(sin(theta5_2)) > eps% sin(theta5_2)~=0
        % T22 = T36_d(2,2);   T21 = T36_d(2,1);
        T22_2 = ox*(-sin(theta2_2)*cos(theta1_1)*cos(theta3_2) - sin(theta3_2)*cos(theta1_1)*cos(theta2_2))...
           +oy*(-sin(theta1_1)*sin(theta2_2)*cos(theta3_2) - sin(theta1_1)*sin(theta3_2)*cos(theta2_2))... 
          +oz*(sin(theta2_2)*sin(theta3_2) - cos(theta2_2)*cos(theta3_2));  
        T21_2 = nx*(-sin(theta2_2)*cos(theta1_1)*cos(theta3_2) - sin(theta3_2)*cos(theta1_1)*cos(theta2_2))...
                + ny*(-sin(theta1_1)*sin(theta2_2)*cos(theta3_2) - sin(theta1_1)*sin(theta3_2)*cos(theta2_2))...
                + nz*(sin(theta2_2)*sin(theta3_2) - cos(theta2_2)*cos(theta3_2)); 
        theta6_2 = atan2(-T22_2/sin(theta5_2), T21_2/sin(theta5_2));
        theta6_6 = atan2(-T22_2/sin(theta5_6), T21_2/sin(theta5_6));
    else 
        %T31=T36_d(3,1); T11=T36_d(1,1) 
        T31_2 = -nx*sin(theta1_1) + ny*cos(theta1_1);
        T11_2 = nx*(-sin(theta2_2)*sin(theta3_2)*cos(theta1_1) + cos(theta1_1)*cos(theta2_2)*cos(theta3_2))... 
            + ny*(-sin(theta1_1)*sin(theta2_2)*sin(theta3_2) + sin(theta1_1)*cos(theta2_2)*cos(theta3_2))... 
            + nz*(-sin(theta2_2)*cos(theta3_2) - sin(theta3_2)*cos(theta2_2));    
        sj6_2 = -( cos(theta4_2)*T31_2 + sin(theta4_2)*T11_2);
        cj6_2 = ( cos(theta4_2)*T11_2 - sin(theta4_2)*T31_2);
        theta6_2 = atan2(sj6_2, cj6_2); 
        if theta6_2<0
            theta6_6 = pi + theta6_2;
        else
            theta6_6 =  theta6_2- pi;
        end    
    end
end

if flag2_3_4
    % theta1_2 theta2_3 theta3_3   theta4_3 theta5_3
    if abs(sin(theta5_3)) > eps %sin(theta5_3)~=0
        % T22 = T36_d(2,2);   T21 = T36_d(2,1);
        T22_3 = ox*(-sin(theta2_3)*cos(theta1_2)*cos(theta3_3) - sin(theta3_3)*cos(theta1_2)*cos(theta2_3))...
           +oy*(-sin(theta1_2)*sin(theta2_3)*cos(theta3_3) - sin(theta1_2)*sin(theta3_3)*cos(theta2_3))... 
          +oz*(sin(theta2_3)*sin(theta3_3) - cos(theta2_3)*cos(theta3_3));  
        T21_3 = nx*(-sin(theta2_3)*cos(theta1_2)*cos(theta3_3) - sin(theta3_3)*cos(theta1_2)*cos(theta2_3))...
                + ny*(-sin(theta1_2)*sin(theta2_3)*cos(theta3_3) - sin(theta1_2)*sin(theta3_3)*cos(theta2_3))...
                + nz*(sin(theta2_3)*sin(theta3_3) - cos(theta2_3)*cos(theta3_3)); 
        theta6_3 = atan2(-T22_3/sin(theta5_3), T21_3/sin(theta5_3));
        theta6_7 = atan2(-T22_3/sin(theta5_7), T21_3/sin(theta5_7));
    else 
        %T31=T36_d(3,1); T11=T36_d(1,1)
        T31_3 = -nx*sin(theta1_2) + ny*cos(theta1_2);
        T11_3 = nx*(-sin(theta2_3)*sin(theta3_3)*cos(theta1_2) + cos(theta1_2)*cos(theta2_3)*cos(theta3_3))... 
            + ny*(-sin(theta1_2)*sin(theta2_3)*sin(theta3_3) + sin(theta1_2)*cos(theta2_3)*cos(theta3_3))... 
            + nz*(-sin(theta2_3)*cos(theta3_3) - sin(theta3_3)*cos(theta2_3));   
        sj6_3 = -( cos(theta4_3)*T31_3 + sin(theta4_3)*T11_3);
        cj6_3 = ( cos(theta4_3)*T11_3 - sin(theta4_3)*T31_3);
        theta6_3 = atan2(sj6_3, cj6_3);  
        if theta6_3<0
            theta6_7 = pi + theta6_3;
        else
            theta6_7 = theta6_3- pi;
        end     
    end
    % theta1_2 theta2_4 theta3_4   theta4_4 theta5_4
     if abs(sin(theta5_4)) > eps %sin(theta5_4)~=0
        % T22 = T36_d(2,2);   T21 = T36_d(2,1);
        T22_4 = ox*(-sin(theta2_4)*cos(theta1_2)*cos(theta3_4) - sin(theta3_4)*cos(theta1_2)*cos(theta2_4))...
           +oy*(-sin(theta1_2)*sin(theta2_4)*cos(theta3_4) - sin(theta1_2)*sin(theta3_4)*cos(theta2_4))... 
          +oz*(sin(theta2_4)*sin(theta3_4) - cos(theta2_4)*cos(theta3_4));  
        T21_4 = nx*(-sin(theta2_4)*cos(theta1_2)*cos(theta3_4) - sin(theta3_4)*cos(theta1_2)*cos(theta2_4))...
               + ny*(-sin(theta1_2)*sin(theta2_4)*cos(theta3_4) - sin(theta1_2)*sin(theta3_4)*cos(theta2_4))...
               + nz*(sin(theta2_4)*sin(theta3_4) - cos(theta2_4)*cos(theta3_4));    
        theta6_4 = atan2(-T22_4/sin(theta5_4), T21_4/sin(theta5_4)); 
        theta6_8 = atan2(-T22_4/sin(theta5_8), T21_4/sin(theta5_8));
    else 
        %T31=T36_d(3,1); T11=T36_d(1,1)
        T31_4 = -nx*sin(theta1_2) + ny*cos(theta1_2);
        T11_4 = nx*(-sin(theta2_4)*sin(theta3_4)*cos(theta1_2) + cos(theta1_2)*cos(theta2_4)*cos(theta3_4))... 
            + ny*(-sin(theta1_2)*sin(theta2_4)*sin(theta3_4) + sin(theta1_2)*cos(theta2_4)*cos(theta3_4))... 
            + nz*(-sin(theta2_4)*cos(theta3_4) - sin(theta3_4)*cos(theta2_4));     
        sj6_4 = -( cos(theta4_4)*T31_4 + sin(theta4_4)*T11_4);
        cj6_4 = ( cos(theta4_4)*T11_4 - sin(theta4_4)*T31_4);
        theta6_4 = atan2(sj6_4, cj6_4);   
        if theta6_4<0
            theta6_8 = pi + theta6_4;
        else
            theta6_8 = theta6_4- pi;
        end    
    end       
end          
%% ik_all
% num
% if flag2_1_2 && flag2_3_4
%     ik_num = 8
% else
%     ik_num = 4
% end
% ik_all = zeros(ik_num,6);

if flag2_1_2 && flag2_3_4
    ik_all_init = [theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1;
            theta1_1 theta2_2 theta3_2 theta4_2 theta5_2 theta6_2;
            theta1_2 theta2_3 theta3_3 theta4_3 theta5_3 theta6_3;
            theta1_2 theta2_4 theta3_4 theta4_4 theta5_4 theta6_4;

            theta1_1 theta2_1 theta3_1 theta4_5 theta5_5 theta6_5;
            theta1_1 theta2_2 theta3_2 theta4_6 theta5_6 theta6_6;
            theta1_2 theta2_3 theta3_3 theta4_7 theta5_7 theta6_7;
            theta1_2 theta2_4 theta3_4 theta4_8 theta5_8 theta6_8;];
        
elseif  flag2_1_2   
    ik_all_init = [theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1;
            theta1_1 theta2_2 theta3_2 theta4_2 theta5_2 theta6_2;
            theta1_1 theta2_1 theta3_1 theta4_5 theta5_5 theta6_5;
            theta1_1 theta2_2 theta3_2 theta4_6 theta5_6 theta6_6;];    
else 
    ik_all_init = [
            theta1_2 theta2_3 theta3_3 theta4_3 theta5_3 theta6_3;
            theta1_2 theta2_4 theta3_4 theta4_4 theta5_4 theta6_4;
            theta1_2 theta2_3 theta3_3 theta4_7 theta5_7 theta6_7;
            theta1_2 theta2_4 theta3_4 theta4_8 theta5_8 theta6_8;];     
end
    
%% real
if flag2_1_2 && flag2_3_4
Offset = [offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;];
else
 Offset = [offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;
    offset1, offset2,offset3,offset4,offset5,offset6;];   
end
flag = true;
ik_all = ik_all_init - Offset;
    
end
