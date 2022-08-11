function ik_all = TCP_IK(Tbase_tcp,type_num)
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
T6_tcp = trotz(tcp_rpy(3))*troty(tcp_rpy(2))*trotx(tcp_rpy(1))*transl(0,0,l_tcp);
Tbase_0 = transl(0,0,l_base);
T06 = Tbase_0^(-1) * Tbase_tcp* T6_tcp^(-1);
ik_all = IK_MDH(T06, type_num);

end
