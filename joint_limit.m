function res = joint_limit(angle)
%limit angle to [-pi,pi]
%   此处显示详细说明
if angle>pi
    res = angle - 2*pi;
elseif angle<-pi
    res = angle + 2*pi;
else
    res = angle;
end

