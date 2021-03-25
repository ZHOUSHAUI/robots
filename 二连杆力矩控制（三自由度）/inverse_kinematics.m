function ref_q=inverse_kinematics(target_p)
global ulink
l1=ulink.l1;
l2=ulink.l2;
xd=target_p(1);
yd=target_p(2);
zd=target_p(3);

ref_q1=atan2(xd,yd);
%转化为y-z平面计算
sida=ref_q1;
R_z=[cos(sida),sin(sida),0;
    -sin(sida),cos(sida),0;
    0,0 ,1]';
target_p_yz=R_z*target_p;
yd=target_p_yz(2);
zd=target_p_yz(3);
l3=sqrt(zd^2+yd^2);
sida1=atan2(zd,yd);
ref_q3=(pi-acos((l1^2+l2^2-l3^2)/(2*l1*l2)));
sida2=acos((l1^2+l3^2-l2^2)/(2*l1*l3));
ref_q2=(pi/2-sida1-sida2);
ref_q=[ref_q1;ref_q2;ref_q3];
% ref_q=[0;0;pi/2];
% q2_ref=acos((x_end^2 + y_end^2-l^2-l^2)/(2*l*l));
% q1_ref=atan(x_end/(-y_end))-acos((l^2+x_end^2+y_end^2-l^2)/(2*l*(x_end^2+y_end^2)^0.5));
end