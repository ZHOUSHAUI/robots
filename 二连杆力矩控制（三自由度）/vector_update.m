function vector_update() 
global ulink
q1=ulink.q1;q2=ulink.q2;q3=ulink.q3; 
l1=ulink.l1;l2=ulink.l2; 
ulink.p_b=[l1*sin(q1)*sin(q2);l1*cos(q1)*sin(q2);l1*cos(q2)];
ulink.p_a=[0;0;0];
ulink.p_c=[l2*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + l1*sin(q1)*sin(q2);l2*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + l1*cos(q1)*sin(q2);l2*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + l1*cos(q2)];
ulink.p_e=[(l1*sin(q1)*sin(q2))/2;(l1*cos(q1)*sin(q2))/2;(l1*cos(q2))/2];
ulink.p_f=[(l2*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/2 + l1*sin(q1)*sin(q2);(l2*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/2 + l1*cos(q1)*sin(q2);(l2*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/2 + l1*cos(q2)];
end 
