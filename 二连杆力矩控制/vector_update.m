function vector_update() 
global ulink
q1=ulink.q1;q2=ulink.q2; 
l1=ulink.l1;l2=ulink.l2; 
ulink.p_b=[0;-l1*sin(q1);l1*cos(q1)];
ulink.p_a=[0;0;0];
ulink.p_c=[0;- l1*sin(q1) - l2*cos(q1)*sin(q2) - l2*cos(q2)*sin(q1);l1*cos(q1) + l2*cos(q1)*cos(q2) - l2*sin(q1)*sin(q2)];
ulink.p_e=[0;-(l1*sin(q1))/2;(l1*cos(q1))/2];
ulink.p_f=[0;- l1*sin(q1) - (l2*cos(q1)*sin(q2))/2 - (l2*cos(q2)*sin(q1))/2;l1*cos(q1) + (l2*cos(q1)*cos(q2))/2 - (l2*sin(q1)*sin(q2))/2];
end 
