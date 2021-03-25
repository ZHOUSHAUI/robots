function vector_update() 
global ulink
q1=ulink.q1;q2=ulink.q2; 
ulink.p_a=[0,0,0]' ;
ulink.p_b=[0;-10*sin(q1);10*cos(q1)];
ulink.p_c=[0;- 10*sin(q1) - 20*cos(q1)*sin(q2) - 20*cos(q2)*sin(q1);10*cos(q1) + 20*cos(q1)*cos(q2) - 20*sin(q1)*sin(q2)];
ulink.p_e=[0;-5*sin(q1);5*cos(q1)];
ulink.p_f=[0;- 10*sin(q1) - 10*cos(q1)*sin(q2) - 10*cos(q2)*sin(q1);10*cos(q1) + 10*cos(q1)*cos(q2) - 10*sin(q1)*sin(q2)];
end 
