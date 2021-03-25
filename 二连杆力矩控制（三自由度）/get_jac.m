function jac=get_jac() 
global ulink 
q1=ulink.q1; q2=ulink.q2; 
l1=ulink.l1;l2=ulink.l2; 
jac1=[-cos(q1)*(l2*sin(q2 + q3) + l1*sin(q2)),-sin(q1)*(l2*cos(q2 + q3) + l1*cos(q2)),0,0,0,0] ;
jac2=[sin(q1)*(l2*sin(q2 + q3) + l1*sin(q2)),-cos(q1)*(l2*cos(q2 + q3) + l1*cos(q2)),0,0,0,0] ;
jac3=[0,l2*sin(q2 + q3) + l1*sin(q2),0,0,0,0] ;
jac4=[0,cos(q1),0,0,0,0] ;
jac5=[0,-sin(q1),0,0,0,0] ;
jac6=[1,0,0,0,0,0] ;
jac=[jac1;jac2;jac3;jac4;jac5;jac6] ;
end 
