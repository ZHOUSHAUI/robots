function zdot=rhs(tspan,z0) 
global ulink I1 I2 
global G 
g=G; 
 m1=ulink.m1;m2=ulink.m2; 
l1=ulink.l1;l2=ulink.l2; 
q1=z0(1);q2=z0(3); 
w1=z0(2);w2=z0(4); 
w1_x=w1; w2_x=w2;
I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);
I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);
I1_31=I1(3,1);I1_12=I1(3,2);I1_13=I1(3,3);
I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);
I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);
I2_31=I2(3,1);I2_12=I2(3,2);I2_13=I2(3,3);

KP=4000;
KD=1000;
T1=(KP*(pi-q1)+KD*(0-w1));
KP=2000;
KD=500;
T2=(KP*(pi/2-q2)+KD*(0-w2));
% T1=0;T2=100;

m11=I1_11 + I2_11 + (m1*abs(l1)^2)/4 + m2*abs(l1)^2 + (m2*abs(l2)^2)/4 + (l2*m2*abs(l1)^2*cos(q2))/(2*l1) + (l1*m2*abs(l2)^2*cos(q2))/(2*l2); 
m12=(4*I2_11*l1*l2 + l1^2*m2*abs(l2)^2*cos(q2) + l2^2*m2*abs(l1)^2*cos(q2) + l1*l2*m2*abs(l2)^2)/(4*l1*l2); 
m21=(4*I2_11*l1*l2 + l1^2*m2*abs(l2)^2*cos(q2) + l2^2*m2*abs(l1)^2*cos(q2) + l1*l2*m2*abs(l2)^2)/(4*l1*l2); 
m22=I2_11 + (m2*abs(l2)^2)/4; 
rhs1=- T1 - g*m2*(conj(l1)*sin(q1) + (sin(q1 + q2)*conj(l2))/2) - (g*m1*conj(l1)*sin(q1))/2 - (m2*w2_x*sin(q2)*(l1^2*abs(l2)^2 + l2^2*abs(l1)^2)*(2*w1_x + w2_x))/(4*l1*l2); 
rhs2=(l1*m2*w1_x^2*conj(l2)*sin(q2))/4 - (g*m2*sin(q1 + q2)*conj(l2))/2 - T2 + (l2*m2*w1_x^2*conj(l1)*sin(q2))/4 + (l1*m2*w1_x*w2_x*conj(l2)*sin(q2))/4 + (l2*m2*w1_x*w2_x*conj(l1)*sin(q2))/4 - (m2*w1_x*w2_x*sin(q2)*(l1^2*abs(l2)^2 + l2^2*abs(l1)^2))/(4*l1*l2); 
M=[m11,m12;m21,m22];
RHS=[rhs1;rhs2]; 

X=-M\RHS ;
dw1=X(1);dw2=X(2); 
zdot=[w1_x dw1 w2_x dw2]';
end
