function  [PE,KE]=energy() 
%动能势能计算 
global ulink I1 I2 
global G 
g=G; 
l1=ulink.l1;l2=ulink.l2; 
m1=ulink.m1;m2=ulink.m2; 
q1=ulink.q1;q2=ulink.q2;  
w1=ulink.w1;w2=ulink.w2;  
w1_x=w1(1); w1_y=w1(2);w1_z=w1(3);w2_x=w2(1) ;w2_y=w2(2);w2_z=w2(3);
I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);
I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);
I1_31=I1(3,1);I1_12=I1(3,2);I1_13=I1(3,3);
I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);
I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);
I2_31=I2(3,1);I2_12=I2(3,2);I2_13=I2(3,3);
KE=(I1_11*w1_x^2)/2 + (I2_11*w1_x^2)/2 + (I2_11*w2_x^2)/2 + (l1^2*m1*w1_x^2)/8 + (l1^2*m2*w1_x^2)/2 + (l2^2*m2*w1_x^2)/8 + (l2^2*m2*w2_x^2)/8 + I2_11*w1_x*w2_x + (l2^2*m2*w1_x*w2_x)/4 + (l1*l2*m2*w1_x^2*cos(q2))/2 + (l1*l2*m2*w1_x*w2_x*cos(q2))/2;
PE=g*m2*((l2*cos(q1 + q2))/2 + l1*cos(q1)) + (g*l1*m1*cos(q1))/2;
end
