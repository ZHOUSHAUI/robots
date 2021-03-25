% close all;
% clear;
% 
% global ulink G I1 I2
% G=10;
% g=G;
% %连杆参数设置
% ulink.m1=1;
% ulink.m2=1;
% ulink.l1=10;
% ulink.l2=20;
% %角速度
% ulink.w1=[0;0;0];%角速度标量
% ulink.w2=[0;0;0];%标量
% ulink.w3=[0;0;0];%标量
% 
% w1_z=ulink.w1(3);
% w2_x=ulink.w2(1);
% w3_x=ulink.w3(1);
% 
% 
% %角度
% ulink.q1=0/180*pi;%相对于o0
% ulink.q2=90/180*pi;%相对于o1
% ulink.q3=0/180*pi;%相对于o2
% 
% I1  =  5*eye(3);  I2 = 5*eye(3);  % inertias about cms

% l1=ulink.l1;l2=ulink.l2; 
% m1=ulink.m1;m2=ulink.m2; 
% q1=ulink.q1;q2=ulink.q2; q3=ulink.q3; 
% w1=ulink.w1;w2=ulink.w2; w3=ulink.w3; 
% 
% w1_x=w1(1); w1_y=w1(2);w1_z=w1(3);
% w2_x=w2(1) ;w2_y=w2(2);w2_z=w2(3);
% w3_x=w3(1) ;w3_y=w3(2);w3_z=w3(3);
% 
% I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);
% I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);
% I1_31=I1(3,1);I1_32=I1(3,2);I1_33=I1(3,3);
% I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);
% I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);
% I2_31=I2(3,1);I2_32=I2(3,2);I2_33=I2(3,3);
% 
% KE=(I1_11*w2_x^2)/2 + (I1_22*w1_z^2)/4 + (I1_33*w1_z^2)/4 + (I2_11*w2_x^2)/2 + (I2_11*w3_x^2)/2 + (I2_22*w1_z^2)/4 + (I2_33*w1_z^2)/4 + (l1^2*m1*w2_x^2)/8 + (l1^2*m1*w1_z^2)/16 + (l1^2*m2*w2_x^2)/2 + (l1^2*m2*w1_z^2)/4 + (l2^2*m2*w2_x^2)/8 + (l2^2*m2*w1_z^2)/16 + (l2^2*m2*w3_x^2)/8 + I2_11*w2_x*w3_x - (I1_22*w1_z^2*cos(2*q2))/4 + (I1_33*w1_z^2*cos(2*q2))/4 + (I1_23*w1_z^2*sin(2*q2))/4 + (I1_32*w1_z^2*sin(2*q2))/4 - (I2_22*w1_z^2*cos(2*q2 + 2*q3))/4 + (I2_33*w1_z^2*cos(2*q2 + 2*q3))/4 + (I2_23*w1_z^2*sin(2*q2 + 2*q3))/4 + (I2_32*w1_z^2*sin(2*q2 + 2*q3))/4 + (I2_13*w2_x*w1_z*cos(q2 + q3))/2 + (I2_13*w1_z*w3_x*cos(q2 + q3))/2 + (I2_31*w2_x*w1_z*cos(q2 + q3))/2 + (I2_31*w1_z*w3_x*cos(q2 + q3))/2 + (I2_12*w2_x*w1_z*sin(q2 + q3))/2 + (I2_12*w1_z*w3_x*sin(q2 + q3))/2 + (I2_21*w2_x*w1_z*sin(q2 + q3))/2 + (I2_21*w1_z*w3_x*sin(q2 + q3))/2 + (I1_13*w2_x*w1_z*cos(q2))/2 + (I1_31*w2_x*w1_z*cos(q2))/2 + (I1_12*w2_x*w1_z*sin(q2))/2 + (I1_21*w2_x*w1_z*sin(q2))/2 + (l2^2*m2*w2_x*w3_x)/4 - (l1^2*m1*w1_z^2*cos(2*q2))/16 - (l1^2*m2*w1_z^2*cos(2*q2))/4 - (l2^2*m2*w1_z^2*cos(2*q2 + 2*q3))/16 + (l1*l2*m2*w2_x^2*cos(q3))/2 + (l1*l2*m2*w1_z^2*cos(q3))/4 - (l1*l2*m2*w1_z^2*cos(2*q2 + q3))/4 + (l1*l2*m2*w2_x*w3_x*cos(q3))/2
% PE=g*m2*((l2*cos(q2 + q3))/2 + l1*cos(q2)) + (g*l1*m1*cos(q2))/2

syms aw1 aw2  aw3 w1_x w1_y w1_z w2_x w2_y w2_z w3_x w3_y w3_z real
syms I1 I1_11 I1_12 I1_13  I1_21  I1_22  I1_23  I1_31  I1_32  I1_33 real 
syms I2 I2_11 I2_12 I2_13  I2_21  I2_22  I2_23  I2_31  I2_32  I2_33 real 
syms dw1 dw2 dw3 real
syms l1 l2 real
syms m1 m2 g real
syms q1 q2 q3 real
w1_z=0;
KE=(I1_11*w2_x^2)/2 + (I1_22*w1_z^2)/4 + (I1_33*w1_z^2)/4 + (I2_11*w2_x^2)/2 + (I2_11*w3_x^2)/2 + (I2_22*w1_z^2)/4 + (I2_33*w1_z^2)/4 + (l1^2*m1*w2_x^2)/8 + (l1^2*m1*w1_z^2)/16 + (l1^2*m2*w2_x^2)/2 + (l1^2*m2*w1_z^2)/4 + (l2^2*m2*w2_x^2)/8 + (l2^2*m2*w1_z^2)/16 + (l2^2*m2*w3_x^2)/8 + I2_11*w2_x*w3_x - (I1_22*w1_z^2*cos(2*q2))/4 + (I1_33*w1_z^2*cos(2*q2))/4 + (I1_23*w1_z^2*sin(2*q2))/4 + (I1_32*w1_z^2*sin(2*q2))/4 - (I2_22*w1_z^2*cos(2*q2 + 2*q3))/4 + (I2_33*w1_z^2*cos(2*q2 + 2*q3))/4 + (I2_23*w1_z^2*sin(2*q2 + 2*q3))/4 + (I2_32*w1_z^2*sin(2*q2 + 2*q3))/4 + (I2_13*w2_x*w1_z*cos(q2 + q3))/2 + (I2_13*w1_z*w3_x*cos(q2 + q3))/2 + (I2_31*w2_x*w1_z*cos(q2 + q3))/2 + (I2_31*w1_z*w3_x*cos(q2 + q3))/2 + (I2_12*w2_x*w1_z*sin(q2 + q3))/2 + (I2_12*w1_z*w3_x*sin(q2 + q3))/2 + (I2_21*w2_x*w1_z*sin(q2 + q3))/2 + (I2_21*w1_z*w3_x*sin(q2 + q3))/2 + (I1_13*w2_x*w1_z*cos(q2))/2 + (I1_31*w2_x*w1_z*cos(q2))/2 + (I1_12*w2_x*w1_z*sin(q2))/2 + (I1_21*w2_x*w1_z*sin(q2))/2 + (l2^2*m2*w2_x*w3_x)/4 - (l1^2*m1*w1_z^2*cos(2*q2))/16 - (l1^2*m2*w1_z^2*cos(2*q2))/4 - (l2^2*m2*w1_z^2*cos(2*q2 + 2*q3))/16 + (l1*l2*m2*w2_x^2*cos(q3))/2 + (l1*l2*m2*w1_z^2*cos(q3))/4 - (l1*l2*m2*w1_z^2*cos(2*q2 + q3))/4 + (l1*l2*m2*w2_x*w3_x*cos(q3))/2
PE=g*m2*((l2*cos(q2 + q3))/2 + l1*cos(q2)) + (g*l1*m1*cos(q2))/2;

KE=(I1_11*w2_x^2)/2 + (I2_11*w2_x^2)/2 + (I2_11*w3_x^2)/2 + (l1^2*m1*w2_x^2)/8 + (l1^2*m2*w2_x^2)/2 + (l2^2*m2*w2_x^2)/8 + (l2^2*m2*w3_x^2)/8 + I2_11*w2_x*w3_x + (l2^2*m2*w2_x*w3_x)/4 + (l1*l2*m2*w2_x^2*cos(q3))/2 + (l1*l2*m2*w2_x*w3_x*cos(q3))/2
PE=g*m2*((l2*cos(q2 + q3))/2 + l1*cos(q2)) + (g*l1*m1*cos(q2))/2;