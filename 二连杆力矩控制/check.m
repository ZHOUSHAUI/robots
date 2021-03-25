global ulink G I1 I2
G=10;
g=G;
%连杆参数设置
ulink.m1=1;
ulink.m2=1;
ulink.l1=10;
ulink.l2=20;
%角速度
ulink.w1=[1;0;0];%角速度标量
ulink.w2=[0;0;0];%标量
w1_x=ulink.w1(1);
w2_x=ulink.w2(1);
%角加速度
ulink.dw1=[0;0;0];
ulink.dw2=[0;0;0];
%角度
ulink.q1=10/180*pi;%相对于o0
ulink.q2=170/180*pi;%相对于o1
I1  =  5*eye(3);  I2 = 5*eye(3);  % inertias about cms
I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);
I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);
I1_31=I1(3,1);I1_12=I1(3,2);I1_13=I1(3,3);
I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);
I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);
I2_31=I2(3,1);I2_12=I2(3,2);I2_13=I2(3,3);

KE=(I1_11*w1_x^2)/2 + (I2_11*w1_x^2)/2 + (I2_11*w2_x^2)/2 + (25*ulink.m1*w1_x^2)/2 + 100*ulink.m2*w1_x^2 + 50*ulink.m2*w2_x^2 + 100*ulink.m2*w1_x^2*cos(ulink.q2) + I2_11*w1_x*w2_x + 100*ulink.m2*w1_x*w2_x + 100*ulink.m2*w1_x*w2_x*cos(ulink.q2)
PE=5*g*ulink.m1*cos(ulink.q1) + g*ulink.m2*(10*cos(ulink.q1 + ulink.q2) + 10*cos(ulink.q1))