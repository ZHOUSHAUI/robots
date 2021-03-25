close all;
clear;
%二连杆动力学仿真
global ulink G I1 I2
G=9.7;
%连杆参数设置
ulink.m1=1;
ulink.m2=1;
ulink.l1=10;
ulink.l2=20;
%角速度
ulink.w1=[0;0;0];%角速度标量
ulink.w2=[0;0;0];%标量
ulink.w3=[0;0;0];%标量
%角加速度
% ulink.dw1=[0;0;0];
% ulink.dw2=[0;0;0];
%角度
ulink.q1=-90/180*pi;%相对于o0
ulink.q2=90/180*pi;%相对于o1
ulink.q3=0/180*pi;%相对于o2

q1=ulink.q1;q2=ulink.q2;q3=ulink.q3; 
l1=ulink.l1;l2=ulink.l2; 
ulink.p_b=[l1*sin(q1)*sin(q2);-l1*cos(q1)*sin(q2);l1*cos(q2)];
ulink.p_a=[0;0;0];
ulink.p_c=[l1*sin(q1)*sin(q2) + l2*cos(q2)*sin(q1)*sin(q3) + l2*cos(q3)*sin(q1)*sin(q2);- l1*cos(q1)*sin(q2) - l2*cos(q1)*cos(q2)*sin(q3) - l2*cos(q1)*cos(q3)*sin(q2);l1*cos(q2) + l2*cos(q2)*cos(q3) - l2*sin(q2)*sin(q3)];
ulink.p_e=[(l1*sin(q1)*sin(q2))/2;-(l1*cos(q1)*sin(q2))/2;(l1*cos(q2))/2];
ulink.p_f=[l1*sin(q1)*sin(q2) + (l2*cos(q2)*sin(q1)*sin(q3))/2 + (l2*cos(q3)*sin(q1)*sin(q2))/2;- l1*cos(q1)*sin(q2) - (l2*cos(q1)*cos(q2)*sin(q3))/2 - (l2*cos(q1)*cos(q3)*sin(q2))/2;l1*cos(q2) + (l2*cos(q2)*cos(q3))/2 - (l2*sin(q2)*sin(q3))/2];

figure(1)
axis([-30,30,-30,30,-30,30,])
grid on
hold on;
handle(1)=plot3([ulink.p_a(1),ulink.p_b(1)],[ulink.p_a(2),ulink.p_b(2)],[ulink.p_a(3),ulink.p_b(3)],'r','LineWidth',5);
hold on;
handle(2)= plot3([ulink.p_b(1),ulink.p_c(1)],[ulink.p_b(2),ulink.p_c(2)],[ulink.p_b(3),ulink.p_c(3)],'b','LineWidth',5);