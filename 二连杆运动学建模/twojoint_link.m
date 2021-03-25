
close all;
%二连杆动力学仿真
global ulink
%连杆参数设置
ulink.l1=10;
ulink.l2=20;
ulink.w1=0;%角速度标量
ulink.w2=0;%标量
ulink.q1=0/180*pi;%相对于o0
ulink.q2=0/180*pi;%相对于o1
%%
%提取公式
% ques1=input('是否载入或更新公式?(y/n):','s');
% if strcmp(ques1,'y')
%    get_params
% end
%%
%控制参数
dt=0.02;
tspan=0:dt:5;
%轨迹规划
trace_y=15+5*sin(2*pi/5*tspan);
trace_z=10+5*cos(2*pi/5*tspan);
%运动学解算
%%
vector_update();
for i=1:1:length(tspan) 
    yd=trace_y(i);
    zd=trace_z(i);
    yr=ulink.p_c(2);
    zr=ulink.p_c(3);
    q1r=ulink.q1;
    q2r=ulink.q2;
    v_y=(yd-yr)/dt;
    v_z=(zd-zr)/dt;
    vr=[0;v_y;v_z;0;0;0];
    l3=sqrt(yd^2+zd^2);
    ulink.q2=-pi+acos(((ulink.l1)^2+(ulink.l2)^2-l3^2)/(2*ulink.l1*ulink.l2));
    ulink.q1=-pi/2+atan2(zd,yd)+acos(((ulink.l1)^2-(ulink.l2)^2+l3^2)/(2*ulink.l1*(l3)));
    vector_update();
    %校验雅克比
    dq1=(ulink.q1-q1r)/dt;
    dq2=(ulink.q2-q2r)/dt;
    dq=[dq1;dq2];
    jac=get_jac();
    jac_v=[jac(2,1),jac(2,2);jac(3,1),jac(3,2)];
    v=jac_v*dq;%计算z,y方向上的速度
    %绘图
    subplot(121)
    axis([-20,20,0,40])
    handle(1)=plot([ulink.p_a(2),ulink.p_b(2)],[ulink.p_a(3),ulink.p_b(3)],'r');
    %验证
%     ll=sqrt((ulink.p_b(3)-ulink.p_a(3))^2+(ulink.p_b(2)-ulink.p_a(2))^2)
%     l2=sqrt((ulink.p_b(3)-ulink.p_c(3))^2+(ulink.p_b(2)-ulink.p_c(2))^2)
    hold on;
    handle(2)= plot([ulink.p_b(2),ulink.p_c(2)],[ulink.p_b(3),ulink.p_c(3)],'b');
    hold on;
    plot(yd,zd,'xb');
    subplot(122)
    plot(i,v(1),'.r')
    hold on;
    plot(i,vr(2),'xb')
    pause(0.02)
    delete(handle);
end
