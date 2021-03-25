close all;
%二连杆动力学仿真
global ulink G I1 I2
G=9.7;
%连杆参数设置
ulink.m1=4;
ulink.m2=2;
ulink.l1=10;
ulink.l2=20;
%角速度
ulink.w1=[0;0;0];%角速度标量
ulink.w2=[0;0;0];%标量
%角加速度
% ulink.dw1=[0;0;0];
% ulink.dw2=[0;0;0];
%角度
ulink.q1=90/180*pi;%相对于o0
ulink.q2=0/180*pi;%相对于o1
%惯性张量
I1  =  5*eye(3);  I2 = 5*eye(3);  % inertias about cms
%%
% flag=input('是否更新计算公式？(y/n):','s');
% if strcmp(flag,'y')
%    get_params; 
% end
%控制参数
dt=0.02;
tspan=0:dt:5;
%轨迹规划
trace_y=0+5*sin(2*pi/5*tspan);
trace_z=25+5*cos(2*pi/5*tspan);
%运动学解算
%%
vector_update();
%初始状态
z0=[ ulink.q1 ulink.w1(1) ulink.q2 ulink.w2(1)];
%ode 解算
options=odeset('abstol',1e-9,'reltol',1e-9);
[t,z]=ode113('rhs',tspan,z0,options);
figure(1)
handle=showobject;
for i=1:1:length(tspan)
%     yd=trace_y(i);
%     zd=trace_z(i);
%  inverse_kinematics(yd,zd);%逆运动学关节解算
    ulink.q1=z(i,1);
    ulink.q2=z(i,3);

    ulink.w1=[z(i,2);0;0];
    ulink.w2=[z(i,4);0;0];
    vector_update();
    [PE,KE]=energy();

    %绘图
    subplot(222)
    title('动能+势能');
    %校验 动能加势能守恒
    plot(i,PE+KE,'.r')
    hold on
    plot(i,PE,'.B',i,KE,'.k');
    hold on
    subplot(221)
    axis([-40,40,-40,40,-50,50])
    pause(0.01);
    delete(handle)
    handle=showobject;
    subplot(223)
    title('角度');
    plot(i,z(i,1),'.r')
    hold on
    plot(i,z(i,3),'.k')
    hold on
    subplot(224)
    title('角速度');
    plot(i,z(i,2),'.r')
    hold on
    plot(i,z(i,4),'.k')
    hold on
end
