close all;
clear all;
%二连杆动力学仿真
global ulink G I1 I2 
G=10;
%连杆参数设置
ulink.m1=1;
ulink.m2=1;
ulink.l1=10;
ulink.l2=10;
%角速度
ulink.w1=[0;0;0];%角速度标量
ulink.w2=[0;0;0];%标量
ulink.w3=[0;0;0];%标量
%角加速度
% ulink.dw1=[0;0;0];
% ulink.dw2=[0;0;0];
%角度
ulink.q1=0/180*pi;%相对于o0
ulink.q2= 22/180*pi;%相对于o1
ulink.q3= 32/180*pi;%相对于o2
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
trace_x=5*sin(2*pi/5*tspan);
trace_y=12.*ones(length(tspan));
trace_z=10+5*cos(2*pi/5*tspan);
%单点测试
target_p=[0;10;10];
%运动学解算
%%
vector_update();
%初始状态
z0=[ulink.q1 ulink.w1(3) ulink.q2 ulink.w2(1) ulink.q3 ulink.w3(1)];
%ode 解算
options=odeset('abstol',1e-5,'reltol',1e-5);
flag=input('是否加载原始仿真数据y/n:','s');
if strcmp(flag,'y')
    load('t.mat');
    load('z.mat');
else
    [t,z]=ode113('rhs',tspan,z0,options);
end
figure(1)
set(gcf,'Position',get(0,'ScreenSize'));
subplot(222)
title('动能+势能');
hold on
subplot(234)
title('角速度');
hold on
subplot(235)
title('角度');
hold on
subplot(236)
title('跟踪误差');
axis equal
hold on
subplot(221)
view(30,30);
axis([-20,20,-20,20,0,30])
title('运动仿真');
hold on;
plot3(trace_x,trace_y,trace_z,'r');
hold on
grid on
hold on
plot3(target_p(1),target_p(2),target_p(3),'.r','LineWidth',10);
hold on
handle=showobject;
%%
for i=1:1:length(tspan)
    ulink.q1=z(i,1);
    ulink.q2=z(i,3);
    ulink.q3=z(i,5);
    ulink.w1=[0;0;z(i,2)];
    ulink.w2=[z(i,4);0;0];
    ulink.w3=[z(i,6);0;0];
    vector_update();
    [PE(i),KE(i)]=energy();
    %绘图
    subplot(221)
    pause(0.02);
    delete(handle)
    handle=showobject;
    plot3(ulink.p_c(1),ulink.p_c(2),ulink.p_c(3),'.k');
    hold on
end
%%
subplot(222)
%校验 动能加势能守恒
plot(t,PE+KE,'r',t,PE,'B',t,KE,'k')
legend('PE+KE','PE','KE')
hold on
subplot(235)
%角度
plot(t,z(:,1),'r',t,z(:,3),'k',t,z(:,5),'b')
legend('q1','q2','q3')
subplot(234)
% 角速度
plot(t,z(:,2),'r',t,z(:,4),'k',t,z(:,6),'b')
legend('w1','w2','w3')
%绘制跟踪误差曲线
fellow_e=zeros(length(tspan),3);
for i=1:1:length(tspan)
    target_p=path_plan(i*dt);
    ulink.q1=z(i,1);
    ulink.q2=z(i,3);
    ulink.q3=z(i,5);
    vector_update();
    fellow_e(i,:)=ulink.p_c-target_p;
end
subplot(236)
plot(t,fellow_e(:,1),'r',t,fellow_e(:,2),'b',t,fellow_e(:,3),'k');
legend('ex','ey','ez')
save('t.mat','t');
save('z.mat','z');