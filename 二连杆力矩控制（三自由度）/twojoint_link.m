close all;
clear all;
%�����˶���ѧ����
global ulink G I1 I2 
G=10;
%���˲�������
ulink.m1=1;
ulink.m2=1;
ulink.l1=10;
ulink.l2=10;
%���ٶ�
ulink.w1=[0;0;0];%���ٶȱ���
ulink.w2=[0;0;0];%����
ulink.w3=[0;0;0];%����
%�Ǽ��ٶ�
% ulink.dw1=[0;0;0];
% ulink.dw2=[0;0;0];
%�Ƕ�
ulink.q1=0/180*pi;%�����o0
ulink.q2= 22/180*pi;%�����o1
ulink.q3= 32/180*pi;%�����o2
%��������
I1  =  5*eye(3);  I2 = 5*eye(3);  % inertias about cms
%%
% flag=input('�Ƿ���¼��㹫ʽ��(y/n):','s');
% if strcmp(flag,'y')
%    get_params; 
% end
%���Ʋ���
dt=0.02;
tspan=0:dt:5;
%�켣�滮
trace_x=5*sin(2*pi/5*tspan);
trace_y=12.*ones(length(tspan));
trace_z=10+5*cos(2*pi/5*tspan);
%�������
target_p=[0;10;10];
%�˶�ѧ����
%%
vector_update();
%��ʼ״̬
z0=[ulink.q1 ulink.w1(3) ulink.q2 ulink.w2(1) ulink.q3 ulink.w3(1)];
%ode ����
options=odeset('abstol',1e-5,'reltol',1e-5);
flag=input('�Ƿ����ԭʼ��������y/n:','s');
if strcmp(flag,'y')
    load('t.mat');
    load('z.mat');
else
    [t,z]=ode113('rhs',tspan,z0,options);
end
figure(1)
set(gcf,'Position',get(0,'ScreenSize'));
subplot(222)
title('����+����');
hold on
subplot(234)
title('���ٶ�');
hold on
subplot(235)
title('�Ƕ�');
hold on
subplot(236)
title('�������');
axis equal
hold on
subplot(221)
view(30,30);
axis([-20,20,-20,20,0,30])
title('�˶�����');
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
    %��ͼ
    subplot(221)
    pause(0.02);
    delete(handle)
    handle=showobject;
    plot3(ulink.p_c(1),ulink.p_c(2),ulink.p_c(3),'.k');
    hold on
end
%%
subplot(222)
%У�� ���ܼ������غ�
plot(t,PE+KE,'r',t,PE,'B',t,KE,'k')
legend('PE+KE','PE','KE')
hold on
subplot(235)
%�Ƕ�
plot(t,z(:,1),'r',t,z(:,3),'k',t,z(:,5),'b')
legend('q1','q2','q3')
subplot(234)
% ���ٶ�
plot(t,z(:,2),'r',t,z(:,4),'k',t,z(:,6),'b')
legend('w1','w2','w3')
%���Ƹ����������
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