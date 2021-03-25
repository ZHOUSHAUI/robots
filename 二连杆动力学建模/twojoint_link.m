close all;
%�����˶���ѧ����
global ulink G I1 I2
G=9.7;
%���˲�������
ulink.m1=4;
ulink.m2=2;
ulink.l1=10;
ulink.l2=20;
%���ٶ�
ulink.w1=[0;0;0];%���ٶȱ���
ulink.w2=[0;0;0];%����
%�Ǽ��ٶ�
% ulink.dw1=[0;0;0];
% ulink.dw2=[0;0;0];
%�Ƕ�
ulink.q1=90/180*pi;%�����o0
ulink.q2=0/180*pi;%�����o1
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
trace_y=0+5*sin(2*pi/5*tspan);
trace_z=25+5*cos(2*pi/5*tspan);
%�˶�ѧ����
%%
vector_update();
%��ʼ״̬
z0=[ ulink.q1 ulink.w1(1) ulink.q2 ulink.w2(1)];
%ode ����
options=odeset('abstol',1e-9,'reltol',1e-9);
[t,z]=ode113('rhs',tspan,z0,options);
figure(1)
handle=showobject;
for i=1:1:length(tspan)
%     yd=trace_y(i);
%     zd=trace_z(i);
%  inverse_kinematics(yd,zd);%���˶�ѧ�ؽڽ���
    ulink.q1=z(i,1);
    ulink.q2=z(i,3);

    ulink.w1=[z(i,2);0;0];
    ulink.w2=[z(i,4);0;0];
    vector_update();
    [PE,KE]=energy();

    %��ͼ
    subplot(222)
    title('����+����');
    %У�� ���ܼ������غ�
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
    title('�Ƕ�');
    plot(i,z(i,1),'.r')
    hold on
    plot(i,z(i,3),'.k')
    hold on
    subplot(224)
    title('���ٶ�');
    plot(i,z(i,2),'.r')
    hold on
    plot(i,z(i,4),'.k')
    hold on
end
