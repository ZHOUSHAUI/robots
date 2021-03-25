function T=motor_control_ID3(ref_q,p_r,v_r)
persistent motor_flag e_p_last e_v_last e_i_last   i_r 
if isempty(motor_flag)
    motor_flag=1;
    e_p_last=0;
    e_v_last=0;
    e_i_last=0;
    i_r=0;
end
%��λ����
ref_q=ref_q/(2*pi);  
p_r=p_r/(2*pi);  
v_r=v_r*60/(2*pi);
%���ϵͳ���� RE40
motor.U=48;          %���ѹ V
motor.R=24.4;       %������ ��
motor.L=6.41*10^-3; %����� H
motor.Kt=266*10^-3; %ת�س��� Nm/A
motor.Ke=1/35.9 ;     %ת�ٳ��� v/rpm
motor.Ji=120*10^-7 ; %ת�ӹ��� kgm
%������
radio=10;
%�ⲿ����
Jr=1  ;%ת������ 1/radio^2
%��Ч����
motor.Ja=motor.Ji+Jr/radio^2;
%��������
dt=0.0002; 
p_pid.p=1000;
p_pid.i=0;
p_pid.d=100;
%�ٶȻ�
v_pid.p=1200;
v_pid.i=0;
v_pid.d=200;
%������
i_pid.p=2500;
i_pid.i=50;
i_pid.d=1000;
%%
%��������
%λ�û�
PID=p_pid;
e_p=ref_q-p_r;%λ�����
v_t=PID.p*e_p+PID.i*(e_p+e_p_last)+PID.d*(e_p-e_p_last);
e_p_last=e_p;
%�ٶȻ�
PID=v_pid;
e_v=v_t-v_r;
i_t=i_r+PID.p*e_v+PID.i*(e_v+e_v_last)+PID.d*(e_v-e_v_last);
e_v_last=e_v;
%������
PID=i_pid;
e_i=i_t-i_r;
u_r=PID.p*e_i+PID.i*(e_i+e_i_last)+PID.d*(e_i-e_i_last);
e_i_last=e_i;
%���ģ��
di=1/motor.L*u_r-motor.R/motor.L*i_r-motor.Ke/motor.L*v_r;
dv=motor.Kt/motor.Ja*i_r;
i_r=i_r+di*dt; %�������
% v_r=v_r+dv*dt;
T=i_r*motor.Kt;%�������
end