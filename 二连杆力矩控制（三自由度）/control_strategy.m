function [T1,T2,T3]=control_strategy(t,s_v)
%·������
target_p=path_plan(t);
%�̶������
% target_p=[0;10;10];
%�ο����� ���˶�ѧ����
ref_q=inverse_kinematics(target_p);
G=get_G(s_v);
%������ؿ���
T1=motor_control_ID1(ref_q(1),s_v(1),s_v(2))+G(1);
T2=motor_control_ID2(ref_q(2),s_v(3),s_v(4))+G(2);
T3=motor_control_ID3(ref_q(3),s_v(5),s_v(6))+G(3);
end