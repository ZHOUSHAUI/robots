%二连杆仿真公式提取
function get_params
syms l1 l2
%%
%提取公式
%提取位置，杆向量位置表达式
x0=[1,0,0]';
y0=[0,1,0]';
z0=[0,0,1]';
syms q1 q2 real
%在o0中
x1=x0;
y1=0+y0*cos(-q1)-z0*sin(-q1);
z1=0+y0*sin(-q1)+z0*cos(-q1);
%在o1中
x2=x0;
y2=0+y0*cos(-q2)-z0*sin(-q2);
z2=0+y0*sin(-q2)+z0*cos(-q2);
%姿态矩阵
R0_1=[x1,y1,z1];
R1_2=[x2,y2,z2];
R0_2=R0_1*R1_2;
%位置向量 世界坐标系下表示
p_a=[0,0,0]';
p_b=p_a+l1*z1;
r_ab=p_b-p_a;
r_ae=0.5*r_ab;
p_e=p_a+r_ae;
p_c=p_b+l2*R0_1*z2;
r_bc=p_c-p_b;
p_f=p_b+0.5*r_bc;
r_bf=p_f-p_b;
file=fopen('vector_update.m','w');
fprintf(file,'function vector_update() \n');
fprintf(file,'global ulink\n');
fprintf(file,'q1=ulink.q1;q2=ulink.q2; \n');
fprintf(file,'l1=ulink.l1;l2=ulink.l2; \n');
fprintf(file,'ulink.p_b=[%s;%s;%s];\n',[p_b(1),p_b(2),p_b(3)]);
fprintf(file,'ulink.p_a=[%d;%d;%d];\n',[p_a(1),p_a(2),p_a(3)]);
fprintf(file,'ulink.p_c=[%s;%s;%s];\n',[p_c(1),p_c(2),p_c(3)]);
fprintf(file,'ulink.p_e=[%s;%s;%s];\n',[p_e(1),p_e(2),p_e(3)]);
fprintf(file,'ulink.p_f=[%s;%s;%s];\n',[p_f(1),p_f(2),p_f(3)]);
fprintf(file,'end \n');
fclose(file);
%提取雅克比矩阵表达式
%方法一：偏导法
p_end=[p_c(1);p_c(2);p_c(3)];
q=[q1;q2];
Jac_1=simplify(jacobian(p_end,q));
%方法二：
Jac2_v=[simplify(cross(x1 ,(p_c-p_a))) ,simplify(cross(x2,(p_c-p_b)))];
Jac2_w=[    x1   ,                      x2       ];
Jac_2=[Jac2_v;Jac2_w];
file=fopen('get_jac.m','w');
fprintf(file,'function jac=get_jac() \n');
fprintf(file,'global ulink \n');
fprintf(file,'q1=ulink.q1; q2=ulink.q2; \n');
fprintf(file,'l1=ulink.l1;l2=ulink.l2; \n');
fprintf(file,'jac1=[%s,%s,0,0,0,0] ;\n',[Jac_2(1,1),Jac_2(1,2)]);
fprintf(file,'jac2=[%s,%s,0,0,0,0] ;\n',[Jac_2(2,1),Jac_2(2,2)]);
fprintf(file,'jac3=[%s,%s,0,0,0,0] ;\n',[Jac_2(3,1),Jac_2(3,2)]);
fprintf(file,'jac4=[%s,%s,0,0,0,0] ;\n',[Jac_2(4,1),Jac_2(4,2)]);
fprintf(file,'jac5=[%s,%s,0,0,0,0] ;\n',[Jac_2(5,1),Jac_2(5,2)]);
fprintf(file,'jac6=[%s,%s,0,0,0,0] ;\n',[Jac_2(6,1),Jac_2(6,2)]);
fprintf(file,'jac=[jac1;jac2;jac3;jac4;jac5;jac6] ;\n');
fprintf(file,'end \n');
fclose(file);
%%
%动力学
%提取动力学表达式
%速度/角速度
v0=[0,0,0]';%基坐标系平动速度（标量）
w0=[0,0,0]';%基坐标系角速度
syms aw1 aw2 w1_x w1_y w1_z w2_x w2_y w2_z real
syms I1 I1_11 I1_12 I1_13  I1_21  I1_22  I1_23  I1_31  I1_32  I1_33 real 
syms I2 I2_11 I2_12 I2_13  I2_21  I2_22  I2_23  I2_31  I2_32  I2_33 real 
syms dw1 dw2 real
%惯性张量（初始状态，在各自局部坐标系中计算）
I1=[I1_11 I1_12 I1_13; I1_21  I1_22  I1_23; I1_31  I1_32  I1_33];
I2=[I2_11 I2_12 I2_13; I2_21  I2_22  I2_23; I2_31  I2_32  I2_33];
%转化为世界坐标系中
I1_w=R0_1*I1*R0_1';
I2_w=R0_2*I2*R0_2';
%关节角速度
aw1=[w1_x,w1_y,w1_z]';
aw2=[w2_x,w2_y,w2_z]';
v_a=v0;
w0_1=aw1.*x1;
w1_2=aw2.*x2;
w1=w0_1;
w2=w1+(R0_1*w1_2);
v_e=v_a+cross(w1,r_ae);
v_b=v_a+cross(w1,r_ab);
v_c=v_b+cross(w2,r_bc);
v_f=v_b+cross(w2,r_bf);
syms m1 m2 g real
%动能 dot 点乘后相加
KE=simplify(0.5*m1*dot(v_e,v_e)+0.5*m2*dot(v_f,v_f)+0.5*w1'*I1_w*w1+0.5*w2'*I2_w*w2);
%势能 position energy
PE=simplify(m1*g*dot(p_e,z0)+m2*g*dot(p_f,z0));
%拉格朗日算子
L=KE-PE;%L(q,dq)
q=[q1;q2];    %角度
dq=[w1_x;w2_x];   %角速度
ddq=[dw1;dw2];%角加速度
dL_dq=jacobian(L,q);
dL_ddq=jacobian(L,dq);
ddL_ddq_dt=simplify(jacobian(dL_ddq,q)*dq+jacobian(dL_ddq,dq)*ddq);
syms T1 T2
T=[T1;T2];%输入力矩
%出错 dL_dq是1*2 的其它的是2*1 的导致运算出错
eqn(1)=ddL_ddq_dt(1)-dL_dq(1)-T(1);
eqn(2)=ddL_ddq_dt(2)-dL_dq(2)-T(2);
eqn_1=collect(simplify(eqn(1)),[dw1;dw2]);
eqn_2=collect(simplify(eqn(2)),[dw1;dw2]);
rhs1=subs(eqn_1,[dw1;dw2],[0;0]);
rhs2=subs(eqn_2,[dw1;dw2],[0;0]);
m11=subs(eqn_1,[dw1;dw2],[1;0])-rhs1;
m12=subs(eqn_1,[dw1;dw2],[0;1])-rhs1;
m21=subs(eqn_2,[dw1;dw2],[1;0])-rhs2;
m22=subs(eqn_2,[dw1;dw2],[0;1])-rhs2;
file=fopen('rhs.m','w');
fprintf(file,'function zdot=rhs(tspan,z0) \n');
fprintf(file,'global ulink I1 I2 \n');
fprintf(file,'global G \n');
fprintf(file , 'g=G; \n');
fprintf(file, ' m1=ulink.m1;m2=ulink.m2; \n');
fprintf(file,'l1=ulink.l1;l2=ulink.l2; \n');
%赋初值
fprintf(file , 'q1=z0(1);q2=z0(3); \n');
fprintf(file , 'w1=z0(2);w2=z0(4); \n');
% fprintf(file , 'dw1=0;dw2=0; \n');
fprintf(file , 'w1_x=w1; w2_x=w2;\n');
fprintf(file ,'I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);\n');
fprintf(file ,'I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);\n');
fprintf(file ,'I1_31=I1(3,1);I1_12=I1(3,2);I1_13=I1(3,3);\n');
fprintf(file ,'I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);\n');
fprintf(file ,'I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);\n');
fprintf(file ,'I2_31=I2(3,1);I2_12=I2(3,2);I2_13=I2(3,3);\n');
fprintf(file ,'\n');
fprintf(file ,'T1=0;T2=0; \n');
fprintf(file ,'\n');
fprintf(file ,'m11=%s; \n',char(m11));
fprintf(file ,'m12=%s; \n',char(m12));
fprintf(file ,'m21=%s; \n',char(m21));
fprintf(file ,'m22=%s; \n',char(m22));
fprintf(file , 'rhs1=%s; \n' , char(rhs1));
fprintf(file , 'rhs2=%s; \n' , char(rhs2));
fprintf(file , 'M=[m11,m12;m21,m22];\n');
fprintf(file , 'RHS=[rhs1;rhs2]; \n');
fprintf(file ,'\n');
fprintf(file,'X=-M\\RHS ;\n'); %AB=C B=A\C;AB=C A=C/B
fprintf(file,'dw1=X(1);dw2=X(2); \n');
fprintf(file,'zdot=[w1_x dw1 w2_x dw2]'';\n');
fprintf(file , 'end\n');
fclose(file);
%%
%提取动能和势能
file=fopen('energy.m','w');
fprintf(file ,'function  [PE,KE]=energy() \n');
fprintf(file ,'%%动能势能计算 \n');
fprintf(file ,'global ulink I1 I2 \n');
fprintf(file ,'global G \n');
fprintf(file , 'g=G; \n');
fprintf(file,'l1=ulink.l1;l2=ulink.l2; \n');
fprintf(file , 'm1=ulink.m1;m2=ulink.m2; \n');
fprintf(file , 'q1=ulink.q1;q2=ulink.q2;  \n');
fprintf(file , 'w1=ulink.w1;w2=ulink.w2;  \n');
fprintf(file , 'w1_x=w1(1); w1_y=w1(2);w1_z=w1(3);w2_x=w2(1) ;w2_y=w2(2);w2_z=w2(3);\n');
fprintf(file ,'I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);\n');
fprintf(file ,'I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);\n');
fprintf(file ,'I1_31=I1(3,1);I1_12=I1(3,2);I1_13=I1(3,3);\n');
fprintf(file ,'I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);\n');
fprintf(file ,'I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);\n');
fprintf(file ,'I2_31=I2(3,1);I2_12=I2(3,2);I2_13=I2(3,3);\n');

fprintf(file , 'KE=%s;\n',char(KE));
fprintf(file , 'PE=%s;\n',char(PE));
fprintf(file , 'end\n');
fclose(file);
end
