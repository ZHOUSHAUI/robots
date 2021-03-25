%二连杆仿真公式提取
function get_params
syms l1 l2 real
%%
%提取公式
%提取位置，杆向量位置表达式
x0=[1,0,0]';
y0=[0,1,0]';
z0=[0,0,1]';
syms q1 q2 q3 real
%在o0中
x1=x0*cos(-q1)+y0*sin(-q1);
y1=-x0*sin(-q1)+y0*cos(-q1);
z1=z0;
%在o1中
x2=x0;
y2=0+y0*cos(q2)-z0*sin(q2);
z2=0+y0*sin(q2)+z0*cos(q2);
%在o2中
x3=x0;
y3=0+y0*cos(q3)-z0*sin(q3);
z3=0+y0*sin(q3)+z0*cos(q3);
%姿态矩阵
R0_1=[x1,y1,z1];
R1_2=[x2,y2,z2];
R2_3=[x3,y3,z3];
R0_2=R0_1*R1_2;
R0_3=R0_1*R1_2*R2_3;
X1=R0_1(:,1);
Y1=R0_1(:,2);
Z1=R0_1(:,3);
X2=R0_2(:,1);
Y2=R0_2(:,2);
Z2=R0_2(:,3);
X3=R0_3(:,1);
Y3=R0_3(:,2);
Z3=R0_3(:,3);
%位置向量 世界坐标系下表示
p_a=[0,0,0]';
p_b=p_a+l1*Z2;
r_ab=p_b-p_a;
r_ae=0.5*r_ab;
p_e=p_a+r_ae;
p_c=p_b+l2*Z3;
r_bc=p_c-p_b;
p_f=p_b+0.5*r_bc;
r_bf=p_f-p_b;
file=fopen('vector_update.m','w');
fprintf(file,'function vector_update() \n');
fprintf(file,'global ulink\n');
fprintf(file,'q1=ulink.q1;q2=ulink.q2;q3=ulink.q3; \n');
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
q=[q1;q2;q3];
Jac_1=simplify(jacobian(p_end,q));
%方法二：
Jac2_v=[simplify(cross(Z1 ,(p_c-p_a))),simplify(cross(X2 ,(p_c-p_a))) ,simplify(cross(X3,(p_c-p_b)))];
Jac2_w=[   Z1                         ,                  X2           ,              X3             ];
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
syms aw1 aw2  aw3 w1_x w1_y w1_z w2_x w2_y w2_z w3_x w3_y w3_z real
syms I1 I1_11 I1_12 I1_13  I1_21  I1_22  I1_23  I1_31  I1_32  I1_33 real 
syms I2 I2_11 I2_12 I2_13  I2_21  I2_22  I2_23  I2_31  I2_32  I2_33 real 
syms dw1 dw2 dw3 real
%惯性张量（初始状态，在各自局部坐标系中计算）
I1=[I1_11 I1_12 I1_13; I1_21  I1_22  I1_23; I1_31  I1_32  I1_33];
I2=[I2_11 I2_12 I2_13; I2_21  I2_22  I2_23; I2_31  I2_32  I2_33];
%转化为世界坐标系中
I1_w=R0_2*I1*R0_2';
I2_w=R0_3*I2*R0_3';
%关节角速度(绝对角速度)
aw1=[w1_x,w1_y,w1_z]';
aw2=[w2_x,w2_y,w2_z]';
aw3=[w3_x,w3_y,w3_z]';


v_a=v0;
w0_1=aw1.*z1;
w1_2=aw2.*x2;
w2_3=aw3.*x3;

w1=w0_1;
w2=w1+(R0_1*w1_2);
w3=w2+(R0_2*w2_3);

v_e=v_a+cross(w2,r_ae);
v_b=v_a+cross(w2,r_ab);
v_c=v_b+cross(w3,r_bc);
v_f=v_b+cross(w3,r_bf);
syms m1 m2 g real
%动能 dot 点乘后相加
KE=simplify(0.5*m1*dot(v_e,v_e)+0.5*m2*dot(v_f,v_f)+0.5*w2'*I1_w*w2+0.5*w3'*I2_w*w3);
%势能 position energy
PE=simplify(m1*g*dot(p_e,z0)+m2*g*dot(p_f,z0));
%拉格朗日算子
L=KE-PE;%L(q,dq)
q=[q1;q2;q3];        %角度
dq= [w1_z;w2_x;w3_x];   %角速度
ddq=[dw1 ;dw2 ;dw3 ];    %角加速度
dL_dq=jacobian(L,q);
dL_ddq=jacobian(L,dq);
ddL_ddq_dt=simplify(jacobian(dL_ddq,q)*dq+jacobian(dL_ddq,dq)*ddq);
syms T1 T2 T3 real
T=[T1;T2;T3];%输入力矩
%出错 dL_dq是1*2 的其它的是2*1 的导致运算出错
eqn(1)=ddL_ddq_dt(1)-dL_dq(1)-T(1);
eqn(2)=ddL_ddq_dt(2)-dL_dq(2)-T(2);
eqn(3)=ddL_ddq_dt(3)-dL_dq(3)-T(3);

eqn_1=collect(simplify(eqn(1)),[dw1;dw2;dw3]);
eqn_2=collect(simplify(eqn(2)),[dw1;dw2;dw3]);
eqn_3=collect(simplify(eqn(3)),[dw1;dw2;dw3]);

rhs1=subs(eqn_1,[dw1;dw2;dw3],[0;0;0]);
rhs2=subs(eqn_2,[dw1;dw2;dw3],[0;0;0]);
rhs3=subs(eqn_3,[dw1;dw2;dw3],[0;0;0]);

m11=subs(eqn_1,[dw1;dw2;dw3],[1;0;0])-rhs1;
m12=subs(eqn_1,[dw1;dw2;dw3],[0;1;0])-rhs1;
m13=subs(eqn_1,[dw1;dw2;dw3],[0;0;1])-rhs1;

m21=subs(eqn_2,[dw1;dw2;dw3],[1;0;0])-rhs2;
m22=subs(eqn_2,[dw1;dw2;dw3],[0;1;0])-rhs2;
m23=subs(eqn_2,[dw1;dw2;dw3],[0;0;1])-rhs2;

m31=subs(eqn_3,[dw1;dw2;dw3],[1;0;0])-rhs3;
m32=subs(eqn_3,[dw1;dw2;dw3],[0;1;0])-rhs3;
m33=subs(eqn_3,[dw1;dw2;dw3],[0;0;1])-rhs3;
%重力补偿
CG1=collect(simplify(rhs1),[w1_z;w2_x;w3_x]);
CG2=collect(simplify(rhs2),[w1_z;w2_x;w3_x]);
CG3=collect(simplify(rhs3),[w1_z;w2_x;w3_x]);
G1=subs(CG1,[w1_z;w2_x;w3_x],[0;0;0;]);
G2=subs(CG2,[w1_z;w2_x;w3_x],[0;0;0;]);
G3=subs(CG3,[w1_z;w2_x;w3_x],[0;0;0;]);
file=fopen('rhs.m','w');
fprintf(file,'function zdot=rhs(t,z0,flag) \n');
fprintf(file,'global ulink I1 I2 \n');
fprintf(file,'global G \n');
fprintf(file , 'g=G; \n');
fprintf(file, ' m1=ulink.m1;m2=ulink.m2; \n');
fprintf(file,'l1=ulink.l1;l2=ulink.l2; \n');
%赋初值
fprintf(file , 'q1=z0(1);q2=z0(3);q3=z0(5); \n');
fprintf(file , 'w1=z0(2);w2=z0(4);w3=z0(6); \n');
% fprintf(file , 'dw1=0;dw2=0; \n');
fprintf(file , 'w1_z=w1;w2_x=w2; w3_x=w3;\n');
fprintf(file ,'I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);\n');
fprintf(file ,'I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);\n');
fprintf(file ,'I1_31=I1(3,1);I1_32=I1(3,2);I1_33=I1(3,3);\n');

fprintf(file ,'I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);\n');
fprintf(file ,'I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);\n');
fprintf(file ,'I2_31=I2(3,1);I2_32=I2(3,2);I2_33=I2(3,3);\n');
fprintf(file ,'\n');
fprintf(file ,'s_v=[q1,w1,q2,w2,q3,w3];\n');
fprintf(file ,'[T1,T2,T3]=control_strategy(t,s_v);\n');
fprintf(file ,'\n');
fprintf(file ,'m11=%s; \n',char(m11));
fprintf(file ,'m12=%s; \n',char(m12));
fprintf(file ,'m13=%s; \n',char(m13));
fprintf(file ,'m21=%s; \n',char(m21));
fprintf(file ,'m22=%s; \n',char(m22));
fprintf(file ,'m23=%s; \n',char(m23));
fprintf(file ,'m31=%s; \n',char(m31));
fprintf(file ,'m32=%s; \n',char(m32));
fprintf(file ,'m33=%s; \n',char(m33));

fprintf(file , 'rhs1=%s; \n' , char(rhs1));
fprintf(file , 'rhs2=%s; \n' , char(rhs2));
fprintf(file , 'rhs3=%s; \n' , char(rhs3));
fprintf(file , 'M=[m11,m12,m13;m21,m22,m23;m31,m32,m33];\n');
fprintf(file , 'RHS=[rhs1;rhs2;rhs3]; \n');
fprintf(file ,'\n');
fprintf(file,'X=-M\\RHS ;\n'); %AB=C B=A\C;AB=C A=C/B
fprintf(file,'dw1=X(1);dw2=X(2);dw3=X(3); \n');
fprintf(file,'zdot=[w1_z dw1 w2_x dw2 w3_x dw3]'';\n');
fprintf(file , 'end\n');
fclose(file);
%%
file=fopen('get_G.m','w');
fprintf(file,'function GG=get_G(s_v) \n');
fprintf(file,'global G ulink  I1 I2\n');
fprintf(file , 'g=G; \n');
fprintf(file , 'm1=ulink.m1;m2=ulink.m2; \n');
fprintf(file , 'l1=ulink.l1;l2=ulink.l2; \n');
fprintf(file ,'I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);\n');
fprintf(file ,'I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);\n');
fprintf(file ,'I1_31=I1(3,1);I1_32=I1(3,2);I1_33=I1(3,3);\n');
fprintf(file ,'I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);\n');
fprintf(file ,'I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);\n');
fprintf(file ,'I2_31=I2(3,1);I2_32=I2(3,2);I2_33=I2(3,3);\n');
fprintf(file , 'T1=0;T2=0;T3=0;\n');
fprintf(file , 'q1=s_v(1);q2=s_v(3);q3=s_v(5);w1_z=s_v(2);w2_x=s_v(4);w3_x=s_v(6);\n');
fprintf(file , 'G1=%s; \n' , char(G1));
fprintf(file , 'G2=%s; \n' , char(G2));
fprintf(file , 'G3=%s; \n' , char(G3));
fprintf(file , 'CG1=%s; \n' , char(CG1));
fprintf(file , 'CG2=%s; \n' , char(CG2));
fprintf(file , 'CG3=%s; \n' , char(CG3));
fprintf(file , 'GG=[G1;G2;G3,CG1;CG2;CG3]; \n');
fprintf(file , 'end \n');
fclose(file);
%提取动能和势能
file=fopen('energy.m','w');
fprintf(file ,'function  [PE,KE]=energy() \n');
fprintf(file ,'%%动能势能计算 \n');
fprintf(file ,'global ulink I1 I2 \n');
fprintf(file ,'global G \n');
fprintf(file , 'g=G; \n');
fprintf(file,'l1=ulink.l1;l2=ulink.l2; \n');
fprintf(file , 'm1=ulink.m1;m2=ulink.m2; \n');
fprintf(file , 'q1=ulink.q1;q2=ulink.q2; q3=ulink.q3; \n');
fprintf(file , 'w1=ulink.w1;w2=ulink.w2; w3=ulink.w3; \n');
fprintf(file , 'w1_x=w1(1); w1_y=w1(2);w1_z=w1(3);\n');
fprintf(file , 'w2_x=w2(1) ;w2_y=w2(2);w2_z=w2(3);\n');
fprintf(file , 'w3_x=w3(1) ;w3_y=w3(2);w3_z=w3(3);\n');
fprintf(file ,'I1_11=I1(1,1);I1_12=I1(1,2);I1_13=I1(1,3);\n');
fprintf(file ,'I1_21=I1(2,1);I1_22=I1(2,2);I1_23=I1(2,3);\n');
fprintf(file ,'I1_31=I1(3,1);I1_32=I1(3,2);I1_33=I1(3,3);\n');

fprintf(file ,'I2_11=I2(1,1);I2_12=I2(1,2);I2_13=I2(1,3);\n');
fprintf(file ,'I2_21=I2(2,1);I2_22=I2(2,2);I2_23=I2(2,3);\n');
fprintf(file ,'I2_31=I2(3,1);I2_32=I2(3,2);I2_33=I2(3,3);\n');

fprintf(file , 'KE=%s;\n',char(KE));
fprintf(file , 'PE=%s;\n',char(PE));
fprintf(file , 'end\n');
fclose(file);
end


