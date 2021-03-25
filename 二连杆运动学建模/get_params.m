function get_params
global ulink
%坐标系(基)
x0=[1,0,0]';
y0=[0,1,0]';
z0=[0,0,1]';
syms q1 q2 
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
p_b=p_a+ulink.l1*z1;
r_ab=p_b-p_a;
r_ae=0.5*r_ab;
p_e=p_a+r_ae;
p_c=p_b+ulink.l2*R0_1*z2;
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
%运动学
v0=[0,0,0]';
w0=[0,0,0]';
%提取雅克比矩阵
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
fprintf(file,'jac1=[%s,%s,0,0,0,0] ;\n',[Jac_2(1,1),Jac_2(1,2)]);
fprintf(file,'jac2=[%s,%s,0,0,0,0] ;\n',[Jac_2(2,1),Jac_2(2,2)]);
fprintf(file,'jac3=[%s,%s,0,0,0,0] ;\n',[Jac_2(3,1),Jac_2(3,2)]);
fprintf(file,'jac4=[%s,%s,0,0,0,0] ;\n',[Jac_2(4,1),Jac_2(4,2)]);
fprintf(file,'jac5=[%s,%s,0,0,0,0] ;\n',[Jac_2(5,1),Jac_2(5,2)]);
fprintf(file,'jac6=[%s,%s,0,0,0,0] ;\n',[Jac_2(6,1),Jac_2(6,2)]);
fprintf(file,'jac=[jac1;jac2;jac3;jac4;jac5;jac6] ;\n');
fprintf(file,'end \n');
fclose(file);
end