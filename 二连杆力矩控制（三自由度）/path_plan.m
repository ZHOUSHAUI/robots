function plan_point=path_plan(t)

x=5*sin(2*pi/5*t);
y=12;
z=10+5*cos(2*pi/5*t);

plan_point=[x;y;z];
end