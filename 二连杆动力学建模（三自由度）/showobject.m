function handle=showobject
global ulink
handle(1)=plot3([ulink.p_a(1),ulink.p_b(1)],[ulink.p_a(2),ulink.p_b(2)],[ulink.p_a(3),ulink.p_b(3)],'r','LineWidth',5);
hold on;
handle(2)= plot3([ulink.p_b(1),ulink.p_c(1)],[ulink.p_b(2),ulink.p_c(2)],[ulink.p_b(3),ulink.p_c(3)],'b','LineWidth',5);
hold on;
