function handle=showobject
global ulink
persistent pt
if isempty(pt)
    pt=0;
end
handle(1)=plot3([ulink.p_a(1),ulink.p_b(1)],[ulink.p_a(2),ulink.p_b(2)],[ulink.p_a(3),ulink.p_b(3)],'r','LineWidth',2);
hold on;
handle(2)= plot3([ulink.p_b(1),ulink.p_c(1)],[ulink.p_b(2),ulink.p_c(2)],[ulink.p_b(3),ulink.p_c(3)],'b','LineWidth',2);
hold on;
%»æÖÆ»ù×ù
pt=pt+1;
if pt==1
    vert = [-1 -1 -1; -1 1 -1; 1 1 -1; 1 -1 -1 ; ...
              -1 -1 1; -1 1 1; 1 1 1; 1 -1 1 ;];
    fac = [1 2 3 4; ...
        2 6 7 3; ...
        4 3 7 8; ...
        1 5 8 4; ...
        1 2 6 5; ...
        5 6 7 8];
    patch('Faces',fac,'Vertices',vert,'FaceColor','k');  % patch function
    hold on
end