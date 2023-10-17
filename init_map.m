function init_map(X,Y,Z,Xpfset,Station,real_source)
    robot_color =string(["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F","#000000"]);
    hight = ones(1,size(Xpfset,2))*5; %地基
    figure(1);hold on;
    surf(X,Y,Z)      % 画曲面图
    shading flat     % 各小曲面之间不要网格
    % scatter3(Xpfset(1, :), Xpfset(2, :),hight,10,'o','MarkerFaceColor',robot_color(1))
    plot3(Xpfset(1, :), Xpfset(2, :),hight,'o','MarkerFaceColor',robot_color(1),'markersize',4)
    plot3(Station(1,1),Station(2,1),5,'o','MarkerFaceColor',robot_color(4),'MarkerEdgeColor', robot_color(4),'markersize',6,'LineWidth', 2);
    plot3(real_source(1,1),real_source(2,1),10,'r*','MarkerFaceColor',robot_color(2),'markersize',20,'LineWidth', 2);
    view(2)
    hold off;
end
