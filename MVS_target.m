figure('Color','w','Units','inches','Position',[1 1 6 3.2]);

% Data (rows = methods, columns = targets)
data = [
    3.1100 5.0817 2.0305;   % Reference
    3.0887 5.2050 2.0188;   % Simulation
    3.1697 5.1652 1.9440;   % HW 1x
    2.9679 5.1197 1.8781;   % HW 2x
    2.6855 7.8986 1.9902    % HW 4xa
];

bar(data);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);

set(gca,'XTickLabel',{'Reference','Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});

ylabel('Trajectory Error (mm)','FontName','Times New Roman','FontSize',10);
xlabel('Method','FontName','Times New Roman','FontSize',10);

legend({'Target 1','Target 2','Target 3'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',8);

title('Minimum Distance to Targets','FontName','Times New Roman','FontSize',10);