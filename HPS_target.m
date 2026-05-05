figure('Color','w','Units','inches','Position',[1 1 6 3.2]);

% Data (rows = methods, columns = targets)
data = [
    0.125 0.037 0.208 0.118 0.248;   % Reference
    0.038 0.104 0.150 0.194 0.142;   % Simulation
    0.162 0.364 0.152 0.168 0.527;   % HW 1x
    0.201 0.400 0.104 0.237 0.212;   % HW 2x
    0.227 0.470 0.666 0.605 0.128    % HW 4x
];

bar(data);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);

set(gca,'XTickLabel',{'Reference','Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});

ylabel('Minimum Distance (mm)','FontName','Times New Roman','FontSize',10);
xlabel('Method','FontName','Times New Roman','FontSize',10);

legend({'Target-1','Target-2','Target-3','Target-4','Target-5'},'Location','northoutside','Orientation','horizontal','FontSize',8);

title('Minimum Distance to Targets','FontName','Times New Roman','FontSize',10);