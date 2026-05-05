figure('Color','w','Units','inches','Position',[1 1 7 3.2]);

tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

% ---------------- POSITION RMSE ----------------
pos_rmse = [
    0.00186 0.00164 0.00113;   % Sim
    0.00297 0.00231 0.00139;   % HW 1x
    0.00412 0.00265 0.00155;   % HW 2x
    0.01166 0.00588 0.00206    % HW 4x
];

nexttile;
bar(pos_rmse);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
set(gca,'XTickLabel',{'Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});

ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
title('Position RMSE','FontName','Times New Roman','FontSize',10);

legend({'Joint-1','Joint-2','Joint-3'},'Location','northwest','FontSize',8);

% ---------------- VELOCITY RMSE ----------------
vel_rmse = [
    0.01208 0.00661 0.00798;   % Sim
    0.05963 0.08583 0.07030;   % HW 1x
    0.10469 0.12189 0.07206;   % HW 2x
    0.43581 0.18387 0.10528    % HW 4x
];

nexttile;
bar(vel_rmse);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
set(gca,'XTickLabel',{'Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});

ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
title('Velocity RMSE','FontName','Times New Roman','FontSize',10);

legend({'J1','J2','J3'},'Location','northwest','FontSize',8);