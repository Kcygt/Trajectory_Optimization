figure('Color','w','Units','inches','Position',[1 1 7 3.2]);

tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

% ---------------- POSITION RMSE ----------------
position_rmse = [
    0.002919 0.000875 0.001546;
    0.002912 0.001437 0.001359;
    0.009680 0.003392 0.004839;
    0.027583 0.008087 0.012069
];

nexttile;
bar(position_rmse);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
set(gca,'XTickLabel',{'Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});

ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
title('Position RMSE','FontName','Times New Roman','FontSize',10);

legend({'Joint-1','Joint-2','Joint-3'},'Location','northwest','FontSize',8);

% ---------------- VELOCITY RMSE ----------------
velocity_rmse = [
    0.015024 0.004590 0.008331;
    0.065922 0.124050 0.071980;
    0.411331 0.148348 0.205789;
    1.262030 0.379649 0.574917
];

nexttile;
bar(velocity_rmse);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
set(gca,'XTickLabel',{'Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});

ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
title('Velocity RMSE','FontName','Times New Roman','FontSize',10);

legend({'Joint-1','Joint-2','Joint-3'},'Location','northwest','FontSize',8);