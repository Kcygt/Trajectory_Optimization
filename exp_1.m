% figure('Color','w','Units','inches','Position',[1 1 4 3.2]);
% 
% % ---------------- POSITION RMSE ----------------
% % Rows = Joints (1–3)
% % Columns = Reference-Simulation, Reference-Hardware
% 
% position_rmse = [
%     0.0034  0.0050;   % Joint 1
%     0.0018  0.0027;   % Joint 2
%     0.0039  0.0018    % Joint 3
% ];
% 
% bar(position_rmse);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'Joint-1','Joint-2','Joint-3'});
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference-Simulation','Reference-Hardware'}, ...
%        'Location','northwest','FontSize',8);



% figure('Color','w','Units','inches','Position',[1 1 4 3.2]);
% 
% % ---------------- VELOCITY RMSE ----------------
% % Rows = Joints (1–3)
% % Columns = Reference-Simulation, Reference-Hardware
% 
% velocity_rmse = [
%     0.0478  0.1238;   % Joint 1
%     0.0156  0.0640;   % Joint 2
%     0.0169  0.0590    % Joint 3
% ];
% 
% bar(velocity_rmse);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'Joint-1','Joint-2','Joint-3'});
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference-Simulation','Reference-Hardware'}, ...
%        'Location','northwest','FontSize',8);


% 
% 
% figure('Color','w','Units','inches','Position',[1 1 7 3.2]);
% 
% % ---------------- POSITION RMSE ----------------
% % Rows = Joints (1–3)
% % Columns = Simulation, 1.5x, 2x, 4x
% 
% position_rmse_speedup = [
%     0.0034  0.0052  0.0069  0.0134;   % Joint 1
%     0.0018  0.0028  0.0038  0.0134;   % Joint 2
%     0.0039  0.0061  0.0083  0.0174    % Joint 3
% ];
% 
% bar(position_rmse_speedup);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'Joint-1','Joint-2','Joint-3'});
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE vs Speed-Up Factors','FontName','Times New Roman','FontSize',10);
% 
% legend({'Simulation','1.5x','2x','4x'}, ...
%        'Location','northwest','FontSize',8);



figure('Color','w','Units','inches','Position',[1 1 7 3.2]);

% ---------------- VELOCITY RMSE ----------------
% Rows = Joints (1–3)
% Columns = Simulation, 1.5x, 2x, 4x

velocity_rmse_speedup = [
    0.474  0.653  0.84292  1.59950;   % Joint 1
    0.275  0.365  0.46718  0.90370;   % Joint 2
    0.632  0.791  1.00280  2.01210    % Joint 3
];

bar(velocity_rmse_speedup);
grid on;

set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
set(gca,'XTickLabel',{'Joint-1','Joint-2','Joint-3'});

ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
title('Joint Velocity RMSE vs Speed-Up Factors','FontName','Times New Roman','FontSize',10);

legend({'Simulation','1.5x','2x','4x'}, ...
       'Location','northwest','FontSize',8);