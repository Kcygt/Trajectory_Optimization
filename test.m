
figure('Color','w','Units','inches','Position',[1 1 4.5 3]);

% Distance data (mm)
cart_data = [
    2.954 5.019 1.987;   % Reference
    3.089 5.205 2.019;   % Simulation
    3.170 5.165 5.165    % Hardware
];

% Plot (transpose so targets are on x-axis)
bar(cart_data');
grid on;

% Axis formatting
set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
set(gca,'XTickLabel',{'Target 1','Target 2','Target 3'});

xlabel('Target Points','FontName','Times New Roman','FontSize',10);
ylabel('Distance Error (mm)','FontName','Times New Roman','FontSize',10);

title('Minimum Distance Between Trajectories and Targets',...
      'FontName','Times New Roman','FontSize',10);

% Legend
legend({'Reference','Simulation','Hardware'},...
       'Location','best','FontSize',8);




% figure('Color','w','Units','inches','Position',[1 1 4.5 3]);
% 
% % Distance data from table
% cart_data = [
%     0.004421 0.003005;   % Simulation
%     0.004687 0.002540;   % Real
%     0.004790 0.002428;   % 1.5x
%     0.004859 0.002258;   % 2x
%     0.004673 0.002105    % 4x
% ];
% 
% % Plot (transpose to match targets on x-axis)
% bar(cart_data');
% grid on;
% 
% % Axis formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'Target 1','Target 2'});
% 
% xlabel('Target Points','FontName','Times New Roman','FontSize',10);
% ylabel('Distance Error (m)','FontName','Times New Roman','FontSize',10);
% 
% title('Cartesian Accuracy at Target Points',...
%       'FontName','Times New Roman','FontSize',10);
% 
% % Legend
% legend({'Simulation','Real','1.5x','2x','4x'},...
%        'Location','best','FontSize',8);
% 







% figure('Color','w','Units','inches','Position',[1 1 4.5 3]);
% 
% % Force data from table
% force_data = [
%    -0.952  -0.918  -0.960  -1.020  -0.898;   % 1x
%    -1.006  -0.974  -0.979  -1.038  -0.932;   % 2x
%    -0.912  -0.922  -0.958  -1.035  -0.941    % 4x
% ];
% 
% % Plot (transpose to match targets on x-axis)
% bar(force_data');
% grid on;
% 
% % Axis formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'T1','T2','T3','T4','T5'});
% 
% xlabel('Target Points','FontName','Times New Roman','FontSize',10);
% ylabel('Force (N)','FontName','Times New Roman','FontSize',10);
% 
% title('Force at Target Points for Different Speeds',...
%       'FontName','Times New Roman','FontSize',10);
% 
% % Legend
% legend({'1x','2x','4x'},'Location','best','FontSize',8);

% figure('Color','w','Units','inches','Position',[1 1 4.5 3]);
% 
% % Data
% rmse_data = [
%     0.01208 0.00661 0.00798;  % Simulation
%     0.05963 0.08583 0.07030;  % Hardware 1x
%     0.10469 0.12189 0.07206;  % Hardware 2x
%     0.43581 0.18387 0.10528   % Hardware 4x
% ];
% 
% traj = {'Simulation','1x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',traj);
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 4.5 3]);
% 
% % Data
% rmse_data = [
%     0.00186 0.00164 0.00113;  % Simulation
%     0.00297 0.00231 0.00139;  % Hardware 1x
%     0.00412 0.00265 0.00155;  % Hardware 2x
%     0.01166 0.00588 0.00206   % Hardware 4x
% ];
% 
% traj = {'Simulation','1x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',traj);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);

% figure('Color','w','Units','inches','Position',[1 1 5 3]);
% 
% % Data (rows = targets, columns = methods)
% error_data = [
%     3.1100 3.0887 3.1697 2.9679 2.6855;  % Target 1
%     5.0817 5.2050 5.1652 5.1197 7.8986;  % Target 2
%     2.0305 2.0188 1.9440 1.8781 1.9902   % Target 3
% ];
% 
% targets = {'Target 1','Target 2','Target 3'};
% 
% % Plot
% bar(error_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',targets);
% 
% ylabel('Error (mm)','FontName','Times New Roman','FontSize',10);
% title('Minimum distance between the trajectories and target points','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference','Simulation','Hardware 1x','Hardware 2x','Hardware 4x'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% rmse_data = [
%     0.0120823  0.0596326;  % q1
%     0.00661234 0.0858276;  % q2
%     0.00797724 0.0702958   % q3
% ];
% 
% joints = {'Joint 1','Joint 2','Joint 3'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',joints);
% 
% ylabel('RMSE (m/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE vs Reference Trajectories','FontName','Times New Roman','FontSize',10);
% 
% legend({'Simulation','Hardware'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% rmse_data = [
%     0.00186167 0.00164099 0.00112503;  % Simulation vs Desired
%     0.00297032 0.00231372 0.00138979   % Phantom vs Desired
% ];
% 
% cases = {'Simulation','Hardware'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',cases);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 5 3]);
% 
% % Data (Joint 1–3 only, total removed)
% rmse_data = [
%     0.015024 0.004590 0.008331;  % Simulation vs Reference
%     0.065922 0.124050 0.071980;  % Hardware 1x vs Reference
%     0.411331 0.148348 0.205789;  % Hardware 2x vs Reference
%     1.262030 0.379649 0.574917   % Hardware 4x vs Reference
% ];
% 
% runs = {'Simulation','1x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',runs);
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE (Speed Comparison)','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 5 3]);
% 
% % Data (Joint 1–3 only, total removed)
% rmse_data = [
%     0.002919 0.000875 0.001546;  % Simulation
%     0.002912 0.001437 0.001359;  % Hardware 1x
%     0.009680 0.003392 0.004839;  % Hardware 2x
%     0.027583 0.008087 0.012069   % Hardware 4x
% ];
% 
% runs = {'Simulation','1x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',runs);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE (Speed Comparison)','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);

% figure('Color','w','Units','inches','Position',[1 1 6 3]);
% 
% % Data (rows = targets, columns = methods)
% distance_data = [
%     0.125 0.038 0.162 0.201 0.227;  % Target 1
%     0.037 0.104 0.364 0.400 0.470;  % Target 2
%     0.208 0.150 0.152 0.104 0.666;  % Target 3
%     0.118 0.194 0.168 0.237 0.605;  % Target 4
%     0.248 0.142 0.527 0.212 0.128   % Target 5
% ];
% 
% targets = {'Target 1','Target 2','Target 3','Target 4','Target 5'};
% 
% % Plot
% bar(distance_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',targets);
% 
% ylabel('Distance (mm)','FontName','Times New Roman','FontSize',10);
% title('Minimum Distance to Targets (Speed Variation)','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference','Simulation','Hardware 1x','Hardware 2x','Hardware 4x'},...
%        'Location','northwest','FontSize',8);

% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% rmse_data = [
%     0.015024 0.065922;  % Joint 1
%     0.004590 0.124050;  % Joint 2
%     0.008331 0.071980   % Joint 3
% ];
% 
% joints = {'Joint 1','Joint 2','Joint 3'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',joints);
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE ','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference–Simulation','Reference–Hardware'},...
%        'Location','northwest','FontSize',8);

% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% rmse_data = [
%     0.002919 0.002912;  % Joint 1
%     0.000875 0.001437;  % Joint 2
%     0.001546 0.001359   % Joint 3
% ];
% 
% joints = {'Joint 1','Joint 2','Joint 3'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',joints);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE ','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference–Simulation','Reference–Hardware'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 5 3]);
% 
% % Data (rows = targets, columns = Reference/Simulation/Hardware)
% distance_data = [
%     0.125 0.038 0.162;  % Target 1
%     0.037 0.104 0.364;  % Target 2
%     0.208 0.150 0.152;  % Target 3
%     0.118 0.194 0.168;  % Target 4
%     0.248 0.142 0.527   % Target 5
% ];
% 
% targets = {'Target 1','Target 2','Target 3','Target 4','Target 5'};
% 
% % Plot
% bar(distance_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',targets);
% 
% ylabel('Distance (mm)','FontName','Times New Roman','FontSize',10);
% title('Minimum Distance to Each Target','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference','Simulation','Hardware'},...
%        'Location','northwest','FontSize',8);
% 
% 
% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data (qdot1–qdot3 only, total removed)
% rmse_data = [
%     0.041036 0.057953 0.071962;  % Real
%     0.107392 0.111751 0.441589;  % 1.5x
%     0.203698 0.198618 0.871794;  % 2x
%     0.610292 0.602191 2.645757   % 4x
% ];
% 
% runs = {'Hardware','1.5x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',runs);
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE ','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);

% 
% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data (q1–q3 only, total removed)
% rmse_data = [
%     0.003180 0.002802 0.002405;  % Real
%     0.003210 0.002889 0.002946;  % 1.5x
%     0.003467 0.003388 0.003954;  % 2x
%     0.005716 0.007897 0.010840   % 4x
% ];
% 
% runs = {'Hardware','1.5x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',runs);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE ','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint 1','Joint 2','Joint 3'},...
%        'Location','northwest','FontSize',8);



% figure('Color','w','Units','inches','Position',[1 1 7 3]);
% 
% tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
% 
% % ---------------- TARGET DISTANCES ----------------
% distance_data = [
%     0.004421 0.003005;  % Simulation
%     0.004687 0.002540;  % Real
%     0.004790 0.002428;  % 1.5x
%     0.004859 0.002258;  % 2x
%     0.004673 0.002105   % 4x
% ];
% 
% runs = {'Simulation','Real','1.5x','2x','4x'};
% 
% nexttile;
% bar(distance_data);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',runs);
% 
% ylabel('Distance (m)','FontName','Times New Roman','FontSize',10);
% title('Minimum Distance to Targets','FontName','Times New Roman','FontSize',10);
% 
% legend({'Target 1','Target 2'},'Location','northwest','FontSize',8);
% 
% % ---------------- TOTAL RMSE ----------------
% rmse_total = [NaN, 0.001132, 0.001154, 0.001335, 0.003007];
% 
% nexttile;
% bar(rmse_total);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',runs);
% 
% ylabel('RMSE (m)','FontName','Times New Roman','FontSize',10);
% title('Total Cartesian RMSE','FontName','Times New Roman','FontSize',10);

% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% rmse_data = [
%     0.0524 0.1205;  % Joint 1
%     0.0318 0.0880;  % Joint 2
%     0.0285 0.0756   % Joint 3
% ];
% 
% joints = {'Joint 1','Joint 2','Joint 3'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',joints);
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE (Reference Comparison)','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference–Simulation','Reference–Hardware'},...
%        'Location','northwest','FontSize',8);


% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% rmse_data = [
%     0.0041 0.0056;  % Joint 1
%     0.0032 0.0044;  % Joint 2
%     0.0027 0.0041   % Joint 3
% ];
% 
% joints = {'Joint 1','Joint 2','Joint 3'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',joints);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE (Reference Comparison)','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference–Simulation','Reference–Hardware'},...
%        'Location','northwest','FontSize',8);
% 

% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data
% distance_data = [
%     0.004421 0.004293 0.005004;  % Target 1
%     0.003005 0.002988 0.002142   % Target 2
% ];
% 
% targets = {'Target 1','Target 2'};
% 
% % Plot
% bar(distance_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',targets);
% 
% ylabel('Distance (m)','FontName','Times New Roman','FontSize',10);
% title('Minimum Distance to Targets','FontName','Times New Roman','FontSize',10);
% 
% legend({'Reference','Simulation','Hardware'},...
%        'Location','northwest','FontSize',8);


% 
% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data (Joint 1–3 only)
% rmse_data = [
%     0.1283  0.07425 0.07352;
%     0.27088 0.09728 0.08428;
%     0.44934 0.15313 0.10223;
%     1.42940 0.48530 0.23416
% ];
% 
% scenarios = {'Hardware','1.5x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',scenarios);
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Joint Velocity RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint-1','Joint-2','Joint-3'},...
%        'Location','northwest','FontSize',8);
% 



% figure('Color','w','Units','inches','Position',[1 1 4 3]);
% 
% % Data (Joint 1–3 only)
% rmse_data = [
%     0.00504 0.00271 0.00180;
%     0.00846 0.00348 0.00204;
%     0.01235 0.00468 0.00214;
%     0.03336 0.01219 0.00473
% ];
% 
% scenarios = {'Simulation','1.5x','2x','4x'};
% 
% % Plot
% bar(rmse_data);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',scenarios);
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Joint Position RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint-1','Joint-2','Joint-3'},...
%        'Location','northwest','FontSize',8);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% figure('Color','w','Units','inches','Position',[1 1 3.5 3]);
% 
% % Data
% scenarios = {'Hardware','1.5x','2x','4x'};
% min_distance = [0.0019, 0.0020, 0.0002, 0.0095];
% 
% % Plot
% bar(min_distance,'FaceColor',[0.2 0.6 0.8]);
% grid on;
% 
% % Formatting
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',scenarios);
% 
% ylabel('Min. Distance (m)','FontName','Times New Roman','FontSize',10);
% title('Minimum Cartesian Distance to Target','FontName','Times New Roman','FontSize',10);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% figure('Color','w','Units','inches','Position',[1 1 7 3.2]);
% 
% tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
% 
% % ---------------- POSITION RMSE ----------------
% position_rmse = [
%     0.002919 0.000875 0.001546;
%     0.002912 0.001437 0.001359;
%     0.009680 0.003392 0.004839;
%     0.027583 0.008087 0.012069
% ];
% 
% nexttile;
% bar(position_rmse);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});
% 
% ylabel('RMSE (rad)','FontName','Times New Roman','FontSize',10);
% title('Position RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint-1','Joint-2','Joint-3'},'Location','northwest','FontSize',8);
% 
% % ---------------- VELOCITY RMSE ----------------
% velocity_rmse = [
%     0.015024 0.004590 0.008331;
%     0.065922 0.124050 0.071980;
%     0.411331 0.148348 0.205789;
%     1.262030 0.379649 0.574917
% ];
% 
% nexttile;
% bar(velocity_rmse);
% grid on;
% 
% set(gca,'FontName','Times New Roman','FontSize',9,'LineWidth',1);
% set(gca,'XTickLabel',{'Simulation','Hardware-1x','Hardware-2x','Hardware-4x'});
% 
% ylabel('RMSE (rad/s)','FontName','Times New Roman','FontSize',10);
% title('Velocity RMSE','FontName','Times New Roman','FontSize',10);
% 
% legend({'Joint-1','Joint-2','Joint-3'},'Location','northwest','FontSize',8);