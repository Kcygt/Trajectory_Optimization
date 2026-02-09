%% =========================
% Full analysis: joint plots + 3D trajectories
%% =========================
clear; clc; close all;

%% ------------------------
% Load data
%% ------------------------
S  = load('Sdata1.mat');   % Simulation
G1 = load('SG1data1.mat'); % 1.5x faster
G2 = load('SG2data1.mat'); % 2x faster
G4 = load('SG4data1.mat'); % 4x faster

%% ------------------------
% Joint positions / velocities
%% ------------------------
qS  = S.yOpt(:,7:9);  qdS  = S.yOpt(:,10:12);
qG1 = G1.yInit(:,7:9); qdG1 = G1.yInit(:,10:12);
qG2 = G2.yInit(:,7:9); qdG2 = G2.yInit(:,10:12);
qG4 = G4.yInit(:,7:9); qdG4 = G4.yInit(:,10:12);


qSD  = S.yOpt(:,1:3);  qdSD  = S.yOpt(:,7:9);
qG1D = G1.yInit(:,1:3); qdG1D = G1.yInit(:,7:9);
qG2D = G2.yInit(:,1:3); qdG2D = G2.yInit(:,7:9);
qG4D = G4.yInit(:,1:3); qdG4D = G4.yInit(:,7:9);

rmse_pos_joint1 = sqrt(mean((qS - qSD).^2, 1));
rmse_pos_joint2 = sqrt(mean((qG1 - qG1D).^2, 1));
rmse_pos_joint3 = sqrt(mean((qG2 - qG2D).^2, 1));
rmse_pos_joint4 = sqrt(mean((qG4 - qG4D).^2, 1));

rmse_vel_joint1 = sqrt(mean((qdS - qdSD).^2, 1));
rmse_vel_joint2 = sqrt(mean((qdG1 - qdG1D).^2, 1));
rmse_vel_joint3 = sqrt(mean((qdG2 - qdG2D).^2, 1));
rmse_vel_joint4 = sqrt(mean((qdG4 - qdG4D).^2, 1));




%% ------------------------
% RMSE Table: Joint Position & Velocity
%% ------------------------

% Joint datasets
datasets = {'S','G1','G2','G4'};

% RMSE values (per joint)
RMSE_pos = [rmse_pos_joint1; rmse_pos_joint2; rmse_pos_joint3; rmse_pos_joint4];
RMSE_vel = [rmse_vel_joint1; rmse_vel_joint2; rmse_vel_joint3; rmse_vel_joint4];

% Compute scalar RMSE (all joints combined)
RMSE_pos_total = sqrt(sum(RMSE_pos.^2,2)/3);  % sqrt(mean of squares)
RMSE_vel_total = sqrt(sum(RMSE_vel.^2,2)/3);

% Create table
T = table(datasets', ...
    RMSE_pos(:,1), RMSE_pos(:,2), RMSE_pos(:,3), RMSE_pos_total, ...
    RMSE_vel(:,1), RMSE_vel(:,2), RMSE_vel(:,3), RMSE_vel_total, ...
    'VariableNames', {'Dataset', ...
                      'Pos_RMSE_q1','Pos_RMSE_q2','Pos_RMSE_q3','Pos_RMSE_total', ...
                      'Vel_RMSE_q1','Vel_RMSE_q2','Vel_RMSE_q3','Vel_RMSE_total'});

disp('==== RMSE: Joint Position & Velocity ====')
disp(T)





% Time vectors
if isfield(S,'tOpt'), tS = S.tOpt(:); else tS = (0:size(qS,1)-1)'; end
tG1 = linspace(0, tS(end)/1.5, size(qG1,1))';
tG2 = linspace(0, tS(end)/2,   size(qG2,1))';
tG4 = linspace(0, tS(end)/4,   size(qG4,1))';

% Interpolate to 1000 samples
tUni  = linspace(0,tS(end),1000)';
qS    = interp1(tS, qS, tUni, 'linear');  qdS  = interp1(tS, qdS, tUni, 'linear');
qG1   = interp1(tG1,qG1,tUni,'linear');   qdG1 = interp1(tG1,qdG1,tUni,'linear');
qG2   = interp1(tG2,qG2,tUni,'linear');   qdG2 = interp1(tG2,qdG2,tUni,'linear');
qG4   = interp1(tG4,qG4,tUni,'linear');   qdG4 = interp1(tG4,qdG4,tUni,'linear');

%% ------------------------
% 3D Cartesian Trajectories
%% ------------------------
colors = {[0 0.6 0],[255 128 0]/255,[0 0.8 0.8],[255 50 152]/255}; % Sim,G1,G2,G4
labels = {'Simulation','1.5x faster','2x faster','4x faster'};

l1 = 0.208; l2 = 0.168;
FK = @(q) [ ...
    sin(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))), ...
    l2 - l2*cos(q(:,3)) + l1*sin(q(:,2)), ...
    -l1 + cos(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))) ];

XYZ_S  = FK(qS);
XYZ_G1 = FK(qG1);
XYZ_G2 = FK(qG2);
XYZ_G4 = FK(qG4);

startPt = [0 0 0];
finalPt = [0.1 0.1 0.1];
target  = [0.08 0.05 0.04];

figure('Name','3D Cartesian Trajectories');
ax = axes; hold on; grid on; view(3); axis equal;
xlabel('$X$ (m)','Interpreter','latex'); ylabel('$Y$ (m)','Interpreter','latex'); zlabel('$Z$ (m)','Interpreter','latex');
title('Cartesian Space Trajectories');

% Trajectories
hS  = plot3(XYZ_S(:,1), XYZ_S(:,2), XYZ_S(:,3), '-', 'LineWidth',2, 'Color', colors{1}, 'DisplayName',labels{1});
hG1 = plot3(XYZ_G1(:,1), XYZ_G1(:,2), XYZ_G1(:,3), '-', 'LineWidth',2, 'Color', colors{2}, 'DisplayName',labels{2});
hG2 = plot3(XYZ_G2(:,1), XYZ_G2(:,2), XYZ_G2(:,3), '-', 'LineWidth',2, 'Color', colors{3}, 'DisplayName',labels{3});
hG4 = plot3(XYZ_G4(:,1), XYZ_G4(:,2), XYZ_G4(:,3), '-', 'LineWidth',2, 'Color', colors{4}, 'DisplayName',labels{4});

% Start / Final points
startColor = [0 0 1]; finalColor = [1 0 0]; targetColor = [0 0 0];
plot3(startPt(1), startPt(2), startPt(3), 'o', 'MarkerFaceColor',startColor, 'MarkerEdgeColor','k','MarkerSize',7,'DisplayName','Start Point');
plot3(finalPt(1), finalPt(2), finalPt(3), 's', 'MarkerFaceColor',finalColor, 'MarkerEdgeColor','k','MarkerSize',7,'DisplayName','Final Point');
plot3(target(1), target(2), target(3), 'p','MarkerFaceColor',targetColor, 'MarkerEdgeColor','k','MarkerSize',12,'DisplayName','Target');

% Projection planes
drawnow;
xl = xlim; yl = ylim; zl = zlim;
offset = 0.01; planes.z = zl(1); planes.y = yl(1); planes.x = xl(2)+offset;
xlim([xl(1),planes.x+0.01]);

% Add projections for all trajectories
add3DProjections(ax, XYZ_S(:,1), XYZ_S(:,2), XYZ_S(:,3), planes, colors{1});
add3DProjections(ax, XYZ_G1(:,1), XYZ_G1(:,2), XYZ_G1(:,3), planes, colors{2});
add3DProjections(ax, XYZ_G2(:,1), XYZ_G2(:,2), XYZ_G2(:,3), planes, colors{3});
add3DProjections(ax, XYZ_G4(:,1), XYZ_G4(:,2), XYZ_G4(:,3), planes, colors{4});

% Project start/final/target points with same colors
addPointProjections(ax, startPt, planes, startColor, false);
addPointProjections(ax, finalPt, planes, finalColor, true);
addTargetProjections(ax, target, planes, targetColor);

legend('Location','eastoutside','Interpreter','latex');

%% ------------------------
% Plot Joint Positions
%% ------------------------
figure('Name','Joint Positions');
for j=1:3
    subplot(3,1,j); hold on; grid on;
    plot(tUni,qS(:,j),'-','Color',colors{1},'LineWidth',1.8);
    plot(tUni,qG1(:,j),'-','Color',colors{2},'LineWidth',1.5);
    plot(tUni,qG2(:,j),'-','Color',colors{3},'LineWidth',1.5);
    plot(tUni,qG4(:,j),'-','Color',colors{4},'LineWidth',1.5);
    ylabel(sprintf('$q_{%d}$ (rad)',j),'Interpreter','latex');
    if j==3, xlabel('Time (s)'); end
    title(sprintf('Joint %d Position',j));
end
legend(labels,'Location','bestoutside');

%% ------------------------
% Plot Joint Velocities
%% ------------------------
figure('Name','Joint Velocities');
for j=1:3
    subplot(3,1,j); hold on; grid on;
    plot(tUni,qdS(:,j),'-','Color',colors{1},'LineWidth',1.8);
    plot(tUni,qdG1(:,j),'-','Color',colors{2},'LineWidth',1.5);
    plot(tUni,qdG2(:,j),'-','Color',colors{3},'LineWidth',1.5);
    plot(tUni,qdG4(:,j),'-','Color',colors{4},'LineWidth',1.5);
    ylabel(sprintf('$\\dot{q}_{%d}$ (rad/s)',j),'Interpreter','latex');
    if j==3, xlabel('Time (s)'); end
    title(sprintf('Joint %d Velocity',j));
end
legend(labels,'Location','bestoutside');

%% ------------------------
% Subfunctions
%% ------------------------
function add3DProjections(ax,X,Y,Z,planes,color)
    plot3(ax,X,Y,planes.z*ones(size(X)),':','Color',color,'HandleVisibility','off'); % XY
    plot3(ax,X,planes.y*ones(size(X)),Z,':','Color',color,'HandleVisibility','off'); % XZ
    plot3(ax,planes.x*ones(size(X)),Y,Z,':','Color',color,'HandleVisibility','off'); % YZ
    plot3(ax,X(1),Y(1),planes.z,'o','MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
    plot3(ax,X(end),Y(end),planes.z,'s','MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
end

function addTargetProjections(ax,pt,planes,color)
    plot3(ax,pt(1),pt(2),planes.z,'p','MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
    plot3(ax,pt(1),planes.y,pt(3),'p','MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
    plot3(ax,planes.x,pt(2),pt(3),'p','MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
end

function addPointProjections(ax,pt,planes,color,isFinal)
    mrk='o'; if isFinal, mrk='s'; end
    plot3(ax,pt(1),pt(2),planes.z,mrk,'MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
    plot3(ax,pt(1),planes.y,pt(3),mrk,'MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
    plot3(ax,planes.x,pt(2),pt(3),mrk,'MarkerFaceColor',color,'MarkerEdgeColor','k','HandleVisibility','off');
end


