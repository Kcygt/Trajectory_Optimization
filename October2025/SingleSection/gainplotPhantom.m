%% =========================
% Cartesian & Joint Analysis (1000 samples)
%% =========================
clear; clc; close all

%% ---------- Target ----------
target = [0.08, 0.05, 0.04];

%% ---------- Link lengths ----------
l1 = 0.208; 
l2 = 0.168;

%% ---------- Load data ----------
S  = load('Pdata1.mat');    % simulation (actual only)
G1 = load('G1p5data1.mat'); % gain = 1.5
G2 = load('G2data1.mat');   % gain = 2
G4 = load('G4data1.mat');   % gain = 4

%% ---------- Extract joints ----------
q_sim  = S.Pdata(:,7:9);  qd_sim = S.Pdata(:,10:12);
q_g1   = G1.G1p5data1(:,7:9); qd_g1 = G1.G1p5data1(:,10:12);
q_g2   = G2.G2data1(:,7:9);   qd_g2 = G2.G2data1(:,10:12);
q_g4   = G4.G4data1(:,7:9);   qd_g4 = G4.G4data1(:,10:12);

%% ---------- Time vectors ----------
if isfield(S,'tOpt')
    tS = S.tOpt(:);
else
    tS = (0:size(q_sim,1)-1)'; 
end
tG1 = linspace(0, tS(end)/1.5, length(q_g1))';
tG2 = linspace(0, tS(end)/2,   length(q_g2))';
tG4 = linspace(0, tS(end)/4,   length(q_g4))';

%% ---------- Uniform 1000-sample time grid ----------
tUni = linspace(0, tS(end), 1000)';

q_sim  = interp1(tS,  q_sim,  tUni, 'linear'); 
qd_sim = interp1(tS,  qd_sim, tUni, 'linear');
q_g1   = interp1(tG1, q_g1,   tUni, 'linear');
qd_g1  = interp1(tG1, qd_g1,  tUni, 'linear');
q_g2   = interp1(tG2, q_g2,   tUni, 'linear');
qd_g2  = interp1(tG2, qd_g2,  tUni, 'linear');
q_g4   = interp1(tG4, q_g4,   tUni, 'linear');
qd_g4  = interp1(tG4, qd_g4,  tUni, 'linear');

%% ---------- Colors ----------
colors = {[0.2 0.6 1.0], [0 0.8 0], [1 0.5 0], [0.7 0 0.7]}; % sim, g1, g2, g4
startColor = [0 0 1]; finalColor = [1 0 0]; targetColor = [0 0 0];
labels = {'Hardware','1.5x faster','2x faster','4x faster'};

%% ==========================================================
%  JOINT POSITIONS
%% ==========================================================
figure('Name','Joint Positions');
for j = 1:3
    subplot(3,1,j); hold on; grid on;
    plot(tUni, q_sim(:,j), '-', 'Color', colors{1}, 'LineWidth', 1.8);
    plot(tUni, q_g1(:,j),  '-', 'Color', colors{2}, 'LineWidth', 1.5);
    plot(tUni, q_g2(:,j),  '-', 'Color', colors{3}, 'LineWidth', 1.5);
    plot(tUni, q_g4(:,j),  '-', 'Color', colors{4}, 'LineWidth', 1.5);
    ylabel(sprintf('$\\mathbf{q_{%d} (rad)}$', j), 'Interpreter', 'latex');
    if j==3, xlabel('Time (s)'); end
    title(sprintf('Joint %d Position', j));
end
legend(labels,'Location','bestoutside');

%% ==========================================================
%  JOINT VELOCITIES
%% ==========================================================
figure('Name','Joint Velocities');
for j = 1:3
    subplot(3,1,j); hold on; grid on;
    plot(tUni, qd_sim(:,j), '-', 'Color', colors{1}, 'LineWidth', 1.8);
    plot(tUni, qd_g1(:,j),  '-', 'Color', colors{2}, 'LineWidth', 1.5);
    plot(tUni, qd_g2(:,j),  '-', 'Color', colors{3}, 'LineWidth', 1.5);
    plot(tUni, qd_g4(:,j),  '-', 'Color', colors{4}, 'LineWidth', 1.5);
    ylabel(sprintf('$\\mathbf{\\dot{q}_{%d} (rad/s)}$', j), 'Interpreter', 'latex');
    if j==3, xlabel('Time (s)'); end
    title(sprintf('Joint %d Velocity', j));
end
legend(labels,'Location','bestoutside');

%% ==========================================================
%  CARTESIAN TRAJECTORIES + MIN DISTANCE
%% ==========================================================
FK = @(q) [ ...
    sin(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))), ...
    l2 - l2*cos(q(:,3)) + l1*sin(q(:,2)), ...
    -l1 + cos(q(:,1)).*(l1*cos(q(:,2)) + l2*sin(q(:,3))) ];

XYZ = cell(1,4);
XYZ{1} = FK(q_sim); XYZ{2} = FK(q_g1); XYZ{3} = FK(q_g2); XYZ{4} = FK(q_g4);

% Minimum distances
dmin = zeros(1,4); idxMin = zeros(1,4);
for k = 1:4
    d = vecnorm(XYZ{k}-target,2,2);
    [dmin(k), idxMin(k)] = min(d);
end

%% 3D Cartesian Plot
figure('Name','3D Cartesian Trajectories'); hold on; grid on; axis equal; view(3);

for k = 1:4
    plot3(XYZ{k}(:,1), XYZ{k}(:,2), XYZ{k}(:,3), '-', 'Color', colors{k}, ...
          'LineWidth', 2, 'DisplayName', labels{k});
end

% Start, Final, Target points
startPt = [0 0 0]; finalPt = [0.1 0.1 0.1];
plot3(startPt(1), startPt(2), startPt(3), 'o','MarkerSize',7,'MarkerFaceColor',startColor,'MarkerEdgeColor','k','DisplayName','Start Point');
plot3(finalPt(1), finalPt(2), finalPt(3), 's','MarkerSize',7,'MarkerFaceColor',finalColor,'MarkerEdgeColor','k','DisplayName','Final Point');
plot3(target(1), target(2), target(3), 'p','MarkerSize',12,'MarkerFaceColor',targetColor,'MarkerEdgeColor','k','DisplayName','Target');

% Projection planes
planes.z = min(cellfun(@(x) min(x(:,3)), XYZ));
planes.y = min(cellfun(@(x) min(x(:,2)), XYZ));
planes.x = max(cellfun(@(x) max(x(:,1)), XYZ)) + 0.01;

% Trajectory projections
for k = 1:4
    add3DProjections(gca, XYZ{k}(:,1), XYZ{k}(:,2), XYZ{k}(:,3), planes, colors{k});
end

% Projections of start, final, target points (once)
addPointProjections(gca, startPt, planes, startColor, false);
addPointProjections(gca, finalPt, planes, finalColor, true);
addTargetProjections(gca, target, planes, targetColor);

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Cartesian Space Trajectories');
legend('Location','bestoutside');

%% ---------- Print minimum distances ----------
fprintf('\nMinimum Cartesian distance to TARGET:\n');
for k = 1:4
    fprintf('  %s : %.6g (at idx %d)\n', labels{k}, dmin(k), idxMin(k));
end

%% =========================
% Subfunctions
%% =========================
function add3DProjections(ax,X,Y,Z,planes,color)
    plot3(ax,X,Y,planes.z*ones(size(X)),':','Color',color,'HandleVisibility','off');
    plot3(ax,X,planes.y*ones(size(X)),Z,':','Color',color,'HandleVisibility','off');
    plot3(ax,planes.x*ones(size(X)),Y,Z,':','Color',color,'HandleVisibility','off');
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
