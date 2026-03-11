

clc;
clear;
close all;

%% Time
dt = 0.01;
t = 0:dt:12;

%% Step levels
s1 = 0.1;
s2 = 0.25;
s3 = 0.4;

%% Step input
u = zeros(size(t));
u(t>=1) = s1;
u(t>=4) = s2;
u(t>=7) = s3;

%% Different filters for each step
tau1 = 1.2;   % slow
tau2 = 0.6;   % medium
tau3 = 0.15;  % fast -> outward convex

sys1 = tf(1,[tau1 1]);
sys2 = tf(1,[tau2 1]);
sys3 = tf(1,[tau3 1]);

%% Create signals for each step
u1 = zeros(size(t));
u2 = zeros(size(t));
u3 = zeros(size(t));

u1(t>=1) = s1;
u2(t>=4) = s2 - s1;
u3(t>=7) = s3 - s2;

%% Filter each step separately
y1 = lsim(sys1,u1,t);
y2 = lsim(sys2,u2,t);
y3 = lsim(sys3,u3,t);

%% Total output
y = y1 + y2 + y3;

%% Colored step segments with rising edges
u_plot1 = nan(size(t));
u_plot2 = nan(size(t));
u_plot3 = nan(size(t));

% Step 1
u_plot1(t>=1 & t<4) = s1;
u_plot1(t>=1-dt & t<1) = 0;

% Step 2
u_plot2(t>=4 & t<7) = s2;
u_plot2(t>=4-dt & t<4) = s1;

% Step 3
u_plot3(t>=7) = s3;
u_plot3(t>=7-dt & t<7) = s2;

%% Plot
figure
hold on
grid on

stairs(t,u_plot1,'LineWidth',3,'Color',[0 0.7 0]) % Step 1
stairs(t,u_plot2,'LineWidth',3,'Color',[0 0 1])   % Step 2
stairs(t,u_plot3,'LineWidth',3,'Color',[1 0 0])   % Step 3

plot(t,y,'k','LineWidth',2) % filtered output

xlabel('Time')
ylabel('Joint position q')
title('Joint Step Input with Prefiltered Output (Outward Convex)')

legend('Step 1','Step 2','Step 3','Prefiltered Output')

%% Step labels
text(2.2, s1 + 0.02, 'S1', 'FontSize', 12, 'FontWeight','bold','Color',[0 0.7 0]);
text(5.2, s2 + 0.02, 'S2', 'FontSize', 12, 'FontWeight','bold','Color',[0 0 1]);
text(9,   s3 + 0.02, 'S3', 'FontSize', 12, 'FontWeight','bold','Color',[1 0 0]);

%% Segment descriptions

text(2.2,0.02,...
{' q_{1j} = [\omega_{1j}, \zeta_{1j}, K_{p_{1j}}, K_{d_{1j}}]'},...
'Interpreter','tex','FontSize',11)

text(5.2,0.14,...
{' q_{2j} = [\omega_{2j}, \zeta_{2j}, K_{p_{2j}}, K_{d_{2j}}]'},...
'Interpreter','tex','FontSize',11)

text(8.6,0.30,...
{' q_{3j} = [\omega_{3j}, \zeta_{3j}, K_{p_{3j}}, K_{d_{3j}}]'},...
'Interpreter','tex','FontSize',11)

% clc;
% clear;
% close all;
% 
% %% Time
% dt = 0.01;
% t = 0:dt:12;
% 
% %% Step levels
% s1 = 0.1;
% s2 = 0.25;
% s3 = 0.4;
% 
% %% Step input
% u = zeros(size(t));
% u(t>=1) = s1;
% u(t>=4) = s2;
% u(t>=7) = s3;
% 
% %% Different filters for each step
% tau1 = 1.2;   % slow
% tau2 = 0.6;   % medium
% tau3 = 0.15;  % fast -> outward convex
% 
% sys1 = tf(1,[tau1 1]);
% sys2 = tf(1,[tau2 1]);
% sys3 = tf(1,[tau3 1]);
% 
% %% Create signals for each step
% u1 = zeros(size(t));
% u2 = zeros(size(t));
% u3 = zeros(size(t));
% 
% u1(t>=1) = s1;
% u2(t>=4) = s2 - s1;
% u3(t>=7) = s3 - s2;
% 
% %% Filter each step separately
% y1 = lsim(sys1,u1,t);
% y2 = lsim(sys2,u2,t);
% y3 = lsim(sys3,u3,t);
% 
% %% Total output
% y = y1 + y2 + y3;
% 
% %% Colored step segments with rising edges
% u_plot1 = nan(size(t));
% u_plot2 = nan(size(t));
% u_plot3 = nan(size(t));
% 
% % Step 1
% u_plot1(t>=1 & t<4) = s1;
% u_plot1(t>=1-dt & t<1) = 0;
% 
% % Step 2
% u_plot2(t>=4 & t<7) = s2;
% u_plot2(t>=4-dt & t<4) = s1;
% 
% % Step 3
% u_plot3(t>=7) = s3;
% u_plot3(t>=7-dt & t<7) = s2;
% 
% %% Plot
% figure
% hold on
% grid on
% 
% stairs(t,u_plot1,'LineWidth',3,'Color',[0 0.7 0]) % Step 1
% stairs(t,u_plot2,'LineWidth',3,'Color',[0 0 1])   % Step 2
% stairs(t,u_plot3,'LineWidth',3,'Color',[1 0 0])   % Step 3
% 
% plot(t,y,'k','LineWidth',2) % filtered output
% 
% xlabel('Time')
% ylabel('Joint position q')
% title('Joint Step Input with Prefiltered Output (Outward Convex)')
% 
% legend('Step 1','Step 2','Step 3','Prefiltered Output')
% 
% %% Add text labels for each step
% text(2.2, s1 + 0.02, 'S1', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0.7 0]);
% text(5.2, s2 + 0.02, 'S2', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0 1]);
% text(9, s3 + 0.02, 'S3', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [1 0 0]);
% % clc;
% % clear;
% % close all;
% % 
% % %% Time
% % dt = 0.01;
% % t = 0:dt:12;
% % 
% % %% Step levels
% % s1 = 0.1;
% % s2 = 0.25;
% % s3 = 0.4;
% % 
% % %% Step input (for simulation)
% % u = zeros(size(t));
% % u(t>=1) = s1;
% % u(t>=4) = s2;
% % u(t>=7) = s3;
% % 
% % %% Different filters for each step
% % tau1 = 1.2;   % slow
% % tau2 = 0.6;   % medium
% % tau3 = 0.25;  % fast
% % 
% % sys1 = tf(1,[tau1 1]);
% % sys2 = tf(1,[tau2 1]);
% % sys3 = tf(1,[tau3 1]);
% % 
% % %% Create signals for each step
% % u1 = zeros(size(t));
% % u2 = zeros(size(t));
% % u3 = zeros(size(t));
% % 
% % u1(t>=1) = s1;
% % u2(t>=4) = s2 - s1;
% % u3(t>=7) = s3 - s2;
% % 
% % %% Filter each step separately
% % y1 = lsim(sys1,u1,t);
% % y2 = lsim(sys2,u2,t);
% % y3 = lsim(sys3,u3,t);
% % 
% % %% Total output
% % y = y1 + y2 + y3;
% % 
% % %% Create colored step segments with rising edges
% % u_plot1 = nan(size(t));
% % u_plot2 = nan(size(t));
% % u_plot3 = nan(size(t));
% % 
% % % Step 1
% % u_plot1(t>=1 & t<4) = s1;
% % u_plot1(t>=1-dt & t<1) = 0;
% % 
% % % Step 2
% % u_plot2(t>=4 & t<7) = s2;
% % u_plot2(t>=4-dt & t<4) = s1;
% % 
% % % Step 3
% % u_plot3(t>=7) = s3;
% % u_plot3(t>=7-dt & t<7) = s2;
% % 
% % %% Plot
% % figure
% % hold on
% % grid on
% % 
% % stairs(t,u_plot1,'LineWidth',3,'Color',[0 0.7 0]) % green
% % stairs(t,u_plot2,'LineWidth',3,'Color',[0 0 1])   % blue
% % stairs(t,u_plot3,'LineWidth',3,'Color',[1 0 0])   % red
% % 
% % plot(t,y,'k','LineWidth',2)
% % 
% % xlabel('Time')
% % ylabel('Joint position q')
% % title('Joint Step Input with Different Prefilter Segments')
% % 
% % legend('Step 1','Step 2','Step 3','Prefiltered Output')