


%% ===============================
% Trajectory and Force Plot for 3 Cycles
% ===============================

%% ----------- Trajectory Plot -----------
figure(1); clf
hold on; grid on; axis equal
xlabel('x-Direction [m]')
ylabel('z-Direction [m]')

% Desired trajectory
plot(x_d_star(:,1), x_d_star(:,3),'k','LineWidth',3);

% Trajectory cycles
plot(trajectory_first(2:end,3),  trajectory_first(2:end,5), 'Color',"#7E2F8E",'LineWidth',1.8);
plot(trajectory_second(2:end,3), trajectory_second(2:end,5), 'Color',"#EDB120",'LineWidth',1.8);
plot(trajectory_third(2:end,3),  trajectory_third(2:end,5), 'Color',"#4DBEEE",'LineWidth',1.8);

% Home position
plot(0.06, -0.195, 'ko', 'MarkerSize',8, 'MarkerFaceColor','r');

% Legend
legend('Reference Trajectory','Iteration 1','Iteration 2','Iteration 3','Home Position','Location','best')

% Paper settings
set(gcf,'PaperOrientation','landscape');
print('Trajectory_3cycles','-dpdf','-fillpage')


%% ----------- Force Plot -----------
figure(2); clf
hold on; grid on
xlabel('Time [s]')
ylabel('Force [N]')

% Plot measured forces
p1 = plot(trajectory_first(2:end,2),  trajectory_first(2:end,12),'Color',"#7E2F8E",'LineWidth',0.9);
p2 = plot(trajectory_second(2:end,2), trajectory_second(2:end,12),'Color',"#EDB120",'LineWidth',0.9);
p3 = plot(trajectory_third(2:end,2),  trajectory_third(2:end,12),'Color',"#4DBEEE",'LineWidth',0.9);

%% ----------- Continuous Reference Force -----------

% Determine total experiment time
t_end = max([trajectory_first(end,2), trajectory_second(end,2), trajectory_third(end,2)]);
t_ref = linspace(0, t_end, length(t_vec)*3)';   % continuous timeline
f_ref = zeros(size(t_ref));

% Cycle parameters
cycle_duration = 72; % seconds per cycle

for k = 0:2  % 3 cycles
    t0 = k*cycle_duration;  % cycle start time
    
    if k == 0
        % First cycle: force starts at 0 s for 1 N (0–8 s)
        f_ref(t_ref >= t0 & t_ref < t0+8) = 1;
        % Also 1 N at 32–40 s in first cycle
        f_ref(t_ref >= t0+32 & t_ref < t0+40) = 1;
        f_ref(t_ref >= t0+64 & t_ref < t0+72) = 1;

    else
        % Subsequent cycles
        f_ref(t_ref >= t0+32 & t_ref < t0+40) = 1;
        f_ref(t_ref >= t0+64 & t_ref < t0+72) = 1;
    end
end

% Plot reference force
pref = plot(t_ref, f_ref,'k','LineWidth',2.5);
uistack(pref,'top')  % ensure reference stays on top

% Legend
legend([p1 p2 p3 pref],{'Iteration 1','Iteration 2','Iteration 3','Reference Force'},'Location','best')

% Paper settings
set(gcf,'PaperOrientation','landscape');
print('Force_3cycles','-dpdf','-fillpage')






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %% ===============================
% % Trajectory and Force Plot for 3 Cycles
% % ===============================
% 
% %% ----------- Trajectory Plot -----------
% figure(1); clf
% hold on; grid on; axis equal
% xlabel('x-Direction [m]')
% ylabel('z-Direction [m]')
% 
% % Desired trajectory
% plot(x_d_star(:,1), x_d_star(:,3),'k','LineWidth',3);
% 
% % Trajectory cycles
% plot(trajectory_first(2:end,3),  trajectory_first(2:end,5), 'Color',"#7E2F8E",'LineWidth',1.8);
% plot(trajectory_second(2:end,3), trajectory_second(2:end,5), 'Color',"#EDB120",'LineWidth',1.8);
% plot(trajectory_third(2:end,3),  trajectory_third(2:end,5), 'Color',"#4DBEEE",'LineWidth',1.8);
% 
% % Home position
% plot(0.06, -0.195, 'ko', 'MarkerSize',8, 'MarkerFaceColor','g');
% 
% % Legend
% legend('Desired','Iteration 1','Iteration 2','Iteration 3','Home Position','Location','best')
% 
% % Paper settings
% set(gcf,'PaperOrientation','landscape');
% print('Trajectory_3cycles','-dpdf','-fillpage')
% 
% 
% %% ----------- Force Plot -----------
% 
% figure(2); clf
% hold on; grid on
% xlabel('Time [s]')
% ylabel('Force [N]')
% 
% % Plot measured forces
% p1 = plot(trajectory_first(2:end,2),  trajectory_first(2:end,12),'Color',"#7E2F8E",'LineWidth',0.9);
% p2 = plot(trajectory_second(2:end,2), trajectory_second(2:end,12),'Color',"#EDB120",'LineWidth',0.9);
% p3 = plot(trajectory_third(2:end,2),  trajectory_third(2:end,12),'Color',"#4DBEEE",'LineWidth',0.9);
% 
% %% Create continuous reference force
% % Determine total experiment time
% t_end = max([trajectory_first(end,2), trajectory_second(end,2), trajectory_third(end,2)]);
% t_ref = linspace(0, t_end, length(t_vec)*3)';   % continuous timeline
% 
% f_ref = zeros(size(t_ref));
% 
% cycle_duration = 72; % seconds per cycle
% for k = 0:2   % 3 cycles
%     t0 = k*cycle_duration;
% 
%     % Reference force profile
%     f_ref(t_ref >= t0+32 & t_ref < t0+40) = 1;
%     f_ref(t_ref >= t0+64 & t_ref < t0+72) = 1;
% end
% 
% % Plot reference force
% pref = plot(t_ref, f_ref,'k--','LineWidth',2.5);
% uistack(pref,'top')  % force reference to top
% 
% % Legend
% legend([p1 p2 p3 pref],{'Iteration 1','Iteration 2','Iteration 3','Reference'},'Location','best')
% 
% % Paper settings
% set(gcf,'PaperOrientation','landscape');
% print('Force_3cycles','-dpdf','-fillpage')
