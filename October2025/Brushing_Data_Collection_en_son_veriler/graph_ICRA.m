% subplot(2,1,1),
% hold on, grid off, xlabel('x-Direction [m]'), ylabel('z-Direction [m]'), axis equal
% g1=plot(x_d_star(:,1),x_d_star(:,3),'r',LineWidth=2);
% g2=plot(trajectory_first(2:end,3),trajectory_first(2:end,5),'Color',"#7E2F8E",LineWidth=2);
% % g3=plot(trajectory_second(2:end,3),trajectory_second(2:end,5),'Color',"#EDB120",LineWidth=2);
% % g4=plot(trajectory_third(2:end,3),trajectory_third(2:end,5),'Color',"#4DBEEE",LineWidth=2);
% %legend([g1 g2 g3 g4],'The Given Trajectory','The First Followed Trajectory','The Second Followed Trajectory','The Third Followed Trajectory','Location','northeastoutside','Orientation','vertical')
% 
% yy1=[trajectory_first(2:end,3)];
% yy2=[trajectory_first(2:end,5)];
% % plot(yy1(1),yy2(1),'g->')
% pp5=gca;
% tic
% for jj=1:2000:60000
%     toc
%     
%     pp5.Children(1).XData=yy1(jj);
%     pp5.Children(1).YData=yy2(jj);pause(.25);
% end

% myVideo = VideoWriter('myVideoFile'); %open video file
% myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
% myVideo.Quality=100;
% open(myVideo)


% subplot(2,1,2),
% hold on, grid off, xlabel('Time [s]'), ylabel('Force [N]'),%title('Force Measurement')
 plot(trajectory_first(2:end,2),trajectory_first(2:end,12), 'Color',"#7E2F8E",'LineWidth',0.5);
%  p2=plot(trajectory_second(2:end,2),trajectory_second(2:end,12), 'r','LineWidth',0.5);
%  p3=plot(trajectory_third(2:end,2),trajectory_third(2:end,12), 'r','LineWidth',0.5);
% 
hold on 
plot(t_vec, f_d_star(:, 3),'r','LineWidth',2);
%  plot(t_vec+32, f_d_star(:, 3),'b','LineWidth',2);
%  plot(t_vec+64, f_d_star(:, 3),'b','LineWidth',2)



h=gcf;
set(h,'PaperOrientation','landscape');


print('The Declined surface force failed','-dpdf','-fillpage')
% print('The Declined Ramp surface - Second loop','-dpdf','-fillpage')
% print('The Declined Ramp surface - third loop','-dpdf','-fillpage')

% for jj=1:10:66740
%     p1.XData(jj);
%     p1.YData(jj);
%     pause(1)
% end
%



% 
% xx1=[trajectory_first(2:end,2)];% trajectory_second(2:end,2); trajectory_third(2:end,2)];
% xx2=[trajectory_first(2:end,12)];% trajectory_second(2:end,12); trajectory_third(2:end,12)];
% 
% plot(xx1,xx2,'r')
% plot(t_vec, f_d_star(:, 3),'b','LineWidth',2);
% % plot(t_vec+32, f_d_star(:, 3),'b','LineWidth',2);
% % plot(t_vec+64, f_d_star(:, 3),'b','LineWidth',2);
% plot(xx1(1),xx2(1),'go',MarkerSize=10,MarkerFaceColor='green');
% p6=gca;
% tic
% for jj=1:200:66700
%     toc
% %     pause(0.1) %Pause and grab frame
% %     frame = getframe(gcf); %get frame
% %     writeVideo(myVideo, frame);
% %     
% %     pp5.Children(1).XData=yy1(jj);
% %     pp5.Children(1).YData=yy2(jj);pause(.25);
%     p6.Children(1).XData=xx1(jj);
%     p6.Children(1).YData=xx2(jj);pause(.25);
% 
% 
% end
% close(myVideo)
% legend([p1 p2],{'The Measured Force','The Desired Force'})
