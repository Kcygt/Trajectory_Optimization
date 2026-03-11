clc
clear
close all

%% Time limits
ts = 0;
t1 = 2;
t2 = 4;
tf = 6;

t_seg1 = linspace(ts,t1,100);
t_seg2 = linspace(t1,t2,100);
t_seg3 = linspace(t2,tf,100);

%% Segment 1 (orange)
x1 = t_seg1;
y1 = 0.5*sin(0.8*t_seg1);

%% Segment 2 (green) - connected
x2 = x1(end) + 0.2*(t_seg2 - t1);
y2 = y1(end) + 1.5*(t_seg2 - t1);

%% Segment 3 (purple) - connected
x3 = x2(end) + 0.5*(t_seg3 - t2);
y3 = y2(end) + 0.6*sin(1.5*(t_seg3 - t2));

%% Plot
figure; grid on;
hold on

plot(x1,y1,'LineWidth',4,'Color',[1 0.4 0])     % orange
plot(x2,y2,'LineWidth',4,'Color',[0 0.8 0])     % green
plot(x3,y3,'LineWidth',4,'Color',[0.6 0 0.8])   % purple

plot(x1(1),y1(1),'ko','MarkerFaceColor','k')
plot(x3(end),y3(end),'ko','MarkerFaceColor','k')

text(x1(1),y1(1),'  Start')
text(x3(end),y3(end),'  Final')
%% Segment text labels

text(mean(x1),mean(y1)-0.7,...
{' \bfSegment 1', ...
' \rmt_s < t < t_1', ...
' \rmq_{1j} = [\omega_{1j}, \zeta_{1j}, Kp_{1j}, Kd_{1j}]'},...
'Interpreter','tex')

text(mean(x2)+0.2,mean(y2),...
{' \bfSegment 2', ...
' \rmt_1 < t < t_2', ...
' \rmq_{2j} = [\omega_{2j}, \zeta_{2j}, Kp_{2j}, Kd_{2j}]'},...
'Interpreter','tex')

text(mean(x3)+0.5,mean(y3)+0.6,...
{' \bfSegment 3', ...
' \rmt_2 < t < t_f', ...
' \rmq_{3j} = [\omega_{3j}, \zeta_{3j}, Kp_{3j}, Kd_{3j}]'},...
'Interpreter','tex')


legend('Segment 1','Segment 2','Segment 3')