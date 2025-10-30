load('Pdata.mat')

time = 0:0.001:5;  % 1x5001 vector
[x,y,z] = FK(Pdata(:,1),Pdata(:,2),Pdata(:,3));
Fz = Pdata(:,6);

xTarget =[0.0100   -0.0300         0;
0.0400   -0.0300         0;
0.0700   -0.0300         0;
0.1000   -0.0300         0;
0.1300   -0.0300         0];

xCtrl(1,:) = [ -0.0125  ,  -0.0336   -0.0021 ];
xCtrl(2,:) = [ 0.0560 ,  -0.0280  , -0.0010 ];
xCtrl(3,:) = [  0.1538 ,  -0.0305  ,  0.0071 ];

figure; hold on; grid on;
plot3(x,y,z)
plot3(0,0,0,'o')
plot3(xCtrl(:,1),xCtrl(:,2),xCtrl(:,3),'*')
plot3(xTarget(:,1),xTarget(:,2),xTarget(:,3),'*')

figure; grid on; hold on;
plot(time,Fz)



% Find the index where Fz first drops below -0.2
Fidx = find(Fz < -0.22, 1, 'first');
Sidx = find(Fz(Fidx:end) > -0.22, 1, 'last');

% Get the corresponding time
if ~isempty(Fidx)
    first_time_below_threshold = time(Fidx);
    fprintf('First time Fz < -0.2 is at t = %.3f seconds\n', first_time_below_threshold);
else
    disp('Fz never drops below -0.2.');
end

% Find the index of minimum distance between trajectory and target point
% Assuming you want to find distance to the first target point (xTarget(1,:))
target_point = xTarget(1,:); % [0.0100, -0.0300, 0]

% Calculate distances from each trajectory point to the target
distances = sqrt((x - target_point(1)).^2 + (y - target_point(2)).^2 + (z - target_point(3)).^2);

% Find the index of minimum distance
[min_distance, min_distance_idx] = min(distances);

fprintf('Minimum distance to target point [%.4f, %.4f, %.4f] is %.6f at index %d\n', ...
    target_point(1), target_point(2), target_point(3), min_distance, min_distance_idx);
fprintf('Corresponding time: %.3f seconds\n', time(min_distance_idx));
fprintf('Trajectory position at minimum distance: [%.6f, %.6f, %.6f]\n', ...
    x(min_distance_idx), y(min_distance_idx), z(min_distance_idx));

% If you want to find minimum distance to all target points
fprintf('\n--- Minimum distances to all target points ---\n');
for i = 1:size(xTarget, 1)
    target_point_i = xTarget(i,:);
    distances_i = sqrt((x - target_point_i(1)).^2 + (y - target_point_i(2)).^2 + (z - target_point_i(3)).^2);
    [min_dist_i, min_idx_i] = min(distances_i);
    
    fprintf('Target %d [%.4f, %.4f, %.4f]: min distance = %.6f at index %d (t = %.3f s)\n', ...
        i, target_point_i(1), target_point_i(2), target_point_i(3), min_dist_i, min_idx_i, time(min_idx_i));
end