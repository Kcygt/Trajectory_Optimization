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
