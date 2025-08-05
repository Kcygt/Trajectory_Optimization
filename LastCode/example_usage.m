%% Example Usage of Flexible Trajectory Optimization
% This file shows how to use the flexible optimization with different numbers of control points

clear; clc; close all;

%% Example 1: 2 Control Points (Original setup)
fprintf('=== Example 1: 2 Control Points ===\n');
xTarget = [...
    0.005, -0.03, 0.0;
    0.01, -0.03, 0.0;
    0.015, -0.03, 0.0;
    0.02, -0.03, 0.0;
    0.025, -0.03, 0.0];
controlPointIndices = [1, 5]; % Use first and last targets as control points

% This will create:
% - 2 control points
% - 3 phases (2 control point phases + 1 final phase)
% - 3 wn parameters (wn1, wn2, wn3)
% - 6 optimization parameters for wn (3 per phase)
% - 6 optimization parameters for control points (3 per control point)
% Total: 1 (time) + 6 (wn) + 6 (control points) = 13 parameters

%% Example 2: 3 Control Points
fprintf('\n=== Example 2: 3 Control Points ===\n');
xTarget = [...
    0.005, -0.03, 0.0;
    0.01, -0.03, 0.0;
    0.015, -0.03, 0.0;
    0.02, -0.03, 0.0;
    0.025, -0.03, 0.0;
    0.03, -0.03, 0.0;
    0.035, -0.03, 0.0];
controlPointIndices = [1, 4, 7]; % Use first, middle, and last targets

% This will create:
% - 3 control points
% - 4 phases (3 control point phases + 1 final phase)
% - 4 wn parameters (wn1, wn2, wn3, wn4)
% - 12 optimization parameters for wn (3 per phase)
% - 9 optimization parameters for control points (3 per control point)
% Total: 1 (time) + 12 (wn) + 9 (control points) = 22 parameters

%% Example 3: 4 Control Points
fprintf('\n=== Example 3: 4 Control Points ===\n');
xTarget = [...
    0.005, -0.03, 0.0;
    0.01, -0.03, 0.0;
    0.015, -0.03, 0.0;
    0.02, -0.03, 0.0;
    0.025, -0.03, 0.0;
    0.03, -0.03, 0.0;
    0.035, -0.03, 0.0;
    0.04, -0.03, 0.0;
    0.045, -0.03, 0.0];
controlPointIndices = [1, 3, 6, 9]; % Use 4 evenly spaced targets

% This will create:
% - 4 control points
% - 5 phases (4 control point phases + 1 final phase)
% - 5 wn parameters (wn1, wn2, wn3, wn4, wn5)
% - 15 optimization parameters for wn (3 per phase)
% - 12 optimization parameters for control points (3 per control point)
% Total: 1 (time) + 15 (wn) + 12 (control points) = 28 parameters

%% Example 4: Single Control Point
fprintf('\n=== Example 4: Single Control Point ===\n');
xTarget = [...
    0.005, -0.03, 0.0;
    0.01, -0.03, 0.0;
    0.015, -0.03, 0.0];
controlPointIndices = [2]; % Use only middle target

% This will create:
% - 1 control point
% - 2 phases (1 control point phase + 1 final phase)
% - 2 wn parameters (wn1, wn2)
% - 6 optimization parameters for wn (3 per phase)
% - 3 optimization parameters for control points (3 per control point)
% Total: 1 (time) + 6 (wn) + 3 (control points) = 10 parameters

%% How to Use:
% 1. Copy the flexible_trajectory_optimization.m file
% 2. Modify the xTarget matrix with your desired targets
% 3. Modify the controlPointIndices array to specify which targets to use as control points
% 4. Run the optimization

fprintf('\n=== Summary ===\n');
fprintf('The flexible optimization automatically handles:\n');
fprintf('- Any number of target points\n');
fprintf('- Any number of control points (up to number of targets)\n');
fprintf('- Dynamic parameter vector sizing\n');
fprintf('- Dynamic bounds generation\n');
fprintf('- Flexible phase transitions\n');
fprintf('- Automatic wn parameter allocation\n'); 