clc;
clear;
close all;
%% --- Load Data ---
load("Pdata7.mat")   % Make sure Pdata7 exists in workspace

%% --- Calibration Matrix (6x6) ---
nanoCalibMatrix = [0.026350, 0.057250, -0.168980, -3.362720, 0.1599000, 3.37204;
                   0.039660, 3.972550, -0.152510, -1.870750, -0.079070, -2.01298;
                   3.759480, -0.06271, 3.785160,  -0.056060, 3.7825700, -0.19971;
                  -0.515690, 24.65602, 19.68848,  -11.89616, -21.76668, -11.4045;
                  -24.92166, 0.092610, 12.63655,   20.64617, 11.455150, -21.56128;
                   0.091560, 14.93053, 0.648980,   14.11840, 0.8499500, 15.119190];

%% --- Offset Vector (6x1) ---
NanoOffset = [-0.0437359113489638;
               0.756998832187297;
               0.180791924759231;
               0.552210975876987;
               0.0372213444991813;
               0.361092663140281];

%% --- Extract Raw Sensor Output (Nx6) ---
Output = Pdata7(:,13:18);   % Nx6


Output_corrected = Output / nanoCalibMatrix';


Input = Output_corrected - NanoOffset';



%% --- Rename Columns for Clarity ---
Fx = Input(:,1);
Fy = Input(:,2);
Fz = Input(:,3);
Mx = Input(:,4);
My = Input(:,5);
Mz = Input(:,6);

xx = Pdata7(:,13);
yy  = Pdata7(:,14);
zz = Pdata7(:,15);

%% --- Compute Tangential Force & Coulomb Friction ---
Ft = sqrt(xx.^2 + yy.^2);          % Tangential force magnitude
epsilon = 1e-6;                     % Avoid divide by zero
mu = Ft ./ (abs(zz) + epsilon);     % Coulomb friction coefficient

%% --- Optional: Plot Forces ---
figure;
subplot(3,1,1)
plot(Fx); title('Fx'); grid on;

subplot(3,1,2)
plot(Fy); title('Fy'); grid on;

subplot(3,1,3)
plot(Fz); title('Fz (Normal)'); grid on;

%% --- Plot Torques ---
figure; hold on; grid on;
plot(Mx); plot(My); plot(Mz);
legend('Mx','My','Mz'); title('Torques'); xlabel('Sample'); ylabel('Nm');

%% --- Plot Coulomb Friction Coefficient ---
figure;
plot(mu); grid on;
title('Coulomb Friction Coefficient (\mu)');
xlabel('Sample'); ylabel('\mu');