% Plotting Sensor Trajectory from 4×200×10 Data in MATLAB
% This script assumes that you have already obtained a 4×200×10 data set from your EMT
% system, stored in a struct called "Data" with fields CoilX, CoilY, CoilZ.
% Each field is a 4×200×10 array: 4 rows, 200 samples, 10 streams.

% Clear workspace
clear; clc; close all;

% -------------------------------------------------------------------------
% 1. Load or simulate your data
% -------------------------------------------------------------------------
% If you have a .mat file containing Data.CoilX, CoilY, CoilZ, load it:
% load('YourDataFile.mat');
% Otherwise, assume Data is already in workspace.
 filename = 'D:\uni\Term8\Project\Result Data\Test Data\7_18\Linear_YM_10.mat';
 Data = load(filename);  
% -------------------------------------------------------------------------
% 2. Instantiate and calibrate the EMT object
% -------------------------------------------------------------------------
EMT_C = EMT();
EMT_C.Pre_calibration('Far_Sensor.mat');  % adjust file name if needed

% Preallocate for N samples (third dimension)
N = size(Data.CoilX, 3);
pos = zeros(N, 3);   % [x, y, z] for each sample

% -------------------------------------------------------------------------
% 3. Loop through each data snapshot, estimate position
% -------------------------------------------------------------------------

for n = 1:N
    data_red = EMT_C.Disturbance_Reduction(Data, n);
    S = EMT_C.S_Estimation(data_red, n);

    % Estimate range
    r = EMT_C.range_Estimation(S, EMT_C.k_0, EMT_C.r_0);
    [X_Angel ,Y_Angel, Z_Angel] = EMT_C.Pos_Estimation(S);
    % Use orientation to find direction vector
    R = EMT_C.Orientation_Estimation(S);
    forward_vector = R(:,3); % Z-axis of the sensor orientation

    x = r*cosd(X_Angel);
    y = r*cosd(Y_Angel);
    z = r*cosd(Z_Angel);

    pos(1,:)=[x,pos(1,1:9)];
    pos(2,:)=[y,pos(2,1:9)];
    pos(3,:)=[z,pos(3,1:9)];
    % Sensor is located at distance r in the direction of forward_vector
    position = r * forward_vector;
    pos(n, :) = position';


R=EMT_C.Orientation_Estimation(S);
quaternion=rotm2quat(R);

end

% -------------------------------------------------------------------------
% 4. Plot 3D trajectory
% -------------------------------------------------------------------------
figure;
plot3(pos(:,1), pos(:,2), pos(:,3), '-o', 'LineWidth', 2, 'MarkerSize', 6);
hold on;
% Mark start and end
plot3(pos(1,1), pos(1,2), pos(1,3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(pos(end,1), pos(end,2), pos(end,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Sensor Trajectory');
grid on;
legend('Path', 'Start', 'End', 'Location', 'best');
axis equal;

% -------------------------------------------------------------------------
% End of Script
% -------------------------------------------------------------------------
