clear; clc; close all;

% مسیر پوشه فایل‌ها
folder_path = 'D:\uni\Term8\Project\Result Data\Test Data\8_2\Linear_XM\'; 
file_list = dir(fullfile(folder_path, '*.mat'));

% ----------- Init -----------
plot_2D = true;

% EMT Simulation Init
EMT_C = EMT();  
k_0 = EMT_C.k_0;
r_0 = EMT_C.r_0;
EMT_C.Pre_calibration('Far_Sensor_XM_Linear.mat');

R_all = zeros(size(angles));
X_Angel_all = zeros(size(angles));
Y_Angel_all = zeros(size(angles));
Z_Angel_all = zeros(size(angles));
x_all = zeros(size(angles));
y_all = zeros(size(angles));
z_all = zeros(size(angles));

% ----------- پردازش هر فایل -----------
for i = 1:num_files
    % بارگذاری فایل
    filename = fullfile(folder_path, file_list(i).name);
    Data = load(filename);

    % فرض: فقط اولین تایم‌استپ استفاده می‌شود (t = 1)
    t = 1;

    data_reduced = EMT_C.Disturbance_Reduction(Data, t);
    S = EMT_C.S_Estimation(data_reduced, t);
    r = EMT_C.range_Estimation(S, k_0, r_0);
    [X_Angle, Y_Angle, Z_Angle] = EMT_C.Pos_Estimation(S);
    R = EMT_C.Orientation_Estimation(S);
    q = rotm2quat(R);

    % محاسبه موقعیت
    x = r * cosd(X_Angle);
    y = r * cosd(Y_Angle);
    z = r * cosd(Z_Angle);

    x_all(i) = r * cosd(X_Angel);
    y_all(i) = r * cosd(Y_Angel);
    z_all(i) = r * cosd(Z_Angel);


    fprintf('File: %s\n', filename)
    fprintf('R= %5.1f  X_Angel= %5.1f  Y_Angel= %5.1f  Z_Angel= %5.1f\n', r, X_Angle, Y_Angle, Z_Angle)
    fprintf('x= %5.1f  y= %5.1f  z= %5.1f\n\n', x, y, z)
end

% ----------- پلات دو‌بعدی (XY) -----------
if plot_2D
    figure;
    plot(positions(1, :), positions(2, :), 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
    xlabel('X');
    ylabel('Y');
    title('2D Trajectory in XY Plane');
    grid on;
    axis equal;
end

% ----------- پلات فاصله از مبدأ بر حسب شماره نقطه -----------
figure;
plot(1:num_files, distances_from_origin, 'ro-', 'LineWidth', 1.5, 'MarkerFaceColor', 'r');
xlabel('Point Index');
ylabel('Distance from Origin');
title('Distance of Each Point from Origin');
grid on;
