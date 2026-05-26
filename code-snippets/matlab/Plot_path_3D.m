clear; clc; close all;

% مسیر پوشه فایل‌ها
folder_path = 'D:\uni\Term8\Project\Result Data\Test Data\7_18\Linear\'; 
file_list = dir(fullfile(folder_path, '*.mat'));

% ----------- Init -----------
plot_3D = true;

% EMT Simulation Init
EMT_C = EMT();  
k_0 = EMT_C.k_0;
r_0 = EMT_C.r_0;
EMT_C.Pre_calibration('Far_Sensor.mat');

% آماده‌سازی برای ذخیره نتایج
num_files = length(file_list);
positions = zeros(3, num_files);
quaternions = zeros(4, num_files);

% ----------- پردازش هر فایل -----------
for idx = 1:num_files
    % بارگذاری فایل
    filename = fullfile(folder_path, file_list(idx).name);
    Data = load(filename);

    % فرض: فقط اولین تایم‌استپ استفاده می‌شود (t = 1)
    t = 5;

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
    positions(:, idx) = [x; y; z];
    quaternions(:, idx) = q';
end

% ----------- پلات سه‌بعدی مسیر -----------

if plot_3D
    figure;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; hold on;
    view(110, 10);
    title('Trajectory from Multiple Files');
    xlim([0 25]); ylim([0 25]); zlim([0 30]);

    for idx = 1:num_files
        pos = positions(:, idx);
        q = quaternions(:, idx);
        R = quat2rotm(q');

        % محورها
        ux = R * [1; 0; 0];
        uy = R * [0; 1; 0];
        uz = R * [0; 0; 1];

        % نمایش نقطه و بردارهای محور
        plot3(pos(1), pos(2), pos(3), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        quiver3(pos(1), pos(2), pos(3), ux(1), ux(2), ux(3), 1.5, 'r', 'LineWidth', 1.5);
        quiver3(pos(1), pos(2), pos(3), uy(1), uy(2), uy(3), 1.5, 'g', 'LineWidth', 1.5);
        quiver3(pos(1), pos(2), pos(3), uz(1), uz(2), uz(3), 1.5, 'b', 'LineWidth', 1.5);
    end

    % رسم مسیر بین نقاط
    plot3(positions(1, :), positions(2, :), positions(3, :), 'k-', 'LineWidth', 1.5);
end
