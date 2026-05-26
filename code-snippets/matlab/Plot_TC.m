clear; clc; close all;

% ---------- بارگذاری فایل ----------
filename = 'D:\uni\Term8\Project\Result Data\Test Data\3D\S_Triangle.mat';  % مسیر فایل
data = load(filename);

S_all = data.S_total;  % ابعاد 3×2859

% ---------- پارامترهای EMT ----------
EMT_C = EMT();
k_0 = EMT_C.k_0;
r_0 = EMT_C.r_0;

% ---------- آماده‌سازی ----------
num_vectors = size(S_all, 2) / 3;
positions = zeros(3, num_vectors);   % موقعیت XYZ
R_all =  zeros(3,3,num_vectors);     % ماتریس‌های چرخش

for i = 1+350:num_vectors-450
    idx = (i-1)*3 + 1;
    S = S_all(:, idx:idx+2);  % S به ابعاد 3×3

    R = EMT_C.Orientation_Estimation(S);
    r = EMT_C.range_Estimation(S, k_0, r_0);
    [X_angle, Y_angle, Z_angle] = EMT_C.Pos_Estimation(S);

    % محاسبه موقعیت (x, y, z)
    x = r * cosd(X_angle);
    y = r * cosd(Y_angle);
    z = r * cosd(Z_angle);

    positions(:, i) = [x; y; z];
    R_all(:,:,i) = R;
end

% ---------- رسم مسیر و محورهای سنسور ----------
figure;
plot3(positions(1,:), positions(2,:), positions(3,:), 'b.-');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Circle Trajectory with Sensor Axes');
grid on; axis equal;
hold on;

scale = 0.2;  % طول بردارها

% رنگ‌ها
color_x = [0.47, 0.67, 0.19];  % سبز نعنایی
color_z = [0.49, 0.18, 0.56];  % ارغوانی تیره

for i = 1+350:num_vectors-450
    R_i = R_all(:,:,i);
    ux = R_i(:,1);  % محور X سنسور
    uy = R_i(:,2);  % محور Y سنسور
    uz = R_i(:,3);  % محور Z سنسور

    x0 = positions(1,i);
    y0 = positions(2,i);
    z0 = positions(3,i);

    % محورهای سنسور
    quiver3(x0, y0, z0, scale*ux(1), scale*ux(2), scale*ux(3), ...
        'Color', color_x, 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

    quiver3(x0, y0, z0, scale*uy(1), scale*uy(2), scale*uy(3), ...
        'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

    quiver3(x0, y0, z0, scale*uz(1), scale*uz(2), scale*uz(3), ...
        'm', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
end

legend('Trajectory', 'Sensor X-axis', 'Sensor Y-axis', 'Sensor Z-axis');
view(3);
hold off;
