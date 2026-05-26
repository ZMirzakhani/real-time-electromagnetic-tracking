
clear; clc; close all;
filename = 'D:\uni\Term8\Project\Result Data\Test Data\7_18\Linear\Linear_YM_10.mat';
Data = load(filename);  
    % فرض: داده‌ها و کلاس EMT قبلاً بارگذاری شده‌اند

% ----------- Init -----------
plot_3D = true;
view_flag = true;

% EMT Simulation Init
EMT_C = EMT();  % فرض بر این است که کلاس EMT موجود است و دارای متدهای مربوطه
k_0 = EMT_C.k_0;
r_0 = EMT_C.r_0;
EMT_C.Pre_calibration('Far_Sensor.mat');

% Weighted average parameters
nn = 0.1;
pp = 10:-1:1;
pp = nn * pp;
W = exp(pp) / sum(exp(pp));

% ذخیره موقعیت‌ها و کواترنیون‌ها
positions = zeros(3, 10);
raw_positions = zeros(3, 10);
quaternions = zeros(4, 10);

% ----------- پردازش داده‌ها -----------
for t = 1:10

    data_reduced = EMT_C.Disturbance_Reduction(Data, t);

    S = EMT_C.S_Estimation(data_reduced, t);
    r = EMT_C.range_Estimation(S, k_0, r_0);
    [X_Angle, Y_Angle, Z_Angle] = EMT_C.Pos_Estimation(S);
    R = EMT_C.Orientation_Estimation(S);
    q = rotm2quat(R);

    % موقعیت اولیه (raw)
    x = r * cosd(X_Angle);
    y = r * cosd(Y_Angle);
    z = r * cosd(Z_Angle);
    raw_positions(:, t) = [x; y; z];
    quaternions(:, t) = q';

    % فیلتر کردن (moving average)
    if t >= 10
        positions(:, t) = raw_positions(:, t-9:t) * W';
    else
        temp = repmat([x; y; z], 1, 10);
        positions(:, t) = temp * W';
    end
end

% ----------- پلات سه‌بعدی -----------
if plot_3D
    figure;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; hold on;
    view(110, 10);
    xlim([0 25]); ylim([0 25]); zlim([0 30]);
    title('3D Position and Orientation');

    for t = 1:10
        pos = positions(:, t);
        q = quaternions(:, t);
        R = quat2rotm(q');

        ux = R * [1; 0; 0];
        uy = R * [0; 1; 0];
        uz = R * [0; 0; 1];

        plot3(pos(1), pos(2), pos(3), 'o', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
        hold on;
        quiver3(pos(1), pos(2), pos(3), ux(1), ux(2), ux(3), 2, 'r', 'LineWidth', 2);
        quiver3(pos(1), pos(2), pos(3), uy(1), uy(2), uy(3), 2, 'g', 'LineWidth', 2);
        quiver3(pos(1), pos(2), pos(3), uz(1), uz(2), uz(3), 2, 'b', 'LineWidth', 2);
    end
end

