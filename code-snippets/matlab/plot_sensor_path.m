% plot_sensor_path.m
% ---------------------------------------------------
% این اسکریپت فایل .mat را می‌خواند، 
% با استفاده از کلاس EMT مسیر سه‌بعدی سنسور را محاسبه و پلات می‌کند.
% ---------------------------------------------------

function plot_sensor_path(filename)
    % filename: مسیر و نام فایل .mat شما، مثال 'Linear_YM_10.mat'
    
    % بارگذاری داده‌ها
    data = load(filename);  
    % انتظار می‌رود data.CoilX, data.CoilY, data.CoilZ ابعاد 4×200×N داشته باشند
    
    % ساخت شی EMT و پیش‌کالیبراسیون
    EMT_C = EMT();
    EMT_C.Pre_calibration(filename);   % از داده‌های دور (Far_Sensor.mat) هم استفاده می‌کند
    
    % تعداد فریم‌ها
    N = size(data.CoilX,3);
    
    % بردارهای خروجی
    Xs = zeros(1,N);
    Ys = zeros(1,N);
    Zs = zeros(1,N);
    
    % حلقه‌ی پردازش تمام فریم‌ها
    for n = 1:N
        % ۱) حذف اغتشاش
        d = struct('CoilX', data.CoilX, 'CoilY', data.CoilY, 'CoilZ', data.CoilZ);
        d_red = EMT_C.Disturbance_Reduction(d, n);
        
        % ۲) برآورد ماتریس S
        S = EMT_C.S_Estimation(d_red, n);
        
        % ۳) محاسبه فاصله r و زاویه‌ها
        r = EMT_C.range_Estimation(S, EMT_C.k_0, EMT_C.r_0);
        [Xa, Ya, Za] = EMT_C.Pos_Estimation(S);
        
        % ۴) تبدیل زاویه‌ها به مختصات کارتزین
        Xs(n) = r * cosd(Xa);
        Ys(n) = r * cosd(Ya);
        Zs(n) = r * cosd(Za);
    end
    
    % رسم مسیر سه‌بعدی
    figure;
    plot3(Xs, Ys, Zs, '-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    grid on; axis equal;
    xlabel('X [mm]');
    ylabel('Y [mm]');
    zlabel('Z [mm]');
    title(sprintf('مسیر حرکت سنسور (%d فریم)', N), 'FontSize', 14);
    view(3);
end
