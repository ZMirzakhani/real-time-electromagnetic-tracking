clc, clear, close all

EMT_C = EMT();

k_0 = EMT_C.k_0;
r_0 = EMT_C.r_0;

EMT_C.Pre_calibration('Far_Sensor_Kaaba.mat');

angles = 0:10:90;

R_all = zeros(size(angles));
X_Angel_all = zeros(size(angles));
Y_Angel_all = zeros(size(angles));
Z_Angel_all = zeros(size(angles));
x_all = zeros(size(angles));
y_all = zeros(size(angles));
z_all = zeros(size(angles));

for i = 1:numel(angles)
    filename = sprintf('Kaaba_21_%d.mat', angles(i));
    data = load(filename);

    data_Reducted = EMT_C.Disturbance_Reduction(data, 1);

    S = EMT_C.S_Estimation(data_Reducted, 1);
    r = EMT_C.range_Estimation(S, k_0, r_0);
    [X_Angel ,Y_Angel, Z_Angel] = EMT_C.Pos_Estimation(S);
    R = EMT_C.Orientation_Estimation(S);

    R_all(i) = r;
    X_Angel_all(i) = X_Angel;
    Y_Angel_all(i) = Y_Angel;
    Z_Angel_all(i) = Z_Angel;

    x_all(i) = r * cosd(X_Angel);
    y_all(i) = r * cosd(Y_Angel);
    z_all(i) = r * cosd(Z_Angel);

    fprintf('File: %s\n', filename)
    fprintf('R= %5.1f  X_Angel= %5.1f  Y_Angel= %5.1f  Z_Angel= %5.1f\n', r, X_Angel, Y_Angel, Z_Angel)
    fprintf('x= %5.1f  y= %5.1f  z= %5.1f\n\n', x_all(i), y_all(i), z_all(i))
end
