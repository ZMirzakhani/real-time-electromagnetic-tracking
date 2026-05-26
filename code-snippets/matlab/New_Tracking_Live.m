clc,clear,close all,

% ---------- Init ----------

%init parameters
R=eye(3);
x=0;
y=0;
z=0;
r=10;
t=0;
time=0;
X_Angel=0;
Y_Angel=0;
Z_Angel=0;
a=0;

% Weighted parameters for moving avarage: Obtained from softmax.
nn=0.1; 
pp= 10:-1:1;
pp=nn*pp;
W=exp(pp)/sum(exp(pp));
pos=zeros(3,10);


%plot_3D = false;
plot_3D = true;
%view_flag = false;
view_flag = true;

Save_Data = false;

S_total = [];

% EMT Init
EMT_C = EMT();
EMT_C.Start();
EMT_C.UDP_Start('127.0.0.1', 1); % local ip for simulation: IP='127.0.0.1', Port=1

k_0 = EMT_C.k_0;
r_0 = EMT_C.r_0;

EMT_C.Pre_calibration('Far_Sensor.mat');

if plot_3D
        figure
        xlabel('X-axis');
        ylabel('Y-axis');
        zlabel('Z-axis');
        grid on;
        hold on;
        set(gca, 'SortMethod', 'depth');
        lighting phong;
        set(gcf, 'Renderer', 'zbuffer'); % Change to 'painters' if needed
        clf
end

stopFlag = false; % Initialize stop flag

% ---------- Main Loop ----------
while ~stopFlag
tic

EMT_C.Command('SS;');


    
if plot_3D
    ux = [1; 0; 0];
    uy = [0; 1; 0];
    uz = [0; 0; 1];
    
%     ux = ux'*R;
%     uy = uy'*R;
%     uz = uz'*R;
    ux = R * ux;
    uy = R * uy;
    uz = R * uz;
    if view_flag
          clf
          view(110,10)
          %view(90,90)
          xlim([0 25])
          ylim([0 25])
          zlim([0 30])
          hold on
    end
    
    plot3(0,0,0,'o','MarkerFaceColor', 'r','LineWidth', 10)
    hold on;
    quiver3(x, y, z, ux(1), ux(2), ux(3), 2, 'r', 'LineWidth', 3); % X-axis orientation
    hold on;
    quiver3(x, y, z, uy(1), uy(2), uy(3), 2, 'g', 'LineWidth', 3); % Y-axis orientation
    hold on;
    quiver3(x, y, z, uz(1), uz(2), uz(3), 2, 'b', 'LineWidth', 3); % Z-axis orientation
    hold off
    drawnow ;
    grid minor
end

data=EMT_C.Receive_Data();
data_Reducted = EMT_C.Disturbance_Reduction(data,1);

clc
%  ------ Estimations : Sensing matrix, Range, Position and Orientation -----
S = EMT_C.S_Estimation(data_Reducted,1);
r = EMT_C.range_Estimation(S,k_0,r_0);
[X_Angel ,Y_Angel, Z_Angel] = EMT_C.Pos_Estimation(S);
R=EMT_C.Orientation_Estimation(S);
quaternion=rotm2quat(R);
    x = r*cosd(X_Angel);
    y = r*cosd(Y_Angel);
    z = r*cosd(Z_Angel);
    
    pos(1,:)=[x,pos(1,1:9)];
    pos(2,:)=[y,pos(2,1:9)];
    pos(3,:)=[z,pos(3,1:9)];
    
    x=pos(1,:)*W';
    y=pos(2,:)*W';
    z=pos(3,:)*W';
send_string=sprintf('%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f,%.2f',...
    x,y,z-3,quaternion(1),quaternion(2),quaternion(3),quaternion(4),r);
EMT_C.Send_UDP(send_string);

fprintf('\nR= %5.1f  ',r)
fprintf(' X_Angel= %5.f  ',X_Angel)
fprintf(' Y_Angel= %5.f  ',Y_Angel)
fprintf(' Z_Angel= %5.f  ',Z_Angel)
fprintf(' \n x= %5.1f  ',x)
fprintf(' \n y= %5.1f  ',y)
fprintf(' \n z= %5.1f  ',z)
fprintf('\n time= %5.4f ',a)

% ------ Stopping the program by pressing q -----
pause(0.001);  % Pause to allow checking for key press
    if ~isempty(get(gcf, 'CurrentCharacter'))
        key = get(gcf, 'CurrentCharacter');
        if strcmpi(key, 'q')
            stopFlag = true;  % Set flag to exit loop if "Q" is pressed
        end
        set(gcf, 'CurrentCharacter', ' ');  % Clear the character by setting to a space
    end
a=toc;
time = (time*t+a)/(t+1);
t=t+1;

if Save_Data
    S_total = [S_total S];
end

end

if Save_Data
    save("finish.mat","S_total")
end



EMT_C.Stop();

% %%
% u = udp('127.0.0.1', 'RemotePort', 12345);
% fopen(u);
% data = '1,2,3,4'; % Example data
% fwrite(u, data, 'char');
% fclose(u);


