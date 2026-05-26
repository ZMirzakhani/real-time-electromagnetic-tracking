classdef EMT < handle
    %EMT EMT main class
    %   Functions related to electromagnetic tracking are written in this class.
    
    properties
        DistParams
        % Coils_calib = [CoilX CoilY CoilZ]
        Coils_calib = [ 1 11230/10400 11230/8219] ;
        % Sensor_calib = [Sensor_X Sensor_Y Sensor_Z]
        Sensor_calib = [1 -1 -(10641/11282)*10700/10400] ;
        disp_S = true;
        % calib params for x angel
        X_Angel_L_P = [-344.3462 906.9532 -795.8873 233.0296];
        X_Angel_U_P = [15.7819 -12.8442 1.1070 1.9044];
        
        k_0=5.901239486302375e+3;
        r_0=20;
        
        %serial ports
        usb_port
        usart_port
        udp_port
        
    end
    
    methods
        function obj = EMT()
            %EMT Construct an instance of this class
            %   Detailed explanation goes here
        end

        function Start(obj)
            obj.usb_port = serialport("COM13",115200);
            obj.usart_port = serialport("COM12",115200);
            
        end
        
        function Command(obj,cmd)
            write(obj.usart_port, cmd, 'uint8');
        end
        
        function data=Receive_Data(obj)
            data_in=read(obj.usb_port, 4800, 'uint8');
            Receive_Data = data_in;
            Data_uint_16 = obj.convertUint8ToUint16(Receive_Data(end, :));

            CoilX(:,:,1) = obj.distributeByFour(Data_uint_16(1:800));
            CoilY(:,:,1) = obj.distributeByFour(Data_uint_16(801:1600));
            CoilZ(:,:,1) = obj.distributeByFour(Data_uint_16(1601:2400));

            data.CoilX = CoilX;
            data.CoilY = CoilY;
            data.CoilZ = CoilZ;
        end
        
        function Sensors = Pre_calibration(obj,FileName)
            Coils = zeros(3,2);
            Sensors = zeros(3,3,3);
            data = load(FileName);
            % Parameters
            fs = 200000;          % Sampling frequency (Hz)
            N = 200;           % Length of signal (number of samples)
            
            % Frequency axis for plotting or checking
            f=10000;
            frequencies = (0:N-1)*(fs/N);
            index_10kHz = find(frequencies >= f, 1);  % Index of the 10 kHz component
            
            
            sX=data.CoilX(4,:,1);
            sY=data.CoilY(4,:,1);
            sZ=data.CoilZ(4,:,1);
            
            sXx=data.CoilX(1,:,1);
            sXy=data.CoilX(2,:,1);
            sXz=data.CoilX(3,:,1);
            
            sYx=data.CoilY(1,:,1);
            sYy=data.CoilY(2,:,1);
            sYz=data.CoilY(3,:,1);
            
            sZx=data.CoilZ(1,:,1);
            sZy=data.CoilZ(2,:,1);
            sZz=data.CoilZ(3,:,1);
            
            % FFT of coils
            SX = fft(sX);
            SY = fft(sY);
            SZ = fft(sZ);
            % FFT of sensors
            SXx = fft(sXx);
            SXy = fft(sXy);
            SXz = fft(sXz);
            
            SYx = fft(sYx);
            SYy = fft(sYy);
            SYz = fft(sYz);
            
            SZx = fft(sZx);
            SZy = fft(sZy);
            SZz = fft(sZz);
            
            % Amplitude of the 10 kHz for Coils
            Coils(1,1) = abs(SX(index_10kHz)) / (N/2);  % Normalize by N/2
            Coils(2,1) = abs(SY(index_10kHz)) / (N/2);  % Normalize by N/2
            Coils(3,1) = abs(SZ(index_10kHz)) / (N/2);  % Normalize by N/2
            
            % Amplitude of the 10 kHz for Sensors
            Sensors(1,1,1) = (abs(SXx(index_10kHz)) / (N/2))/Coils(1,1);  % Normalize by N/2
            Sensors(1,2,1) = (abs(SXy(index_10kHz)) / (N/2))/Coils(1,1);  % Normalize by N/2
            Sensors(1,3,1) = (abs(SXz(index_10kHz)) / (N/2))/Coils(1,1);  % Normalize by N/2
            
            Sensors(2,1,1) = (abs(SYx(index_10kHz)) / (N/2))/Coils(2,1);  % Normalize by N/2
            Sensors(2,2,1) = (abs(SYy(index_10kHz)) / (N/2))/Coils(2,1);  % Normalize by N/2
            Sensors(2,3,1) = (abs(SYz(index_10kHz)) / (N/2))/Coils(2,1);  % Normalize by N/2
            
            Sensors(3,1,1) = (abs(SZx(index_10kHz)) / (N/2))/Coils(3,1);  % Normalize by N/2
            Sensors(3,2,1) = (abs(SZy(index_10kHz)) / (N/2))/Coils(3,1);  % Normalize by N/2
            Sensors(3,3,1) = (abs(SZz(index_10kHz)) / (N/2))/Coils(3,1);  % Normalize by N/2
            
            % Phase of the 10 kHz for Coils
            Coils(1,2) = angle(SX(index_10kHz));
            Coils(2,2) = angle(SY(index_10kHz));
            Coils(3,2) = angle(SZ(index_10kHz));
            
            % Phase of the 10 kHz for Sensors
            Sensors(1,1,2) = angle(SXx(index_10kHz))-Coils(1,2);
            Sensors(1,2,2) = angle(SXy(index_10kHz))-Coils(1,2);
            Sensors(1,3,2) = angle(SXz(index_10kHz))-Coils(1,2);
            
            Sensors(2,1,2) = angle(SYx(index_10kHz))-Coils(2,2);
            Sensors(2,2,2) = angle(SYy(index_10kHz))-Coils(2,2);
            Sensors(2,3,2) = angle(SYz(index_10kHz))-Coils(2,2);
            
            Sensors(3,1,2) = angle(SZx(index_10kHz))-Coils(3,2);
            Sensors(3,2,2) = angle(SZy(index_10kHz))-Coils(3,2);
            Sensors(3,3,2) = angle(SZz(index_10kHz))-Coils(3,2);
            
            % Offset of Sensors
            Sensors(1,1,3)=abs(SXx(1)) / N ;
            Sensors(1,2,3)=abs(SXy(1)) / N ;
            Sensors(1,3,3)=abs(SXz(1)) / N ;
            
            Sensors(2,1,3)=abs(SYx(1)) / N ;
            Sensors(2,2,3)=abs(SYy(1)) / N ;
            Sensors(2,3,3)=abs(SYz(1)) / N ;
            
            Sensors(3,1,3)=abs(SZx(1)) / N ;
            Sensors(3,2,3)=abs(SZy(1)) / N ;
            Sensors(3,3,3)=abs(SZz(1)) / N ;
            obj.DistParams = Sensors;
        end
    
        function data_out = Disturbance_Reduction(obj,data,n)
            
            % data : Input data set from "data = load('DataSet.mat');"
            % Sensors : Pre_Calibrated parameters obtained by Pre_calibration function
            % n : the data stream number from 1 to 10
            % data_out : The filtered data with disturbance reduction at nth sample.
            
            % Parameters
            fs = 200000;          % Sampling frequency (Hz)
            T = 1/fs;           % Sampling period (s)
            N = 200;           % Length of signal (number of samples)
            t = (0:N-1)*T;      % Time vector
            Sensors = obj.DistParams ;
            % Frequency axis for plotting or checking
            f=10000;
            frequencies = (0:N-1)*(fs/N);
            index_10kHz = find(frequencies >= f, 1);  % Index of the 10 kHz component
            
            data_out=data;
            Coils = zeros(3,2);
            
            sX=data.CoilX(4,:,n);
            sY=data.CoilY(4,:,n);
            sZ=data.CoilZ(4,:,n);
            % FFT of coils
            SX = fft(sX);
            SY = fft(sY);
            SZ = fft(sZ);
            
            % Amplitude of the 10 kHz for Coils
            Coils(1,1) = abs(SX(index_10kHz)) / (N/2);  % Normalize by N/2
            Coils(2,1) = abs(SY(index_10kHz)) / (N/2);  % Normalize by N/2
            Coils(3,1) = abs(SZ(index_10kHz)) / (N/2);  % Normalize by N/2
            
            % Phase of the 10 kHz for Coils
            Coils(1,2) = angle(SX(index_10kHz));
            Coils(2,2) = angle(SY(index_10kHz));
            Coils(3,2) = angle(SZ(index_10kHz));
            
            data_out.CoilX(1,:,n) = data_out.CoilX(1,:,n) - Coils(1,1)*Sensors(1,1,1) * ...
                cos(2*pi*f*t + (Coils(1,2) + Sensors(1,1,2)))-Sensors(1,1,3);
            data_out.CoilX(2,:,n) = data_out.CoilX(2,:,n) - Coils(1,1)*Sensors(1,2,1) * ...
                cos(2*pi*f*t + (Coils(1,2) + Sensors(1,2,2)))-Sensors(1,2,3);
            data_out.CoilX(3,:,n) = data_out.CoilX(3,:,n) - Coils(1,1)*Sensors(1,3,1) * ...
                cos(2*pi*f*t + (Coils(1,2) + Sensors(1,3,2)))-Sensors(1,3,3);
            
            data_out.CoilY(1,:,n) = data_out.CoilY(1,:,n) - Coils(2,1)*Sensors(2,1,1) * ...
                cos(2*pi*f*t + (Coils(2,2) + Sensors(2,1,2)))-Sensors(2,1,3);
            data_out.CoilY(2,:,n) = data_out.CoilY(2,:,n) - Coils(2,1)*Sensors(2,2,1) * ...
                cos(2*pi*f*t + (Coils(2,2) + Sensors(2,2,2)))-Sensors(2,2,3);
            data_out.CoilY(3,:,n) = data_out.CoilY(3,:,n) - Coils(2,1)*Sensors(2,3,1) * ...
                cos(2*pi*f*t + (Coils(2,2) + Sensors(2,3,2)))-Sensors(2,3,3);
            
            data_out.CoilZ(1,:,n) = data_out.CoilZ(1,:,n) - Coils(3,1)*Sensors(3,1,1) * ...
                cos(2*pi*f*t + (Coils(3,2) + Sensors(3,1,2)))-Sensors(3,1,3);
            data_out.CoilZ(2,:,n) = data_out.CoilZ(2,:,n) - Coils(3,1)*Sensors(3,2,1) * ...
                cos(2*pi*f*t + (Coils(3,2) + Sensors(3,2,2)))-Sensors(3,2,3);
            data_out.CoilZ(3,:,n) = data_out.CoilZ(3,:,n) - Coils(3,1)*Sensors(3,3,1) * ...
                cos(2*pi*f*t + (Coils(3,2) + Sensors(3,3,2)))-Sensors(3,3,3);
            
        end
        
        function [S] =  S_Estimation(obj,data,n)
        
            S = zeros(3) ;
            i = 0 ;
            j = 0 ;
            CoilX = data.CoilX;
            CoilY = data.CoilY;
            CoilZ = data.CoilZ;
            

            % for disp resulte
            %clc
            % for plot result
            %figure
        
        
            for Coil = ['X','Y','Z'] 
                i = i + 1 ;
                if Coil == 'X'
                    Selected_Coil = CoilX ;
                end
                if Coil == 'Y'
                    Selected_Coil = CoilY ;
                end
                if Coil == 'Z'
                    Selected_Coil = CoilZ ;
                end
        
                for sensor = ['x','y','z']
        
                    if sensor == 'x'
                        Selected_sensor = Selected_Coil(1,:,n) ;
                    end
                    if sensor == 'y'
                        Selected_sensor = Selected_Coil(2,:,n) ;
                    end
                    if sensor == 'z'
                        Selected_sensor = Selected_Coil(3,:,n) ;
                    end
        
                    j = j + 1;
                    % Define parameters
                    Fs = 200000; % Sampling frequency (Hz)
                    N = 200; % Number of samples
                    t = (0:N-1)/Fs; % Time vector
                    x = Selected_sensor; % Example signal with two frequencies
        
                    % Compute the Fourier Transform
                    X = fft(x); % Compute the FFT
        
                    % Compute the amplitude
                    amplitude = abs(X/N); % Normalize the magnitude
                    amplitude = amplitude(1:N/2+1); % Extract the first half (due to symmetry)
                    amplitude(2:end-1) = 2*amplitude(2:end-1); % Double the values (except DC and Nyquist frequency)
                    amplitude(1) = 0;
        
                    % Frequency vector
                    f = Fs*(0:(N/2))/N;
                    % Find the peak amplitude and corresponding frequency
                    [maxAmplitude, idx] = max(amplitude); % Find the peak value and its index
                    peakFrequency = f(idx); % Corresponding frequency of the peak
        
                    phase_sig=sign(angle(X(11)));

                    S(i,j) = phase_sig*amplitude(11)*obj.Coils_calib(i)*obj.Sensor_calib(j);
        
                    % Display the results
                    if obj.disp_S
                        fprintf('%c%c: %.4f %.0f - ',Coil,sensor, S(i,j), f(11));
                    end
                    % 
        
                    % plot fft amp 
                    %subplot(3,3,(j-1)*3+i)
                    %plot(amplitude)
                    %title(strcat(Coil,sensor))
        
                end
                if obj.disp_S
                fprintf('\n')
                end
                j = 0;
            end
        end
    
        function plot_Waves(~,data,n)
            yaxis = [-2^16 2^16];
            figure
                % Coil X
                subplot(4,3,1)
                plot(data.CoilX(4,1:60,n))
                title('Coil X curret')
                ylim(yaxis)
            
                subplot(4,3,4)
                plot(data.CoilX(1,1:60,n))
                title('Sensor x')
                ylim(yaxis)
            
                subplot(4,3,7)
                plot(data.CoilX(2,1:60,n))
                title('Sensor y')
                ylim(yaxis)
            
                subplot(4,3,10)
                plot(data.CoilX(3,1:60,n))
                title('Sensor z')
                ylim(yaxis)
            
                % Coil Y
                subplot(4,3,2)
                plot(data.CoilY(4,1:60,n),'r')
                title('Coil Y curret')
                ylim(yaxis)
            
                subplot(4,3,5)
                plot(data.CoilY(1,1:60,n),'r')
                title('Sensor x')
                ylim(yaxis)
            
                subplot(4,3,8)
                plot(data.CoilY(2,1:60,n),'r')
                title('Sensor y')
                ylim(yaxis)
            
                subplot(4,3,11)
                plot(data.CoilY(3,1:60,n),'r')
                title('Sensor z')
                ylim(yaxis)
            
                % Coil Z
                subplot(4,3,3)
                plot(data.CoilZ(4,1:60,n),'k')
                title('Coil Z curret')
                ylim(yaxis)
            
                subplot(4,3,6)
                plot(data.CoilZ(1,1:60,n),'k')
                title('Sensor x')
                ylim(yaxis)
            
                subplot(4,3,9)
                plot(data.CoilZ(2,1:60,n),'k')
                title('Sensor y')
                ylim(yaxis)
            
                subplot(4,3,12)
                plot(data.CoilZ(3,1:60,n),'k')
                title('Sensor z')
                ylim(yaxis)
        end
    
        function r = range_Estimation(~,S,k_0,r_0)
            % k = sqrt( |B_X|^2 + |B_Y|^2 + |B_Z|^2 )
            k = sqrt((1/6)*trace((S.')*S)); 
            r = r_0 * (k_0/k) ^ (1/3);
        end

        function [X_Angel ,Y_Angel, Z_Angel] = Pos_Estimation(obj,S)
           k = sqrt(6/trace((S.')*S));
           B_Y = sqrt(S(2,1)^2+S(2,2)^2+S(2,3)^2);
           B_X = sqrt(S(1,1)^2+S(1,2)^2+S(1,3)^2);
           B_Z = sqrt(S(3,1)^2+S(3,2)^2+S(3,3)^2);
           KB_Y = (k*B_Y);
           KB_X = (k*B_X);
           KB_Z = (k*B_Z);

           if KB_Y>2
               Y_Angel = acos(sqrt((1/3)*(2^2-1)))*180/pi;
               %disp('KB_Y bigger than 2')
           elseif KB_Y<1
               %disp('KB_Y smaller than 1')
               Y_Angel = acos(sqrt((1/3)*(1^2-1)))*180/pi;
           else 
               Y_Angel = acos(sqrt((1/3)*(KB_Y^2-1)))*180/pi;
           end

           % if KB_X>2
           %     X_Angel = acos(sqrt((1/3)*(2^2-1)))*180/pi;
           %     %disp('KB_X bigger than 2')
           % elseif KB_X<1
           %     %disp('KB_X smaller than 1')
           %     X_Angel = acos(sqrt((1/3)*(1^2-1)))*180/pi;
           % else
           %     X_Angel = acos(sqrt((1/3)*(KB_X^2-1)))*180/pi;
           %     if X_Angel<27
           %         x = sqrt((1/3)*(KB_X^2-1));
           %         X_Angel = obj.X_Angel_L_P(1)*x^3 + obj.X_Angel_L_P(2)*x^2 + obj.X_Angel_L_P(3)*x + obj.X_Angel_L_P(4);
           %         if X_Angel<0
           %         X_Angel = 0;
           %         else
           %         X_Angel = X_Angel*180/pi;
           %         end
           %     end
           %     if X_Angel>60
           %         x = sqrt((1/3)*(KB_X^2-1));
           %         X_Angel = obj.X_Angel_U_P(1)*x^3 + obj.X_Angel_U_P(2)*x^2 + obj.X_Angel_U_P(3)*x + obj.X_Angel_U_P(4);
           %         if X_Angel>1.5707
           %         X_Angel = 90;
           %         else
           %         X_Angel = X_Angel*180/pi;
           %         end
           %     end
           % end

           if KB_X>2
               X_Angel = acos(sqrt((1/3)*(2^2-1)))*180/pi;
               disp('KB_X bigger than 2')
           elseif KB_X<1
               disp('KB_X smaller than 1')
               X_Angel = acos(sqrt((1/3)*(1^2-1)))*180/pi;
           else
               X_Angel = acos(sqrt((1/3)*(KB_X^2-1)))*180/pi;
           end


           if KB_Z>2
               Z_Angel = acos(sqrt((1/3)*(2^2-1)))*180/pi;
               %disp('KB_X bigger than 2')
           elseif KB_Z<1
               %disp('KB_X smaller than 1')
               Z_Angel = acos(sqrt((1/3)*(1^2-1)))*180/pi;
           else
               Z_Angel = acos(sqrt((1/3)*(KB_Z^2-1)))*180/pi;
           end


        end
        
        function R=Orientation_Estimation(~,S)
            k = sqrt(trace((S.')*S)/6);
            R=S/k-2*k*(S^-1)';
            [U, ~, V] = svd(R);  % SVD decomposition
            % Orthogonalize the matrix by recomposing with U and V
            R = U * V';
        end

        function abs_fild_array = abs_fild(~,S)
            %   fine |B_X| |B_Y| |B_Z|
            abs_fild_array = [];
            abs_fild_array = [abs_fild_array sqrt(S(1,1)^2+S(1,2)^2+S(1,3)^2)];
            abs_fild_array = [abs_fild_array sqrt(S(2,1)^2+S(2,2)^2+S(2,3)^2)];
            abs_fild_array = [abs_fild_array sqrt(S(3,1)^2+S(3,2)^2+S(3,3)^2)];
        end
        
        function result = convertUint8ToUint16(~,inputMatrix)
            % Check if the number of columns is even
            if mod(size(inputMatrix, 2), 2) ~= 0
                error('The number of columns must be even.');
            end

            % Get the size of the input matrix
            [rows, cols] = size(inputMatrix);

            % Preallocate the output matrix
            result = zeros(rows, cols/2, 'uint16');

            % Loop to convert every two uint8 values into one uint16
            for i = 1:rows
                for j = 1:2:cols
                    % Combine two uint8 values into one uint16
                    high_byte = inputMatrix(i, j+1);       % High byte
                    low_byte = inputMatrix(i, j);      % Low byte
                    result(i, (j+1)/2) = bitor(bitshift(uint16(high_byte), 8), uint16(low_byte));
                end
            end
        end

        function result = distributeByFour(~,inputRow)
            % This function takes a row matrix and distributes its elements into a 
            % 4-row matrix. Each row will contain elements based on their position 
            % modulo 4.

            % Number of elements in the input row
            n = length(inputRow);

            % Initialize a 4-row matrix with zeros
            result = zeros(4, ceil(n/4));

            % Loop through the input row and assign elements to the appropriate row
            for i = 1:n
                % Determine which row the element should go to
                row = mod(i-1, 4) + 1;
                % Determine the column in the result matrix
                col = ceil(i/4);
                % Assign the element to the correct position in the result matrix
                result(row, col) = inputRow(i);
            end
        end
        function UDP_Start(obj,IP,Port)
            obj.udp_port=udp(IP, 'RemotePort', Port);
            if obj.udp_port.Status =='closed'
                fopen(obj.udp_port);
            end
            
        end
        function Send_UDP(obj,data)  
            fwrite(obj.udp_port, data, 'char'); % Adjust type as needed (e.g., 'uint8', 'char', etc.)
        end
        
        function Stop(obj)
            fclose(obj.udp_port);
        end

    end
end

