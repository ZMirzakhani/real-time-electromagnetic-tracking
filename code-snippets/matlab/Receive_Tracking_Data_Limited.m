clc,clear,close all,
usb_port = serialport("COM11",115200);
usart_port = serialport("COM10",115200);
Receive_Data = [];
Data_uint_16 = [];
i = 1 ;


while i<=10

tic
write(usart_port,"SS;","uint8")
Receive_Data = [Receive_Data ;read(usb_port,4800,"uint8")];
Data_uint_16 = [Data_uint_16 ;convertUint8ToUint16(Receive_Data(end, :))];

CoilX(:,:,i) = distributeByFour(Data_uint_16(end,1:800));
CoilY(:,:,i) = distributeByFour(Data_uint_16(end,801:1600));
CoilZ(:,:,i) = distributeByFour(Data_uint_16(end,1601:2400));

i = i +1;
a=toc;
end

%save("R_20.mat","CoilX","CoilY","CoilZ")
save("Far_Sensor.mat","CoilX","CoilY","CoilZ")



function result = convertUint8ToUint16(inputMatrix)
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

function result = distributeByFour(inputRow)
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