% Hàm gửi dữ liệu (file output.txt)
function sendDataUART()
    % Kiểm tra cài đặt UART đã được đặt chưa
    if isfield(app, 'uartSettings')
        % Đọc file output.txt
        outputFile = fullfile('data', 'output.txt');
        if exist(outputFile, 'file')
            % Mở kết nối UART với cài đặt đã lưu
            uart = serialport("COM3", str2double(app.uartSettings.Baudrate), ...
                'DataBits', str2double(app.uartSettings.DataBits), ...
                'Parity', app.uartSettings.Parity, ...
                'StopBits', str2double(app.uartSettings.StopBits));
    
            % Đọc nội dung file
            fileID = fopen(outputFile, 'r');
            fileData = fread(fileID, '*char')';
            fclose(fileID);
    
            % Gửi dữ liệu qua UART
            write(uart, fileData, 'char');
    
            % Đóng kết nối UART
            clear uart;
    
            % Thông báo đã gửi dữ liệu
            disp('Đã gửi dữ liệu từ file output.txt qua UART.');
        else
            disp('Không tìm thấy file output.txt trong thư mục data.');
        end
    else
        disp('Vui lòng cài đặt thông số UART trước khi gửi dữ liệu.');
    end
end