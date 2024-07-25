function config = readConfig(configFile)
    % READCONFIG Đọc cấu hình từ file văn bản và chuyển đổi dữ liệu thành cấu hình
    %   configFile: Tên file văn bản chứa cấu hình
    %   config: Đối tượng containers.Map chứa các cặp khóa-giá trị từ file
    
    % Kiểm tra sự tồn tại của file
    if exist(configFile, 'file')
        % Mở file để đọc
        fileID = fopen(configFile, 'r');
        
        % Đọc dữ liệu từ file
        data = textscan(fileID, '%s %s', 'Delimiter', '=');
        
        % Đóng file
        fclose(fileID);
        
        % Chuyển đổi dữ liệu thành cấu hình
        keys = data{1};  % Các khóa
        values = data{2};  % Các giá trị
        config = containers.Map(keys, values);  % Tạo đối tượng containers.Map
        
    else
        error('File không tồn tại: %s', configFile);
    end
end
