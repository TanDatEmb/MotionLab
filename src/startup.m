function startup(robotName)
    % Hiển thị thông tin về ứng dụng
    fprintf('Khởi động hệ thống cho robot: %s...\n', robotName);
    
    % Kiểm tra phiên bản MATLAB
    if verLessThan('matlab', '9.0')
        warning('Bạn đang sử dụng phiên bản MATLAB cũ. Một số tính năng có thể không hoạt động như mong đợi.');
    end
    
    % Xác định đường dẫn của script hiện tại
    startup_path = fileparts(mfilename('fullpath'));
    [~, folder] = fileparts(startup_path);
    
    % Nếu thư mục chứa script không phải là thư mục chung
    if ~contains(folder, 'common')
        % Thay đổi đường dẫn nếu cần
        rvcpath = startup_path;
    else
        rvcpath = fileparts(startup_path);
    end
    
    % Đường dẫn tới thư mục tạm thời
    tempDir = fullfile(rvcpath, 'temp');
    if ~exist(tempDir, 'dir')
        mkdir(tempDir);
    end
    
    % Đường dẫn tới file cấu hình của robot
    robotConfigFile = fullfile(rvcpath, 'config', sprintf('%s_config.txt', robotName));
    configFile = fullfile(tempDir, 'config.txt');
    
    % Đọc các thông số từ file cấu hình nếu có
    if exist(robotConfigFile, 'file')
        % Đọc các thông số từ file cấu hình
        % fprintf('Đang đọc file cấu hình: %s\n', robotConfigFile);
        configData = fileread(robotConfigFile);
        
        % Ghi thông số vào file config.txt trong thư mục temp
        fid = fopen(configFile, 'w');
        if fid == -1
            error('Không thể mở file %s để ghi.', configFile);
        end
        fprintf(fid, '%s', configData);
        fclose(fid);
        
        % fprintf('Thông số đã được ghi vào %s\n', configFile);
    else
        warning('File cấu hình %s không tồn tại.', robotConfigFile);
    end
    
    % Thêm các thư mục vào đường dẫn MATLAB
    addpath(fullfile(rvcpath, 'algorithms'));
    addpath(fullfile(rvcpath, 'controllers'));
    addpath(fullfile(rvcpath, 'move'));
    addpath(fullfile(rvcpath, 'objects'));
    addpath(fullfile(rvcpath, 'utilities'));
    addpath(fullfile(rvcpath, 'temp'));

    % Hiển thị thông báo hoàn thành
    fprintf('Chương trình đã khởi động thành công.\n');
end
