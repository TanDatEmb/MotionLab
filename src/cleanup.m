function cleanup()
    % Hiển thị thông tin kết thúc
    fprintf('Đang kết thúc chương trình...\n');
    
    % Xóa biến toàn cục nếu cần thiết
    clearvars -global;
    
    % Xóa các biến khác
    clearvars;
    
    % Gỡ bỏ các thư mục khỏi đường dẫn MATLAB
    rvcpath = fileparts(mfilename('fullpath'));
    
    % Các thư mục cần gỡ bỏ khỏi đường dẫn
    removePaths = {...
        fullfile(rvcpath, 'algorithms'), ...
        fullfile(rvcpath, 'controllers'), ...
        fullfile(rvcpath, 'move'), ...
        fullfile(rvcpath, 'objects'), ...
        fullfile(rvcpath, 'utilities')};
    
    for i = 1:length(removePaths)
        rmpath(removePaths{i});
        % fprintf('Đã gỡ bỏ thư mục: %s\n', removePaths{i});
    end
    
    % Xóa tất cả file trong thư mục temp
    tempDir = fullfile(rvcpath, 'temp');
    if exist(tempDir, 'dir')
        files = dir(fullfile(tempDir, '*'));
        for i = 1:length(files)
            if ~files(i).isdir
                delete(fullfile(tempDir, files(i).name));
                % fprintf('Đã xóa file: %s\n', files(i).name);
            end
        end
        % Xóa thư mục temp nếu rỗng
        rmdir(tempDir);
        % fprintf('Đã xóa thư mục tạm thời: %s\n', tempDir);
    end
    
    % Lưu trữ thông tin hoặc báo cáo cuối cùng nếu cần thiết
    % Ví dụ: lưu thông tin log
    logFile = 'program_log.txt'; % Thay đổi nếu bạn có file log khác
    if exist(logFile, 'file')
        fid = fopen(logFile, 'a');
        if fid ~= -1
            fprintf(fid, '\nChương trình kết thúc vào: %s\n', datestr(now));
            fclose(fid);
            fprintf('Đã lưu thông tin vào file log: %s\n', logFile);
        end
    end
    
    % Thông báo hoàn thành
    fprintf('Chương trình đã kết thúc thành công.\n');
end
