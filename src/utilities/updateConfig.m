function updateConfig(fileName, key, value)
    % Đọc nội dung file config
    fid = fopen(fileName, 'r');
    if fid == -1
        error('Không thể mở file cấu hình.');
    end
    configLines = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    
    % Cập nhật giá trị cho key
    updatedLines = {};
    keyFound = false;
    for i = 1:length(configLines{1})
        line = configLines{1}{i};
        if startsWith(line, key)
            updatedLines{end+1} = sprintf('%s=%s', key, value); %#ok<AGROW>
            keyFound = true;
        else
            updatedLines{end+1} = line; %#ok<AGROW>
        end
    end
    
    % Nếu key chưa được tìm thấy, thêm dòng mới
    if ~keyFound
        updatedLines{end+1} = sprintf('%s=%s', key, value); %#ok<AGROW>
    end
    
    % Ghi lại file config với các cập nhật
    fid = fopen(fileName, 'w');
    if fid == -1
        error('Không thể mở file cấu hình để ghi.');
    end
    fprintf(fid, '%s\n', updatedLines{:});
    fclose(fid);
end