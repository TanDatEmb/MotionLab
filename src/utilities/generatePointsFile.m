function generatePointsFile(numPoints, decimalPlaces, valueRange)
    % Xác định thư mục lưu trữ file
    dataDir = 'data';
    
    % Tạo thư mục nếu chưa tồn tại
    if ~exist(dataDir, 'dir')
        mkdir(dataDir);
    end
    
    % Tạo đường dẫn đầy đủ đến file
    filePath = fullfile(dataDir, 'points.txt');
    
    % Mở file để ghi (sẽ ghi đè nếu file đã tồn tại)
    fileID = fopen(filePath, 'w');
    
    % Kiểm tra xem file có mở thành công không
    if fileID == -1
        error('Cannot open file for writing.');
    end
    
    % Kiểm tra kích thước của valueRange
    if numel(valueRange) ~= 2
        error('valueRange must be a 2-element vector specifying the range [min, max].');
    end
    
    % Định dạng số thập phân
    formatSpec = ['%s %.' num2str(decimalPlaces) 'f %.' num2str(decimalPlaces) 'f %.' num2str(decimalPlaces) 'f %.' num2str(decimalPlaces) 'f %.' num2str(decimalPlaces) 'f %.' num2str(decimalPlaces) 'f %.' num2str(decimalPlaces) 'f\n'];
    
    % Tạo số liệu điểm giả
    for i = 1:numPoints
        % Tạo số liệu điểm giả
        pointName = sprintf('P%d', i);
        
        % Tạo giá trị ngẫu nhiên trong khoảng [min, max] cho 3 giá trị đầu
        x = valueRange(1) + (valueRange(2) - valueRange(1)) * rand();
        y = valueRange(1) + (valueRange(2) - valueRange(1)) * rand();
        z = valueRange(1) + (valueRange(2) - valueRange(1)) * rand();
        
        % Tạo quaternion hợp lệ (4 thành phần)
        q = rand(4, 1); % Tạo 4 giá trị ngẫu nhiên
        q = q / norm(q); % Chuẩn hóa để đảm bảo q là quaternion hợp lệ
        
        % Ghi vào file
        fprintf(fileID, formatSpec, pointName, x, y, z, q(1), q(2), q(3), q(4));
    end
    
    % Đóng file
    fclose(fileID);
    
    fprintf('File "points.txt" has been created or overwritten in the "data" directory with %d points.\n', numPoints);
end
