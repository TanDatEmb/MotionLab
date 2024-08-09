classdef TrajectoryPlanner < handle
    properties
        Points = containers.Map; % Map để lưu trữ các điểm
        CurrentPosition = Point([0 0 0], [1 0 0 0], 'World'); % Vị trí hiện tại
        CurrentVelocity = 0; % Vận tốc hiện tại
        PointTrajectory = {}; % Cell array để lưu quỹ đạo điểm
        VelocityTrajectory = {}; % Cell array để lưu quỹ đạo vận tốc
        AccelerationTrajectory = {}; % Cell array để lưu quỹ đạo gia tốc
        OrientationTrajectory = {}; % Cell array để lưu quỹ đạo hướng
        TimeTrajectory = {}; % Cell array để lưu quỹ đạo thời gian
    end

    methods
        %% Khởi tạo
        function obj = TrajectoryPlanner(option)
            if nargin < 1
                option = 0; % Mặc định không vẽ gì
            end

            % Đọc các điểm từ tệp points.txt
            obj.readPoints('points.txt');

            % Đọc và xử lý các lệnh di chuyển từ tệp commands.txt
            obj.processCommands('commands.txt');

            % Ghép các quỹ đạo chuyển động
            obj.combineTrajectories();

            % Vẽ quỹ đạo hoặc quỹ đạo 3D có hướng dựa trên tham số option
            switch option
                case 'plot2D'
                    % Vẽ quỹ đạo
                    plotTrajectory(obj.PointTrajectory, obj.VelocityTrajectory, obj.AccelerationTrajectory, obj.OrientationTrajectory, obj.TimeTrajectory);
                case 'plot3D'
                    % Vẽ quỹ đạo 3D có hướng
                    plot3DTrajectoryWithQuaternions(obj.PointTrajectory,obj.OrientationTrajectory);
                case 'save'
                    obj.saveTrajectoryToFile('point_value.txt');
                otherwise
                    % Không vẽ gì
            end
        end

        % Phương thức để đọc các điểm từ tệp
        function readPoints(obj, filename)
            fileID = fopen(filename, 'r');
            tline = fgetl(fileID);
            while ischar(tline)
                data = strsplit(tline);
                pointName = data{1};
                position = str2double(data(2:4));
                orientation = str2double(data(5:8));
                obj.Points(pointName) = Point(position, orientation, 'World');
                tline = fgetl(fileID);
            end
            fclose(fileID);
        end

        % Phương thức để đọc và xử lý các lệnh di chuyển
        function processCommands(obj, filename)
            fileID = fopen(filename, 'r');
            tline = fgetl(fileID);
            while ischar(tline)
                data = strsplit(tline);
                command = data{1};

                switch command
                    case 'moveLinear'
                        targetPoint = obj.Points(data{2});
                        desiredVelocity = str2double(data{3}(2:end));
                        zone = str2double(data{4}(2:end));
                        move = MoveLinear(obj.CurrentPosition, targetPoint, obj.CurrentVelocity, desiredVelocity, 0, zone);
                        obj.CurrentPosition = targetPoint; % Cập nhật vị trí hiện tại

                    case 'moveCircle'
                        endPoint = obj.Points(data{2});
                        midPoint = obj.Points(data{3});
                        desiredVelocity = str2double(data{4}(2:end));
                        zone = str2double(data{5}(2:end));
                        move = MoveCircle(obj.CurrentPosition, endPoint, midPoint, obj.CurrentVelocity, desiredVelocity, 0, zone);
                        obj.CurrentPosition = endPoint; % Cập nhật vị trí hiện tại
                end

                % Cập nhật quỹ đạo liên tục
                obj.updateTrajectory(move);

                % Cập nhật vận tốc hiện tại
                obj.CurrentVelocity = desiredVelocity;

                tline = fgetl(fileID);
            end
            fclose(fileID);
            % obj.combineTrajectories();
        end

        % Phương thức để cập nhật quỹ đạo liên tục
        function updateTrajectory(obj, move)
            % Thêm các quỹ đạo từ lệnh mới vào cell arrays
            obj.PointTrajectory{end+1} = move.PointTrajectory;
            obj.VelocityTrajectory{end+1} = move.VelocityTrajectory;
            obj.AccelerationTrajectory{end+1} = move.AccelerationTrajectory;
            obj.OrientationTrajectory{end+1} = move.OrientationTrajectory;
            obj.TimeTrajectory{end+1} = move.TimeTrajectory;
        end

        % Phương thức để kết hợp tất cả các quỹ đạo từ cell arrays
        function combineTrajectories(obj)
            % Kiểm tra kích thước của các cell arrays trước khi kết hợp
            if ~isempty(obj.PointTrajectory)
                obj.PointTrajectory = vertcat(obj.PointTrajectory{:});
            else
                obj.PointTrajectory = [];
            end

            if ~isempty(obj.VelocityTrajectory)
                obj.VelocityTrajectory = vertcat(obj.VelocityTrajectory{:});
            else
                obj.VelocityTrajectory = [];
            end

            if ~isempty(obj.AccelerationTrajectory)
                obj.AccelerationTrajectory = vertcat(obj.AccelerationTrajectory{:});
            else
                obj.AccelerationTrajectory = [];
            end

            if ~isempty(obj.OrientationTrajectory)
                obj.OrientationTrajectory = vertcat(obj.OrientationTrajectory{:});
            else
                obj.OrientationTrajectory = [];
            end

            % Kết hợp quỹ đạo thời gian
            if ~isempty(obj.TimeTrajectory)
                combinedTime = obj.TimeTrajectory{1}; % Bắt đầu với mảng thời gian đầu tiên
                for i = 2:length(obj.TimeTrajectory)
                    if ~isempty(obj.TimeTrajectory{i})
                        % Đảm bảo obj.TimeTrajectory{i} là một vector hàng
                        currentTrajectory = obj.TimeTrajectory{i};
                        if iscolumn(currentTrajectory)
                            currentTrajectory = currentTrajectory';
                        end
                        % Điều chỉnh currentTrajectory bằng cách cộng thêm giá trị cuối cùng của combinedTime
                        currentTrajectory = currentTrajectory + combinedTime(end);
                        combinedTime = horzcat(combinedTime, currentTrajectory); % Nối ngang
                    end
                end
                obj.TimeTrajectory = combinedTime;
            else
                obj.TimeTrajectory = [];
            end
        end
        % Phương thức để lưu các điểm quy hoạch vào file
        function saveTrajectoryToFile(obj, filename)
            % Tạo đường dẫn đầy đủ đến thư mục 'data' và tên file
            filepath = fullfile('data', filename);

            % Mở file để ghi, nếu file đã tồn tại thì sẽ ghi đè
            fileID = fopen(filepath, 'w');

            if fileID == -1
                error('Không thể mở file %s để ghi.', filepath);
            end
            % Get the current date and time
            currentDateTime = datetime("now");

            % Write the date and time at the beginning of the file
            fprintf(fileID, 'Date and Time: %s\n', currentDateTime);
            fprintf(fileID, '---------------------------------\n');
            % Lặp qua từng điểm trong quỹ đạo
            for i = 1:length(obj.PointTrajectory)
                % Lấy điểm hiện tại
                point = obj.PointTrajectory(i,:);

                % Ghi số thứ tự hàng và tọa độ X, Y, Z
                for j = 1:size(point, 1)
                    fprintf(fileID, 'N%d X%.6f Y%.6f Z%.6f\n', i, point(j, 1), point(j, 2), point(j, 3));
                end
            end

            % Đóng file
            fclose(fileID);
        end
    end
end
