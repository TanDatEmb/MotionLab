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

        % Phương thức để vẽ quỹ đạo
        function plotTrajectory(obj)
            obj.combineTrajectories();
            plotTrajectory(obj.PointTrajectory, obj.VelocityTrajectory, obj.AccelerationTrajectory, obj.OrientationTrajectory, obj.TimeTrajectory);
        end
    end
end
