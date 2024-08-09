classdef MoveCircle
    properties
        % Vị trí
        StartPoint
        EndPoint
        MidPoint
        % Vận tốc
        StartVelocity
        DesiredVelocity
        EndVelocity
        % Vùng chuyển quỹ đạo
        SmoothZone

        % Tính toán
        DeltaTime
        TimeTrajectory
        PointTrajectory
        VelocityTrajectory
        AccelerationTrajectory
        OrientationTrajectory
        JerkTrajectory

        % Cài đặt
        PlanningMethod
        % Giới hạn
        MaxDistance
        MaxVelocity
        MaxAcceleration
        MinDeltaTime
        MaxJerk
    end

    methods
        %% Khởi tạo
        function obj = MoveCircle(startPoint, endPoint, midPoint, startVel, desVel, endVel, zone)
            % Đọc thông số cấu hình
            config = readConfig('config.txt');
            % Cập nhật thuộc tính từ cấu hình
            obj.PlanningMethod = config('trajectoryPlanningMethod');
            obj.MaxDistance = str2double(config('maxDistance'));
            obj.MaxVelocity = str2double(config('maxVelocity'));
            obj.MaxAcceleration = str2double(config('maxAcceleration'));
            obj.MinDeltaTime = str2double(config('minDeltaTime'));
            obj.MaxJerk = str2double(config('maxJerk'));

            % Khởi tạo các thuộc tính
            obj.StartPoint = startPoint;
            obj.EndPoint = endPoint;
            obj.MidPoint = midPoint;
            obj.StartVelocity = startVel;
            obj.DesiredVelocity = desVel;
            obj.EndVelocity = endVel;

            obj.SmoothZone = zone;

            obj = obj.planTrajectory();
        end

        %% Trajectory Planning Method
        function obj = planTrajectory(obj)
            switch obj.PlanningMethod
                case 'LSPB'
                    obj = obj.lspbTrajectory();
                case 'Quintic'
                    obj = obj.quinticTrajectory();
                case 'JerkControl'
                    obj = obj.jerkControlTrajectory();
                otherwise
                    error('Unsupported planning method.');
            end
        end
        %% LSPB
        function obj = lspbTrajectory(obj)
            %% Các Thông Số Đầu Vào
            startPos = obj.StartPoint.Position;
            endPos = obj.EndPoint.Position;
            midPos = obj.MidPoint.Position;

            startQuaternion = obj.StartPoint.OrientationQuaternion;
            endQuaternion = obj.EndPoint.OrientationQuaternion;
            startVelocity = obj.StartVelocity;
            desiredVelocity = obj.DesiredVelocity;
            endVelocity = obj.EndVelocity;
            maxVelocity = obj.MaxVelocity;
            maxAcceleration = obj.MaxAcceleration;
            minDeltaTime = obj.MinDeltaTime;
            smoothZone = obj.SmoothZone;

            %% Các Thông Số Tính Toán
            % Độ dài quãng đường
            [centerPos, radius, totalAngle, normalVector] = circleFromThreePoints(startPos, endPos, midPos);
            % Độ dài cung tròn
            arcLength = radius * abs(totalAngle);

            % Kiểm tra và điều chỉnh vận tốc nếu cần thiết
            if desiredVelocity > maxVelocity
                desiredVelocity = maxVelocity;
            end

            % Thời gian tăng giảm tốc tối đa
            timeAccelMax = desiredVelocity / maxAcceleration;

            if timeAccelMax < minDeltaTime
                error('Error: timeAccelMax')
            end

            % Số bước thời gian tối đa cho tăng giảm tốc
            numStepsAccelMax = floor(timeAccelMax / minDeltaTime);

            % Gia tốc thực tế
            actualAcceleration = desiredVelocity / timeAccelMax;

            % Tích phân gia tốc thực tế theo thời gian tối ưu để tính quãng đường tăng giảm tốc
            distanceAccel = 0.5 * actualAcceleration * timeAccelMax^2;

            % Thời gian vận tốc cố định
            distanceConstantVelocity = arcLength - 2 * distanceAccel;
            if distanceConstantVelocity < 0
                % Nếu quãng đường không đủ cho vận tốc cố định, điều chỉnh lại thời gian tăng/giảm tốc
                distanceConstantVelocity = 0;
                timeAccelMax = sqrt(arcLength / maxAcceleration);
                numStepsAccelMax = floor(timeAccelMax / minDeltaTime);
                actualAcceleration = maxAcceleration;
                distanceAccel = 0.5 * actualAcceleration * timeAccelMax^2;
            end

            timeConstantVelocity = distanceConstantVelocity / desiredVelocity;
            totalTime = 2 * timeAccelMax + timeConstantVelocity;

            % Số bước thời gian tối đa cho vận tốc cố định
            numStepsConstantVelocity = floor(timeConstantVelocity / minDeltaTime);

            %% Tổng hợp
            timeAccel = linspace(0, timeAccelMax, numStepsAccelMax);
            timeConstantVelocityArray = linspace(timeAccelMax, timeAccelMax + timeConstantVelocity, numStepsConstantVelocity);
            timeDecel = linspace(timeAccelMax + timeConstantVelocity, 2 * timeAccelMax + timeConstantVelocity, numStepsAccelMax);

            timeVector = [timeAccel, timeConstantVelocityArray(2:end), timeDecel(2:end)];

            accelerationProfile = zeros(1, length(timeVector));
            velocityProfile = zeros(1, length(timeVector));
            pointProfile = zeros(1, length(timeVector));

            % Gia tốc tăng tốc
            accelerationProfile(1:numStepsAccelMax) = actualAcceleration;

            % Gia tốc giữ tốc độ cố định
            accelerationProfile(numStepsAccelMax+1:numStepsAccelMax+numStepsConstantVelocity-1) = 0;

            % Gia tốc giảm tốc
            accelerationProfile(numStepsAccelMax+numStepsConstantVelocity:end) = -actualAcceleration;

            % Compute the trajectory
            for i = 2:length(timeVector)
                deltaTime = timeVector(i) - timeVector(i-1);
                if deltaTime < minDeltaTime
                    error('Error: deltaTime');
                end
                if timeVector(i) < timeAccelMax
                    % Acceleration phase
                    accelerationProfile(i) = actualAcceleration;
                    velocityProfile(i) = actualAcceleration * timeVector(i);
                    pointProfile(i) = 0 + 0.5 * actualAcceleration * timeVector(i)^2;
                elseif timeVector(i) < (timeAccelMax + timeConstantVelocity)
                    % Constant velocity phase
                    accelerationProfile(i) = 0;
                    velocityProfile(i) = desiredVelocity;
                    pointProfile(i) = distanceAccel + desiredVelocity * (timeVector(i) - timeAccelMax);
                else
                    % Deceleration phase
                    accelerationProfile(i) = -actualAcceleration;
                    velocityProfile(i) = desiredVelocity - actualAcceleration * (timeVector(i) - timeAccelMax - timeConstantVelocity);
                    pointProfile(i) = distanceAccel + distanceConstantVelocity + desiredVelocity * (timeVector(i) - timeAccelMax - timeConstantVelocity) - 0.5 * actualAcceleration * (timeVector(i) - timeAccelMax - timeConstantVelocity)^2;
                end
            end

            %% Chuyển Đổi Sang Vector 3D
            thetaProfile = pointProfile / radius;
            pointProfile3D = zeros(length(thetaProfile), 3);

            %% Chuyển đổi từ hệ tọa độ 2D trên mặt phẳng vuông góc với `normalVector` sang 3D
            % Định nghĩa trục tọa độ gốc trên mặt phẳng
            v1 = (startPos - centerPos) / norm(startPos - centerPos);
            v2 = cross(normalVector, v1);
            v2 = v2 / norm(v2);

            % Tính các điểm trên cung tròn
            for i = 1:length(thetaProfile)
                pointProfile3D(i, :) = centerPos + radius * (v1 * cos(thetaProfile(i)) + v2 * sin(thetaProfile(i)));
            end

            % Tính toán hướng chuyển động tại mỗi điểm
            diffPoints = diff([startPos; pointProfile3D]); % Tính toán điểm khác biệt
            norms = vecnorm(diffPoints, 2, 2); % Tính toán độ dài của các vector
            norms(norms == 0) = 1; % Tránh chia cho 0 bằng cách thay đổi độ dài vector bằng 1 nếu bằng 0
            directionProfile = diffPoints ./ norms; % Tính toán hướng và chuẩn hóa các vector

            % Kiểm tra và xử lý kích thước không phù hợp
            if length(velocityProfile) ~= size(directionProfile, 1)
                error('Error: Size mismatch between velocityProfile and directionProfile.');
            end

            % Cập nhật các profile vận tốc và gia tốc
            velocityProfile3D = velocityProfile' .* directionProfile; % Đảm bảo rằng kích thước của directionProfile và velocityProfile tương thích
            accelerationProfile3D = accelerationProfile' .* directionProfile; % Đảm bảo rằng kích thước của directionProfile và accelerationProfile tương thích


            %% Quy Hoạch Hướng Quaternions
            orientationTrajectory = slerpQuaternion(startQuaternion, endQuaternion, linspace(0, 1, length(timeVector)));
            
            %% Cập Nhật Các Thuộc Tính của Đối Tượng
            obj.TimeTrajectory = timeVector;
            obj.PointTrajectory = pointProfile3D;
            obj.VelocityTrajectory = velocityProfile3D;
            obj.AccelerationTrajectory = accelerationProfile3D;
            obj.OrientationTrajectory = orientationTrajectory;
        end
    end
end

