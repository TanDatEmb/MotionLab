classdef MoveLinear
    properties
        % Vị trí
        StartPoint
        EndPoint
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

        % Cài đặt
        PlanningMethod
        % Giới hạn
        MaxDistance
        MaxVelocity
        MaxAcceleration
        MinDeltaTime
    end

    methods
        %% Khởi tạo
        function obj = MoveLinear(startPoint, endPoint, startVel, desVel, endVel, zone)
            % Đọc thông số cấu hình
            config = readConfig('config.txt');
            % Cập nhật thuộc tính từ cấu hình
            obj.PlanningMethod = config('trajectoryPlanningMethod');

            obj.MaxDistance = str2double(config('maxDistance'));
            obj.MaxVelocity = str2double(config('maxVelocity'));
            obj.MaxAcceleration = str2double(config('maxAcceleration'));
            obj.MinDeltaTime = str2double(config('minDeltaTime'));

            % Khởi tạo các thuộc tính
            obj.StartPoint = startPoint;
            obj.EndPoint = endPoint;

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
                    obj.quinticTrajectory();
                case 'JerkControl'
                    obj.jerkControlTrajectory();
                otherwise
                    error('Unsupported planning method.');
            end
        end

        function obj = lspbTrajectory(obj)
            %% Các Thông Số Đầu Vào
            startPoint = obj.StartPoint.Position;
            endPoint = obj.EndPoint.Position;
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
            distance = norm(endPoint - startPoint);

            % Kiểm tra và điều chỉnh vận tốc nếu cần thiết
            if desiredVelocity > maxVelocity
                desiredVelocity = maxVelocity;
            end

            % Thời gian tăng giảm tốc tối đa
            timeAccelMax = desiredVelocity / maxAcceleration;

            % Số bước thời gian tối đa cho tăng giảm tốc
            numStepsAccelMax = ceil(timeAccelMax / minDeltaTime);

            % Gia tốc thực tế
            actualAcceleration = desiredVelocity / timeAccelMax;

            % Tích phân gia tốc thực tế theo thời gian tối ưu để tính quãng đường tăng giảm tốc
            distanceAccel = 0.5 * actualAcceleration * timeAccelMax^2;

            % Thời gian vận tốc cố định
            distanceConstantVelocity = distance - 2 * distanceAccel;
            if distanceConstantVelocity < 0
                % Nếu quãng đường không đủ cho vận tốc cố định, điều chỉnh lại thời gian tăng/giảm tốc
                distanceConstantVelocity = 0;
                timeAccelMax = sqrt(distance / maxAcceleration);
                numStepsAccelMax = ceil(timeAccelMax / minDeltaTime);
                actualAcceleration = desiredVelocity / timeAccelMax;
                distanceAccel = 0.5 * actualAcceleration * timeAccelMax^2;
            end

            timeConstantVelocity = distanceConstantVelocity / desiredVelocity;
            totalTime = 2 * timeAccelMax + timeConstantVelocity;

            % Số bước thời gian tối đa cho vận tốc cố định
            numStepsConstantVelocity = ceil(timeConstantVelocity / minDeltaTime);

            %% Kết Hợp Các Yếu Tố Để Tính Vận Tốc và Vị Trí
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

            % Tính vận tốc và vị trí
            for i = 2:length(timeVector)
                deltaTime = timeVector(i) - timeVector(i-1);
                velocityProfile(i) = velocityProfile(i-1) + accelerationProfile(i-1) * deltaTime;
                pointProfile(i) = pointProfile(i-1) + velocityProfile(i-1) * deltaTime;
            end

            %% Chuyển Đổi Sang Vector 3D
            direction = (endPoint - startPoint) / distance;
            pointProfile3D = startPoint + pointProfile' .* direction;
            velocityProfile3D = velocityProfile' .* direction;
            accelerationProfile3D = accelerationProfile' .* direction;

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
